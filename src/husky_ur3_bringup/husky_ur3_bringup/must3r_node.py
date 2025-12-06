#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import message_filters
import torch
import numpy as np
import cv2
import PIL.Image
import struct
import os

# --- MUST3R Imports based on your repo structure ---
from must3r.model import load_model
from must3r.engine.inference import inference_multi_ar, postprocess
from must3r.model import get_pointmaps_activation
from dust3r.datasets import ImgNorm
from must3r.tools.image import get_resize_function

class Must3rNode(Node):
    def __init__(self):
        super().__init__('must3r_node')

        # --- Parameters ---
        self.declare_parameter('camera1_topic', '/h_camera/depth/image/image') # Base camera
        self.declare_parameter('camera2_topic', '/camera/depth/image/image')   # Arm camera
        self.declare_parameter('output_topic', '/must3r/points')
        self.declare_parameter('weights_path', '/home/praise/mm_ws/MUSt3R_512.pth') # UPDATE THIS PATH
        self.declare_parameter('image_size', 512)
        
        cam1_topic = self.get_parameter('camera1_topic').get_parameter_value().string_value
        cam2_topic = self.get_parameter('camera2_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        weights_path = self.get_parameter('weights_path').get_parameter_value().string_value
        self.img_size = self.get_parameter('image_size').get_parameter_value().integer_value

        # --- Setup MUST3R Model ---
        self.get_logger().info("Loading MUST3R model...")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {self.device}")

        if not os.path.exists(weights_path):
            self.get_logger().error(f"Weights file not found at: {weights_path}")
            # Fallback or exit? Continuing might crash.
        
        # Load model using the repo's load_model function
        self.model = load_model(weights_path, device=self.device, img_size=self.img_size, verbose=True)
        self.encoder, self.decoder = self.model
        
        # Pre-calculate activation for post-processing
        self.pointmaps_activation = get_pointmaps_activation(self.decoder, verbose=False)

        # --- ROS Setup ---
        self.bridge = CvBridge()
        self.pub_cloud = self.create_publisher(PointCloud2, out_topic, 10)

        # Synchronized Subscribers
        self.sub1 = message_filters.Subscriber(self, Image, cam1_topic)
        self.sub2 = message_filters.Subscriber(self, Image, cam2_topic)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub1, self.sub2], 
            queue_size=2, 
            slop=0.1 
        )
        self.ts.registerCallback(self.callback)

        self.get_logger().info("MUST3R Node Initialized.")

    def callback(self, img1_msg, img2_msg):
        self.get_logger().info("Processing image pair...")
        
        try:
            # 1. Convert ROS -> PIL Image
            # We use PIL because that's what the repo's preprocessing expects
            cv_img1 = self.bridge.imgmsg_to_cv2(img1_msg, desired_encoding='rgb8')
            cv_img2 = self.bridge.imgmsg_to_cv2(img2_msg, desired_encoding='rgb8')
            
            pil_imgs = [PIL.Image.fromarray(cv_img1), PIL.Image.fromarray(cv_img2)]
            
            # 2. Preprocess Images (Resize & Normalize)
            # Adapted from load_images() in inference.py
            imgs_tensor = []
            true_shapes = []
            transform = ImgNorm

            for img in pil_imgs:
                W, H = img.size
                # Using the repo's resize helper
                resize_func, _, _ = get_resize_function(self.img_size, self.encoder.patch_size, H, W)
                rgb_tensor = resize_func(transform(img)) # (3, H_new, W_new)
                imgs_tensor.append(rgb_tensor)
                true_shapes.append(np.int32([rgb_tensor.shape[-2], rgb_tensor.shape[-1]]))

            # 3. Batch data for Inference
            # MUST3R inference expects list of tensors on device
            imgs = [img.to(self.device) for img in imgs_tensor]
            true_shapes = [torch.from_numpy(s).to(self.device) for s in true_shapes]
            # Do NOT stack true_shapes. inference_multi_ar expects a list.
            
            # IDs are required by inference_multi_ar
            img_ids = [torch.tensor(i) for i in range(len(imgs))]

            # Memory batches config (simplified for 2 views)
            mem_batches = [2] 

            # 4. Run Inference
            with torch.no_grad():
                x_out_0, x_out = inference_multi_ar(
                    self.encoder, self.decoder, 
                    imgs, img_ids, true_shapes, # Pass the LIST, not the stack
                    mem_batches=mem_batches,
                    max_bs=1, 
                    verbose=False,
                    device=self.device,
                    preserve_gpu_mem=True,
                    post_process_function=lambda x: postprocess(x, pointmaps_activation=self.pointmaps_activation, compute_cam=True)
                )

            # 5. Extract Point Cloud
            # x_out is a list of dicts, one per view. We take the first view (Base Camera) as reference.
            res = x_out[0] # View 0
            
            # pts3d is usually (H, W, 3)
            pts3d = res['pts3d'].cpu().numpy()
            
            # Filter by confidence if needed (conf is (H, W))
            conf = res['conf'].cpu().numpy()
            mask = conf > 1.5 # Threshold from demo defaults
            
            # Flatten
            points_flat = pts3d[mask]
            
            # Get colors from original resized image (denormalize if needed, or just use original)
            # The tensor 'imgs[0]' is normalized. Let's use the original cv_img1 for colors.
            # But wait, points correspond to the RESIZED tensor coordinates.
            # Easier: use the tensor, denormalize, then mask.
            
            # Simple denorm for visualization: (img * 0.5 + 0.5) roughly if ImgNorm was standard
            # Better: interpolate original image to match point cloud size?
            # Actually, x_out resolution matches 'imgs[0]' resolution.
            
            # Let's just use the normalized tensor for color for now (it will look okay-ish, maybe contrasty)
            # Or better, just map the original CV image.
            # For simplicity in this assignment: use the tensor.
            rgb_tensor = imgs[0].permute(1, 2, 0).cpu().numpy() # (H, W, 3)
            # Denormalize: ImgNorm usually does (x - 0.5)/0.5. So x * 0.5 + 0.5
            rgb_tensor = (rgb_tensor * 0.5) + 0.5
            rgb_tensor = np.clip(rgb_tensor * 255, 0, 255).astype(np.uint8)
            
            colors_flat = rgb_tensor[mask]

            # 6. Publish
            # Use frame_id of the FIRST camera (Base Camera)
            header = img1_msg.header
            header.frame_id = "h_camera_link" # Ensure this matches TF
            self.publish_pointcloud(points_flat, colors_flat, header)

        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")
            import traceback
            traceback.print_exc()

    def publish_pointcloud(self, points, colors, header):
        if len(points) == 0:
            return

        cloud_data = np.zeros(points.shape[0], dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('rgb', np.float32)
        ])
        
        cloud_data['x'] = points[:, 0]
        cloud_data['y'] = points[:, 1]
        cloud_data['z'] = points[:, 2]
        
        # Pack RGB
        rgb_int = (colors[:, 0].astype(np.uint32) << 16) | \
                  (colors[:, 1].astype(np.uint32) << 8)  | \
                  (colors[:, 2].astype(np.uint32))
        
        cloud_data['rgb'] = rgb_int.view(np.float32)

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        msg.data = cloud_data.tobytes()

        self.pub_cloud.publish(msg)
        self.get_logger().info(f"Published PointCloud with {points.shape[0]} points.")

def main(args=None):
    rclpy.init(args=args)
    node = Must3rNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2, PointField
# from cv_bridge import CvBridge
# import message_filters
# import torch
# import numpy as np
# import cv2
# import struct

# # Import MUST3R
# # Note: The import might vary slightly depending on how the package installs.
# # Common import for the repo:
# # from must3r import MUST3R
# from must3r.model import MUST3R

# class Must3rNode(Node):
#     def __init__(self):
#         super().__init__('must3r_node')

#         # --- Parameters ---
#         self.declare_parameter('camera1_topic', '/h_camera/depth/image/image') # Base camera
#         self.declare_parameter('camera2_topic', '/camera/depth/image/image')   # Arm camera
#         self.declare_parameter('output_topic', '/must3r/points')
        
#         cam1_topic = self.get_parameter('camera1_topic').get_parameter_value().string_value
#         cam2_topic = self.get_parameter('camera2_topic').get_parameter_value().string_value
#         out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

#         # --- Setup MUST3R Model ---
#         self.get_logger().info("Loading MUST3R model...")
#         device = 'cuda' if torch.cuda.is_available() else 'cpu'
#         self.get_logger().info(f"Using device: {device}")
        
#         # Load model as per rubric snippet
#         self.model = MUST3R.from_pretrained(
#             "naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt"
#         ).to(device)
#         self.model.eval()
#         self.device = device

#         # --- ROS Setup ---
#         self.bridge = CvBridge()
#         self.pub_cloud = self.create_publisher(PointCloud2, out_topic, 10)

#         # Synchronized Subscribers
#         # We use ApproximateTimeSynchronizer because cameras might not be perfectly synced in sim
#         self.sub1 = message_filters.Subscriber(self, Image, cam1_topic)
#         self.sub2 = message_filters.Subscriber(self, Image, cam2_topic)
        
#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.sub1, self.sub2], 
#             queue_size=10, 
#             slop=0.1 # Allow 0.1s difference
#         )
#         self.ts.registerCallback(self.callback)

#         self.get_logger().info("MUST3R Node Initialized.")

#     def callback(self, img1_msg, img2_msg):
#         self.get_logger().info("Received image pair. Processing...")
        
#         try:
#             # 1. Convert ROS Images to OpenCV/Numpy
#             # Use 'bgr8' for OpenCV compatibility
#             cv_img1 = self.bridge.imgmsg_to_cv2(img1_msg, desired_encoding='rgb8')
#             cv_img2 = self.bridge.imgmsg_to_cv2(img2_msg, desired_encoding='rgb8')

#             # 2. Prepare for MUST3R (Resize to 512x512 as per rubric recommendation for speed/memory)
#             # MUST3R expects list of images (H, W, 3)
#             # Ideally keep aspect ratio or crop, but simple resize is often "good enough" for demo
#             # Or better: keep original if GPU allows. Let's start with original or moderate resize.
#             # The rubric mentions 512 resolution configuration.
            
#             # Simple list of numpy arrays
#             images_list = [cv_img1, cv_img2]

#             # 3. Run Inference
#             with torch.no_grad():
#                 # The API call depends on the specific MUST3R wrapper. 
#                 # Based on rubric: pointmaps, confidences = self.model(images)
#                 # Note: 'images' usually needs to be loaded/preprocessed slightly.
#                 # Standard MUST3R inference often takes file paths or PIL images.
#                 # Assuming direct forward pass accepts tensor batch or list of arrays handled by wrapper.
                
#                 # If the wrapper requires file paths, we might need to save temp files.
#                 # Assuming the 'MUST3R' class handles raw data:
#                 results = self.model(images_list) 
#                 # Usually returns a list of pointmaps (one per view)
                
#                 # Let's assume results[0] is the pointmap for view 1, results[1] for view 2
#                 # A pointmap is (H, W, 3) tensor of 3D coordinates
#                 pointmap1 = results[0]['pts3d'] if isinstance(results, list) else results 
                
#                 # If output is tuple (pointmaps, confidences), adjust accordingly.
#                 # Rubric says: pointmaps, confidences = self.model(images)
#                 if isinstance(results, tuple):
#                      pointmaps, confidences = results
#                      pointmap1 = pointmaps[0] # Use the first view's reconstruction

#             # 4. Convert to PointCloud2
#             # pointmap1 should be a tensor on GPU/CPU. Move to CPU numpy.
#             if isinstance(pointmap1, torch.Tensor):
#                 points = pointmap1.detach().cpu().numpy() # (H, W, 3)
#             else:
#                 points = pointmap1

#             # Flatten to (N, 3)
#             h, w, c = points.shape
#             points_flat = points.reshape(-1, 3)
            
#             # Get colors from original image for the point cloud
#             colors_flat = cv_img1.reshape(-1, 3) # RGB

#             self.publish_pointcloud(points_flat, colors_flat, img1_msg.header)

#         except Exception as e:
#             self.get_logger().error(f"Error in callback: {e}")

#     def publish_pointcloud(self, points, colors, header):
#         # Create PointCloud2 message manually or use convenience function
#         # Fields: x, y, z, rgb
        
#         # Simple packing
#         # Points: float32, Colors: float32 (packed RGB) or uint8
#         # Efficient packing often requires struct or numpy tricks
        
#         # Let's use a simplified approach (slower but readable) or a standard library if available.
#         # Here is a structured numpy approach for speed:
        
#         cloud_data = np.zeros(points.shape[0], dtype=[
#             ('x', np.float32), ('y', np.float32), ('z', np.float32),
#             ('rgb', np.float32)
#         ])
        
#         cloud_data['x'] = points[:, 0]
#         cloud_data['y'] = points[:, 1]
#         cloud_data['z'] = points[:, 2]
        
#         # Pack RGB
#         # (r << 16) | (g << 8) | b
#         rgb_int = (colors[:, 0].astype(np.uint32) << 16) | \
#                   (colors[:, 1].astype(np.uint32) << 8)  | \
#                   (colors[:, 2].astype(np.uint32))
        
#         # View as float32 for ROS message
#         cloud_data['rgb'] = rgb_int.view(np.float32)

#         msg = PointCloud2()
#         msg.header = header
#         msg.header.frame_id = "h_camera_link" # Adjust to match your camera frame!
#         msg.height = 1
#         msg.width = points.shape[0]
#         msg.fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#             PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
#         ]
#         msg.is_bigendian = False
#         msg.point_step = 16
#         msg.row_step = msg.point_step * points.shape[0]
#         msg.is_dense = True # MUST3R outputs dense map usually
#         msg.data = cloud_data.tobytes()

#         self.pub_cloud.publish(msg)
#         self.get_logger().info("Published PointCloud2.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = Must3rNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

