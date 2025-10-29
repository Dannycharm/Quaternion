from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, Command, FindExecutable


def generate_launch_description():
    pkg = get_package_share_directory('husky_ur3_bringup')

    # Use the combined model you prepared
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'husky_ur3_gripper.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg, 'config', 'controllers.yaml'])

    # Render xacro → XML string ONCE
    robot_xml_cmd = Command([FindExecutable(name='xacro'), ' ', xacro_file])

    # Publish /robot_description as a string param (for RViz/TF)
    robot_description = ParameterValue(robot_xml_cmd, value_type=str)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output='screen'
    )

    # Set Gazebo Sim environment variables
    set_gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH'), ':/opt/ros/jazzy/lib']
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[EnvironmentVariable('GZ_SIM_RESOURCE_PATH'), ':',
               '/home/praise/mm_ws/install/husky_ur3_bringup/share']
    )


   # --- ADD CLOCK BRIDGE HERE (It must come before spawners) ---
    # clock_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    #     output='screen'
    # )
    # delayed start, connects reliably
    clock_bridge = TimerAction(
    period=3.0,
    actions=[Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )]
)
    # -------------------------------------------------------------
    
    # Start Gazebo Sim
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', 'empty.sdf'],
        output='screen'
    )

    # Spawn the robot into Gazebo world
    spawn = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'empty',
                '-string', robot_xml_cmd,
                '-name', 'husky_ur3'
            ],
            output='screen'
        )]
    )

    # Controller manager namespace for the robot entity
    # cm = '/husky_ur3/controller_manager'
    cm = '/controller_manager'

    # Spawn controller processes
    jsb = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster',
             '--controller-manager', cm,
             '--param-file', controllers_yaml,
             '--controller-manager-timeout', '15'],
        output='screen'
    )
    base = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'base_controller',
             '--controller-manager', cm,
             '--param-file', controllers_yaml,
             '--controller-manager-timeout', '15'],
        output='screen'
    )
    arm = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'arm_controller',
             '--controller-manager', cm,
             '--param-file', controllers_yaml,
             '--controller-manager-timeout', '15'],
        output='screen'
    )
    grip = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'gripper_controller',
             '--controller-manager', cm,
             '--param-file', controllers_yaml,
             '--controller-manager-timeout', '15'],
        output='screen'
    )

    # Delay controller spawning to ensure robot is loaded first
    delayed_spawners = TimerAction(period=6.0, actions=[jsb, base, arm, grip])

    # Return the complete launch description
     # Final launch order: env → RSP → clock → Gazebo → spawn → spawners
    return LaunchDescription([
        set_gz_plugin_path,
        set_gz_resource_path,
        rsp,
        clock_bridge,
        gz,
        spawn,
        delayed_spawners
    ])




# from launch import LaunchDescription
# from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from ament_index_python.packages import get_package_share_directory
# from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, Command, FindExecutable

# def generate_launch_description():
#     pkg = get_package_share_directory('husky_ur3_bringup')

#     # Use the combined model you prepared
#     xacro_file = PathJoinSubstitution([pkg, 'urdf', 'husky_ur3_gripper.urdf.xacro'])
#     controllers_yaml = PathJoinSubstitution([pkg, 'config', 'controllers.yaml'])

#     # Render xacro -> XML string ONCE, reuse it for both RSP and create
#     # (Keep the explicit space tokens so 'xacro' and its args are separated)
#     robot_xml_cmd = Command([FindExecutable(name='xacro'), ' ', xacro_file])


#     # Publish /robot_description as a string param (for RViz/TF)
#     robot_description = ParameterValue(robot_xml_cmd, value_type=str)
#     rsp = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
#         output='screen'
#     )

#     # Start Gazebo Sim (GUI ok; world name will be 'empty')
#     gz = ExecuteProcess(
#         cmd=['gz', 'sim', '-r', '-v', '3', 'empty.sdf'],
#         output='screen'
#     )

#     # Spawn the robot: IMPORTANT -> use the correct world name: 'empty'
#     # Feed the model XML directly via -string (no temp files, no topics)
#     spawn = TimerAction(
#         period=2.0,
#         actions=[ExecuteProcess(
#             cmd=[
#                 'ros2', 'run', 'ros_gz_sim', 'create',
#                 '-world', 'empty',
#                 '-string', robot_xml_cmd,
#                 '-name', 'husky_ur3'
#             ],
#             output='screen'
#         )]
#     )

#     # Spawn controllers after robot entity exists; give a few seconds
#     cm = '/husky_ur3/controller_manager'  # manager lives under the model ns
    
#     jsb = ExecuteProcess(
#         cmd=['ros2','run','controller_manager','spawner','joint_state_broadcaster',
#              '--controller-manager', cm,
#              '--param-file', controllers_yaml,
#              '--controller-manager-timeout','15'],
#         output='screen'
#     )
#     base = ExecuteProcess(
#         cmd=['ros2','run','controller_manager','spawner','base_controller',
#              '--controller-manager', cm,
#              '--param-file', controllers_yaml,
#              '--controller-manager-timeout','15'],
#         output='screen'
#     )
#     arm = ExecuteProcess(
#         cmd=['ros2','run','controller_manager','spawner','arm_controller',
#              '--controller-manager', cm,
#              '--param-file', controllers_yaml,
#              '--controller-manager-timeout','15'],
#         output='screen'
#     )
#     grip = ExecuteProcess(
#         cmd=['ros2','run','controller_manager','spawner','gripper_controller',
#              '--controller-manager', cm,
#              '--param-file', controllers_yaml,
#              '--controller-manager-timeout','15'],
#         output='screen'
#     )
    

#     delayed_spawners = TimerAction(period=6.0, actions=[jsb, base, arm, grip])

#     return LaunchDescription([rsp, gz, spawn, delayed_spawners])

