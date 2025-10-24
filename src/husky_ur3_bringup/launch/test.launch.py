from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('husky_ur3_bringup')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'husky_ur3_gripper.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg, 'config', 'controllers.yaml'])

    # Render xacro (note the explicit space between exe and path)
    xacro_cmd = Command([FindExecutable(name='xacro'), ' ', xacro_file])
    robot_description = ParameterValue(xacro_cmd, value_type=str)


    # 2) Publish /robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output='screen'
    )

    # 3) Start Gazebo Sim (Harmonic)
    gz = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py'],
        output='screen'
    )

    # 4) Spawn from /robot_description
    spawn = ExecuteProcess(
        cmd=['ros2','run','ros_gz_sim','create',
             '-world','default',
             '-topic','robot_description',
             '-name','husky_ur3'],
        output='screen'
    )

    # 5) Controllers (delay so controller_manager is up)
    jsb = ExecuteProcess(
        cmd=['ros2','run','controller_manager','spawner','joint_state_broadcaster',
             '--controller-manager','/controller_manager',
             '--param-file', controllers_yaml,
             '--controller-manager-timeout','10'],
        output='screen'
    )
    base = ExecuteProcess(
        cmd=['ros2','run','controller_manager','spawner','base_controller',
             '--controller-manager','/controller_manager',
             '--param-file', controllers_yaml,
             '--controller-manager-timeout','10'],
        output='screen'
    )
    arm = ExecuteProcess(
        cmd=['ros2','run','controller_manager','spawner','arm_controller',
             '--controller-manager','/controller_manager',
             '--param-file', controllers_yaml,
             '--controller-manager-timeout','10'],
        output='screen'
    )
    grip = ExecuteProcess(
        cmd=['ros2','run','controller_manager','spawner','gripper_controller',
             '--controller-manager','/controller_manager',
             '--param-file', controllers_yaml,
             '--controller-manager-timeout','10'],
        output='screen'
    )

    delayed_spawners = TimerAction(period=2.0, actions=[jsb, base, arm, grip])

    return LaunchDescription([rsp, gz, spawn, delayed_spawners])

