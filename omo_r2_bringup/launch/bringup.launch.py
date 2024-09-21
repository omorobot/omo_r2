import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('omo_r2_bringup')
    description_dir = get_package_share_directory('omo_r2_description')
    robot_dir = get_package_share_directory('omo_r2_robot')

    lidar_yaml = LaunchConfiguration('lidar_yaml', default=os.path.join(bringup_dir, 'param', os.environ['LIDAR_MODEL'] + '.yaml'))
    robot_yaml = LaunchConfiguration('robot_yaml', default=os.path.join(robot_dir, 'param', os.environ['ROBOT_MODEL'] + '.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    lidar_yaml_arg = DeclareLaunchArgument('lidar_yaml', default_value=lidar_yaml)
    robot_yaml_arg = DeclareLaunchArgument('robot_yaml', default_value=robot_yaml)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    with open(os.path.join(description_dir, 'urdf', os.environ['ROBOT_MODEL'] + '.urdf'), 'r') as infp:
        robot_description = infp.read()

    robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_dir, '/launch/robot.launch.py']),
        launch_arguments={'robot_yaml': robot_yaml}.items()
    )

    lidar_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_yaml],
        namespace='/',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(lidar_yaml_arg)
    ld.add_action(robot_yaml_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_node)
    ld.add_action(lidar_node)
    ld.add_action(robot_state_publisher_node)

    return ld
