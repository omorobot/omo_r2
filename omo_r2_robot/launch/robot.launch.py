import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_dir = get_package_share_directory('omo_r2_robot')

    robot_yaml = LaunchConfiguration('robot_yaml', default=os.path.join(robot_dir, 'param', os.environ['ROBOT_MODEL'] + '.yaml'))

    robot_yaml_arg = DeclareLaunchArgument('robot_yaml', default_value=robot_yaml)
    
    robot_control_node = Node(
        package='omo_r2_robot',
        executable='robot_control',
        name='robot_control',
        emulate_tty=True,
        parameters=[robot_yaml],
        namespace='',
        output='screen',
    )
    
    ld = LaunchDescription()
    ld.add_action(robot_yaml_arg)
    ld.add_action(robot_control_node)
    
    return ld
