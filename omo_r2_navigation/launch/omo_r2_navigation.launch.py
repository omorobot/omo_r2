import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    navigation_dir = get_package_share_directory('r2mini_navigation')

    map = LaunchConfiguration('map', default=os.path.join(navigation_dir, 'map', 'map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(navigation_dir, 'param', os.environ['ROBOT_MODEL'] + '.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_arg = DeclareLaunchArgument('map', default_value=map)
    params_file_arg = DeclareLaunchArgument('params_file', default_value=params_file)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/bringup_launch.py']),
        launch_arguments={'map': map, 'params_file': params_file, 'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()
    ld.add_action(map_arg)
    ld.add_action(params_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(navigation_node)

    return ld
