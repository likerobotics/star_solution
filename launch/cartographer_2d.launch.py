import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    config_dir = LaunchConfiguration(
        "configuration_directory",
        default=os.path.join(
            get_package_share_directory("star_solution"), "config"
        ),
    )
    config_basename = LaunchConfiguration(
        "configuration_basename", default="cartographer_2d.lua"
    )

    carto_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory", config_dir,
            "-configuration_basename", config_basename,
        ],
        remappings=[
            ("points2", "/livox/lidar"),
            ("imu", "/livox/imu"),
        ],
    )

    occ_node = Node(
        package="cartographer_ros",
        executable="occupancy_grid_node",
        name="occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-resolution", "0.05",
            "-publish_period_sec", "1.0",
        ],
    )

    return LaunchDescription([carto_node, occ_node])
