import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml_config(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_share = get_package_share_directory('star_solution')

    default_config = os.path.join(pkg_share, 'config', 'config.yaml')
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to YAML with parameters (topics, rviz config, bag path).'
    )

    def setup_nodes(context, *args, **kwargs):
        config_path = LaunchConfiguration('config_file').perform(context)
        cfg = load_yaml_config(config_path) or {}

        # === Параметры из YAML ===
        node_params = cfg.get('solution1', {})
        rviz_config = cfg.get('rviz_config', '')
        bag_path = cfg.get('bag_path', '')
        carto_params = cfg.get('cartographer', {})

        # абсолютные пути
        if rviz_config and not os.path.isabs(rviz_config):
            rviz_config = os.path.join(pkg_share, rviz_config)
        if bag_path and not os.path.isabs(bag_path):
            bag_path = os.path.join(pkg_share, bag_path)

        if bag_path:
            meta = os.path.join(bag_path, 'metadata.yaml')
            if not os.path.exists(meta):
                print(f"[star_solution] WARNING: Bag path '{bag_path}' не найден или отсутствует metadata.yaml")

        # === Cartographer nodes ===
        config_dir = os.path.join(pkg_share, "config")
        config_basename = "cartographer_2d.lua"

        carto_node = Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[carto_params],
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
            executable="cartographer_occupancy_grid_node",
            name="occupancy_grid_node",
            output="screen",
            parameters=[carto_params],
            arguments=[
                "-resolution", "0.05",
                "-publish_period_sec", "1.0",
            ],
        )

        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="livox_tf_pub",
            arguments=["0", "0", "0.3",
                    "-0.70711", "0.70711", "4.3298e-17", "4.3298e-17",
                    "base_link", "livox_frame"]
        )

        # === solution1 node ===
        solution_node = Node(
            package='star_solution',
            executable='solution1',
            name='solution1',
            output='screen',
            parameters=[node_params]
        )

        # === RViz node ===
        rviz_node = None
        if rviz_config:
            rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config]
            )

        # === Bag playback ===
        bag_process = None
        if bag_path:
            bag_process = TimerAction(
                period=0.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
                        output='screen',
                        shell=False
                    )
                ]
            )

        # Собираем все действия
        actions = [carto_node, occ_node, solution_node, static_tf]
        if rviz_node:
            actions.append(rviz_node)
        if bag_process:
            actions.append(bag_process)

        return actions

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=setup_nodes)
    ])
