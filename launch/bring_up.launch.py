from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

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

        node_params = cfg.get('solution1', {})
        
        rviz_path = cfg.get('rviz_config', '')
        rviz_config = cfg.get('rviz_config', rviz_path)
        bag_path = cfg.get('bag_path', '')


        # Если rviz_config/ bag_path заданы относительными путями — дополним их до пути внутри пакета
        if rviz_config and not os.path.isabs(rviz_config):
            rviz_config = os.path.join(pkg_share, rviz_config)
        if bag_path and not os.path.isabs(bag_path):
            bag_path = os.path.join(pkg_share, bag_path)

        # Небольшая проверка существования bag (и наличия metadata.yaml)
        if bag_path:
            meta = os.path.join(bag_path, 'metadata.yaml')
            if not os.path.exists(meta):
                print(f"[star_solution] WARNING: Bag path '{bag_path}' не найден или отсутствует metadata.yaml")

        solution_node = Node(
            package='star_solution',
            executable='solution1',
            name='solution1',
            output='screen',
            parameters=[node_params]
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )

        bag_process = None
        if bag_path:
            bag_process = TimerAction(
                period=1.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'bag', 'play', bag_path],
                        output='screen',
                        shell=False
                    )
                ]
            )

        actions = [solution_node, rviz_node]
        if bag_process:
            actions.append(bag_process)
        return actions

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=setup_nodes)
    ])
