"""
multi_shared_map.launch.py
==========================

Escenario E — Multi-robot, mapa global compartido.

Árbol TF esperado en ROS:
  map → robot1/odom → robot1/base_footprint
        robot2/odom → robot2/base_footprint

TFs publicadas:
  world → map          (corrección del mapa compartido; robot1 actúa de maestro)
  world → robot2/odom  (corrección residual de odometría de robot2 vs map)

Uso con yaml (recomendado):
  ros2 launch gz_ground_truth multi_shared_map.launch.py

  El launch carga por defecto config/caddy_ai2_multi_shared_map.yaml.
  Para usar otro yaml:
    ros2 launch gz_ground_truth multi_shared_map.launch.py \\
        params_file:=/ruta/a/mi_config.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_YAML = os.path.join(
    get_package_share_directory('gz_ground_truth'),
    'config', 'caddy_ai2_multi_shared_map.yaml',
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=_DEFAULT_YAML,
                              description='Ruta al yaml de parámetros multi-robot'),
        Node(
            package='gz_ground_truth',
            executable='ground_truth_tf_publisher',
            name='ground_truth_tf_publisher',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
