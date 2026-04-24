"""
multi_odom.launch.py
====================

Escenario C — Multi-robot, sin localizador (corrección de odometría por robot).

Árbol TF esperado en ROS:
  robot1/odom → robot1/base_footprint
  robot2/odom → robot2/base_footprint

TFs publicadas:
  world → robot1/odom
  world → robot2/odom

Uso con yaml (recomendado):
  ros2 launch gz_ground_truth multi_odom.launch.py

  El launch carga por defecto config/caddy_ai2_multi_odom.yaml.
  Para usar otro yaml:
    ros2 launch gz_ground_truth multi_odom.launch.py \\
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
    'config', 'caddy_ai2_multi_odom.yaml',
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
