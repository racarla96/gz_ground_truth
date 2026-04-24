"""
single_map.launch.py
====================

Escenario B — Robot único con localizador activo (AMCL, Nav2, etc.).

Árbol TF esperado en ROS:
  map → <tf_prefix>/odom → <tf_prefix>/base_footprint

TF publicada:
  world → map   (error del localizador respecto al ground truth)

Uso (valores por defecto para caddy_ai2):
  ros2 launch gz_ground_truth single_map.launch.py

Uso con mapa por robot:
  ros2 launch gz_ground_truth single_map.launch.py \\
      gz_model_name:=my_robot tf_prefix:=my_robot \\
      localization_frame:=my_robot/map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_node(context, *args, **kwargs):
    gz_model_name: str = LaunchConfiguration('gz_model_name').perform(context)
    tf_prefix_raw: str = LaunchConfiguration('tf_prefix').perform(context)
    tf_prefix = tf_prefix_raw if tf_prefix_raw else gz_model_name
    loc_frame: str = LaunchConfiguration('localization_frame').perform(context)
    gt_frame:  str = LaunchConfiguration('ground_truth_frame').perform(context)
    rate:      str = LaunchConfiguration('publish_rate').perform(context)

    return [Node(
        package='gz_ground_truth',
        executable='ground_truth_tf_publisher',
        name='ground_truth_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':       True,
            'multi_robot':        False,
            'gz_model_name':      gz_model_name,
            'tf_prefix':          tf_prefix,
            'localization_frame': loc_frame,
            'ground_truth_frame': gt_frame,
            'base_suffix':        'base_footprint',
            'publish_rate':       float(rate),
        }],
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('gz_model_name',      default_value='caddy_ai2',
                              description='Nombre del modelo en Gazebo'),
        DeclareLaunchArgument('tf_prefix',          default_value='',
                              description='Prefijo TF del robot (vacío → usa gz_model_name)'),
        DeclareLaunchArgument('localization_frame', default_value='map',
                              description='Frame a corregir (map global o robot/map por robot)'),
        DeclareLaunchArgument('ground_truth_frame', default_value='world',
                              description='Frame de ground truth (origen del mundo)'),
        DeclareLaunchArgument('publish_rate',       default_value='50.0',
                              description='Frecuencia de publicación (Hz)'),
        OpaqueFunction(function=_create_node),
    ])
