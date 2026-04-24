"""
single_odom.launch.py
=====================

Escenario A — Robot único, sin localizador (solo odometría).

Árbol TF esperado en ROS:
  <tf_prefix>/odom → <tf_prefix>/base_footprint

TF publicada:
  world → <tf_prefix>/odom   (corrección de deriva de odometría)

Uso (valores por defecto para caddy_ai2):
  ros2 launch gz_ground_truth single_odom.launch.py

Uso con robot personalizado:
  ros2 launch gz_ground_truth single_odom.launch.py \\
      gz_model_name:=my_robot tf_prefix:=my_robot \\
      localization_frame:=my_robot/odom
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

    # Si localization_frame está vacío, derivarlo como <tf_prefix>/odom
    if not loc_frame:
        loc_frame = f'{tf_prefix}/odom'

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
        DeclareLaunchArgument('localization_frame', default_value='',
                              description='Frame a corregir (vacío → <tf_prefix>/odom)'),
        DeclareLaunchArgument('ground_truth_frame', default_value='world',
                              description='Frame de ground truth (origen del mundo)'),
        DeclareLaunchArgument('publish_rate',       default_value='50.0',
                              description='Frecuencia de publicación (Hz)'),
        OpaqueFunction(function=_create_node),
    ])
