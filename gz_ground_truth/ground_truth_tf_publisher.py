"""
ground_truth_tf_publisher.py
============================

Publica la TF de corrección:

    ground_truth_frame → localization_frame

calculada como:

    T_world_loc = T_world_base · inv(T_loc_base)

donde:
  - T_world_base  : pose exacta del robot en world (Gazebo ground truth)
  - T_loc_base    : pose del robot en el frame raíz de localización (árbol TF de ROS)
  - T_world_loc   : corrección a publicar en /tf

Escenarios soportados
---------------------
1. Sin localizador
     Árbol TF: odom → robot/base_footprint
     localization_frame = 'robot/odom'
     TF publicada: world → robot/odom   (deriva de la odometría)

2. Con localizador (map por robot o map compartido)
     Árbol TF: [robot/]map → robot/odom → robot/base_footprint
     localization_frame = '[robot/]map'
     TF publicada: world → [robot/]map  (error del localizador respecto al GT)

Parámetros ROS
--------------
# ── Modo robot único ──────────────────────────────────────────────────────
multi_robot         (bool)     false
gz_model_name       (string)   'caddy_ai2'
tf_prefix           (string)   ''               (vacío → usa gz_model_name)
localization_frame  (string)   'caddy_ai2/odom'
# ── Modo multi-robot ──────────────────────────────────────────────────────
robots              (string[]) Lista de tríos 'gz_model_name:tf_prefix:localization_frame'
# ── Comunes ───────────────────────────────────────────────────────────────
ground_truth_frame  (string)   'world'
base_suffix         (string)   'base_footprint'
publish_rate        (double)   50.0
use_sim_time        (bool)     true
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from tf2_ros import (
    TransformBroadcaster,
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose


# ── Math helpers ──────────────────────────────────────────────────────────────
# Pure-Python quaternion ops; no extra deps beyond rclpy.

def _quat_mult(q1: tuple, q2: tuple) -> tuple:
    """Hamilton product for (x, y, z, w) unit quaternions."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def _quat_conj(q: tuple) -> tuple:
    """Conjugate (= inverse) of a unit quaternion (x, y, z, w)."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def _rotate_vec(q: tuple, v: tuple) -> tuple:
    """Rotate vector v = (x, y, z) by unit quaternion q = (x, y, z, w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # Rodrigues via double cross-product: v' = v + 2w(qv×v) + 2(qv×(qv×v))
    cx = qy*vz - qz*vy
    cy = qz*vx - qx*vz
    cz = qx*vy - qy*vx
    return (
        vx + 2*qw*cx + 2*(qy*cz - qz*cy),
        vy + 2*qw*cy + 2*(qz*cx - qx*cz),
        vz + 2*qw*cz + 2*(qx*cy - qy*cx),
    )


def _compute_correction(
    pos_wb: tuple, q_wb: tuple,
    pos_lb: tuple, q_lb: tuple,
) -> tuple[tuple, tuple]:
    """Compute T_world_loc = T_world_base · inv(T_loc_base).

    Parameters
    ----------
    pos_wb, q_wb : position and orientation of base_frame in world (ground truth)
    pos_lb, q_lb : position and orientation of base_frame in localization frame (from TF)

    Returns
    -------
    pos_wl, q_wl : position and orientation of localization frame in world
    """
    # inv(T_loc_base) = T_base_loc:  rotation = q_lb^-1,  translation = -R_lb^T · t_lb
    q_lb_inv = _quat_conj(q_lb)
    t_base_loc = _rotate_vec(q_lb_inv, (-pos_lb[0], -pos_lb[1], -pos_lb[2]))

    # T_world_loc = T_world_base · T_base_loc
    q_wl = _quat_mult(q_wb, q_lb_inv)
    t_wl = tuple(a + b for a, b in zip(_rotate_vec(q_wb, t_base_loc), pos_wb))
    return t_wl, q_wl


# ── RobotEntry ────────────────────────────────────────────────────────────────

class RobotEntry:
    """Stores ground-truth pose and TF frame names for one robot."""

    def __init__(
        self,
        gz_model_name: str,
        tf_prefix: str,
        localization_frame: str,
        base_frame: str,
    ) -> None:
        self.gz_model_name      = gz_model_name
        self.tf_prefix          = tf_prefix
        self.localization_frame = localization_frame
        self.base_frame         = base_frame
        self._lock              = threading.Lock()
        self._position:    tuple[float, float, float] | None = None
        self._orientation: tuple[float, float, float, float] | None = None

    def update(self, pose: Pose) -> None:
        p = pose.position
        q = pose.orientation
        with self._lock:
            self._position    = (p.x, p.y, p.z)
            self._orientation = (q.x, q.y, q.z, q.w)

    def get_pose(self) -> tuple:
        with self._lock:
            return self._position, self._orientation


# ── Node ──────────────────────────────────────────────────────────────────────

class GroundTruthTFPublisher(Node):
    """Publishes correction TF: ground_truth_frame → localization_frame."""

    def __init__(self) -> None:
        super().__init__('ground_truth_tf_publisher')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('multi_robot',        False)
        self.declare_parameter('gz_model_name',      'caddy_ai2')
        self.declare_parameter('tf_prefix',          '')
        self.declare_parameter('localization_frame', 'caddy_ai2/odom')
        self.declare_parameter('robots',             ['caddy_ai2:caddy_ai2:caddy_ai2/odom'])
        self.declare_parameter('ground_truth_frame', 'world')
        self.declare_parameter('base_suffix',        'base_footprint')
        self.declare_parameter('publish_rate',       50.0)

        multi_robot:        bool  = self.get_parameter('multi_robot').get_parameter_value().bool_value
        ground_truth_frame: str   = self.get_parameter('ground_truth_frame').get_parameter_value().string_value
        base_suffix:        str   = self.get_parameter('base_suffix').get_parameter_value().string_value
        publish_rate:       float = self.get_parameter('publish_rate').get_parameter_value().double_value

        self._ground_truth_frame = ground_truth_frame
        self._robots: list[RobotEntry] = []

        self.get_logger().info('GroundTruthTFPublisher iniciando...')

        if not multi_robot:
            gz_name:       str = self.get_parameter('gz_model_name').get_parameter_value().string_value
            tf_prefix_raw: str = self.get_parameter('tf_prefix').get_parameter_value().string_value
            tf_prefix = tf_prefix_raw if tf_prefix_raw else gz_name
            loc_frame: str = self.get_parameter('localization_frame').get_parameter_value().string_value
            base_frame = f'{tf_prefix}/{base_suffix}'
            self._robots.append(RobotEntry(gz_name, tf_prefix, loc_frame, base_frame))
        else:
            for entry in self.get_parameter('robots').get_parameter_value().string_array_value:
                parts = entry.split(':', 2)
                if len(parts) < 3:
                    self.get_logger().error(
                        f'Entrada inválida en robots (esperado gz:prefix:loc_frame): {entry!r}')
                    continue
                gz_name   = parts[0].strip()
                tf_prefix = parts[1].strip()
                loc_frame = parts[2].strip()
                base_frame = f'{tf_prefix}/{base_suffix}'
                self._robots.append(RobotEntry(gz_name, tf_prefix, loc_frame, base_frame))

        self.get_logger().info(
            f'  ground_truth_frame : {ground_truth_frame}\n'
            f'  base_suffix        : {base_suffix}\n'
            f'  publish_rate       : {publish_rate} Hz'
        )
        for r in self._robots:
            self.get_logger().info(
                f'  robot  gz={r.gz_model_name}  prefix={r.tf_prefix}'
                f'  base={r.base_frame}  loc={r.localization_frame}'
            )

        # ── TF broadcaster + listener ──────────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer      = Buffer()
        self._tf_listener    = TransformListener(self._tf_buffer, self)

        # ── Suscripciones gz-transport (una por robot) ────────────────────
        self._gz_node = GzNode()
        for robot in self._robots:
            gz_topic = f'/model/{robot.gz_model_name}/pose'
            self._gz_node.subscribe(Pose, gz_topic, robot.update)
            self.get_logger().info(f'  Suscrito a: {gz_topic}')

        self._timer = self.create_timer(1.0 / publish_rate, self._publish_tfs)

    # ──────────────────────────────────────────────────────────────────────
    def _publish_tfs(self) -> None:
        now     = self.get_clock().now()
        now_msg = now.to_msg()

        for robot in self._robots:
            pos_wb, q_wb = robot.get_pose()
            if pos_wb is None:
                continue

            try:
                tf_loc_base = self._tf_buffer.lookup_transform(
                    robot.localization_frame,
                    robot.base_frame,
                    Time(),
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(
                    f'TF no disponible {robot.localization_frame}→{robot.base_frame}: {e}',
                    throttle_duration_sec=5.0,
                )
                continue

            tl = tf_loc_base.transform.translation
            rl = tf_loc_base.transform.rotation
            pos_lb = (tl.x, tl.y, tl.z)
            q_lb   = (rl.x, rl.y, rl.z, rl.w)

            t_wl, q_wl = _compute_correction(pos_wb, q_wb, pos_lb, q_lb)

            tf_msg = TransformStamped()
            tf_msg.header.stamp    = now_msg
            tf_msg.header.frame_id = self._ground_truth_frame
            tf_msg.child_frame_id  = robot.localization_frame

            tf_msg.transform.translation.x = t_wl[0]
            tf_msg.transform.translation.y = t_wl[1]
            tf_msg.transform.translation.z = t_wl[2]
            tf_msg.transform.rotation.x    = q_wl[0]
            tf_msg.transform.rotation.y    = q_wl[1]
            tf_msg.transform.rotation.z    = q_wl[2]
            tf_msg.transform.rotation.w    = q_wl[3]

            self._tf_broadcaster.sendTransform(tf_msg)


# ──────────────────────────────────────────────────────────────────────────────
def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GroundTruthTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
