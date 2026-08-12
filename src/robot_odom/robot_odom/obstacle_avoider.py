#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
obstacle_avoider — локальная защита от препятствий лидаром (улица).

Подписан на /scan_reliable (sensor_msgs/LaserScan) и /cmd_vel/auto
(целевая команда от waypoint_follower). Публикует /cmd_vel (итоговая).

Логика (реактивная, 2D):
  * разбивает скан на сектор ВПЕРЁД (±45°) и БОКА (слева/справа);
  * если в секторе впереди есть препятствие ближе safety_dist —
    блокирует движение вперёд и поворачивает в сторону с большим
    свободным пространством (слева/справа);
  * иначе пропускает команду как есть.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.declare_parameter('scan_topic', '/scan_reliable')
        self.declare_parameter('cmd_in_topic', '/cmd_vel/auto_goal')
        self.declare_parameter('cmd_out_topic', '/cmd_vel/auto')
        self.declare_parameter('safety_dist', 1.5)        # м — стоп перед препятствием
        self.declare_parameter('front_sector_deg', 45.0)  # сектор «впереди»
        self.declare_parameter('side_clear_dist', 2.0)    # м — считаем бок свободным
        self.declare_parameter('avoid_angular', 0.6)      # рад/с — поворот при объезде
        self.declare_parameter('min_linear', 0.0)         # м/с — при объезде (0 = стоим)

        self.safety = float(self.get_parameter('safety_dist').value)
        self.front_sector = math.radians(float(self.get_parameter('front_sector_deg').value))
        self.side_clear = float(self.get_parameter('side_clear_dist').value)
        self.avoid_ang = float(self.get_parameter('avoid_angular').value)
        self.min_lin = float(self.get_parameter('min_linear').value)

        self.scan = None
        self.cmd_auto = Twist()

        self.sub_scan = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan, 10)
        self.sub_cmd = self.create_subscription(
            Twist, self.get_parameter('cmd_in_topic').value, self._on_cmd, 10)
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_out_topic').value, 10)
        self.timer = self.create_timer(0.1, self._tick)

    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    def _on_cmd(self, msg: Twist):
        self.cmd_auto = msg

    def _dist_in_sector(self, angles, ranges, center, width):
        """Минимальное расстояние в секторе [center-width, center+width] (рад)."""
        d = (angles - center + math.pi) % (2 * math.pi) - math.pi
        mask = (np.abs(d) <= width) & np.isfinite(ranges) & (ranges > 0.0)
        if not mask.any():
            return None
        return float(np.min(ranges[mask]))

    def _tick(self):
        if self.scan is None:
            return
        ranges = np.array(self.scan.ranges, dtype=float)
        n = len(ranges)
        angles = self.scan.angle_min + np.arange(n) * self.scan.angle_increment

        # Расстояние вперёд, слева, справа:
        d_front = self._dist_in_sector(angles, ranges, 0.0, self.front_sector)
        d_left = self._dist_in_sector(angles, ranges, math.pi / 2.0, math.radians(30.0))
        d_right = self._dist_in_sector(angles, ranges, -math.pi / 2.0, math.radians(30.0))

        out = Twist()
        blocked = d_front is not None and d_front < self.safety

        if blocked:
            # Препятствие впереди — поворачиваем в свободную сторону:
            left_ok = d_left is None or d_left > self.side_clear
            right_ok = d_right is None or d_right > self.side_clear
            if left_ok and not right_ok:
                out.angular.z = self.avoid_ang
            elif right_ok and not left_ok:
                out.angular.z = -self.avoid_ang
            elif left_ok and right_ok:
                # обе свободны — в сторону, где больше места
                out.angular.z = self.avoid_ang if (d_left or 99.0) >= (d_right or 99.0) \
                    else -self.avoid_ang
            else:
                out.angular.z = self.avoid_ang   # тупик — разворот налево
            out.linear.x = self.min_lin
            self.get_logger().debug(
                f"Препятствие впереди {d_front:.2f} м — объезд "
                f"(left={d_left}, right={d_right})")
        else:
            # Свободно — пропускаем команду автопилота:
            out = self.cmd_auto

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
