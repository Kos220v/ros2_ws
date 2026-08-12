#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
waypoint_follower — движение по GPS-маршруту (уличный сценарий).

Подписан на /odometry/global (метры от старта, ENU, выход navsat_transform
+ ekf) и публикует /cmd_vel/auto (geometry_msgs/Twist) — целевые команды,
которые затем обрабатывает obstacle_avoider (объезд препятствий лидаром).

Маршрут — YAML вида:
    waypoints:
      - lat: 56.2991446
        lon: 43.9230612
        radius: 2.0        # радиус достижения, м
      - ...

Считает:
  * курс на текущую точку (bearing по координатам GPS);
  * поворот = bearing - текущий курс (из /odom или /imu/data);
  * скорость = min(max_speed, k_lin * расстояние до точки);
  * при попадании в radius точки — переходит к следующей.

Курс робота берётся из /odom (yaw), т.к. он у нас стабильный (от магнитометра).
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

import yaml

RAD2DEG = 180.0 / math.pi
EARTH_R = 6371000.0   # средний радиус Земли, м


def latlon_to_xy(lat1, lon1, lat2, lon2):
    """Разница (dx, dy) в метрах между двумя точками (ENU: x-восток, y-север)."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    lat_mid = math.radians((lat1 + lat2) / 2.0)
    x = EARTH_R * dlon * math.cos(lat_mid)   # восток
    y = EARTH_R * dlat                       # север
    return x, y


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # --- параметры -----------------------------------------------------
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel/auto')
        self.declare_parameter('home_latitude', 0.0)
        self.declare_parameter('home_longitude', 0.0)
        self.declare_parameter('max_speed', 0.8)        # м/с
        self.declare_parameter('k_lin', 0.5)            # коэфф. скорости к цели
        self.declare_parameter('k_ang', 1.5)            # коэфф. поворота
        self.declare_parameter('yaw_tolerance_deg', 10.0)  # порог поворота перед движением
        self.declare_parameter('stop_dist', 1.0)        # тормозим за N метров до точки
        self.declare_parameter('loop', False)           # зациклить маршрут
        self.declare_parameter('use_yaw_from_imu', True)  # yaw из /odom

        wp_file = self.get_parameter('waypoints_file').value
        self.home_lat = float(self.get_parameter('home_latitude').value)
        self.home_lon = float(self.get_parameter('home_longitude').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.k_lin = float(self.get_parameter('k_lin').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.yaw_tol = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.loop = bool(self.get_parameter('loop').value)

        # --- маршрут -------------------------------------------------------
        self.waypoints = []   # список (dx, dy, radius) в метрах от старта
        if wp_file and os.path.exists(wp_file):
            with open(wp_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            for wp in data.get('waypoints', []):
                dx, dy = latlon_to_xy(self.home_lat, self.home_lon,
                                      float(wp['lat']), float(wp['lon']))
                r = float(wp.get('radius', 2.0))
                self.waypoints.append((dx, dy, r))
        else:
            self.get_logger().error(f"Файл маршрута не найден: {wp_file}")

        self.get_logger().info(
            f"Маршрут: {len(self.waypoints)} точек, старт "
            f"({self.home_lat:.6f}, {self.home_lon:.6f}), "
            f"max_speed={self.max_speed} м/с")

        self.idx = 0
        self.pose_xy = (0.0, 0.0)
        self.yaw = 0.0
        self.active = True   # автопилот (переключается сервисом)

        # --- pub/sub/service ----------------------------------------------
        self.pub_cmd = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
        self.sub_odom = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.srv_enable = self.create_service(Trigger, '~/enable', self._cb_enable)
        self.srv_disable = self.create_service(Trigger, '~/disable', self._cb_disable)
        self.timer = self.create_timer(0.1, self._tick)   # 10 Гц

    # --- сервисы включения/выключения --------------------------------------
    def _cb_enable(self, req, res):
        self.active = True
        res.success = True
        res.message = "Waypoint follower включён"
        self.get_logger().info("Автопилот ВКЛЮЧЁН")
        return res

    def _cb_disable(self, req, res):
        self.active = False
        self.pub_cmd.publish(Twist())   # остановка
        res.success = True
        res.message = "Waypoint follower выключен"
        self.get_logger().info("Автопилот ВЫКЛЮЧЕН")
        return res

    # --- одометрия ----------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        self.pose_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # --- главный цикл -------------------------------------------------------
    def _tick(self):
        if not self.active:
            return
        if self.idx >= len(self.waypoints):
            # Маршрут пройден
            self.pub_cmd.publish(Twist())
            if self.loop:
                self.idx = 0
            else:
                self.get_logger().info("Маршрут завершён")
            return

        tx, ty, radius = self.waypoints[self.idx]
        x, y = self.pose_xy
        dist = math.hypot(tx - x, ty - y)

        # --- достигли точки? ---
        if dist < radius:
            self.get_logger().info(f"Точка {self.idx + 1}/{len(self.waypoints)} "
                                   f"достигнута (dist={dist:.1f} м)")
            self.idx += 1
            return

        # --- курс на цель (в мировой системе ENU: x-восток, y-север) ---
        bearing = math.atan2(tx - x, ty - y)   # угол от оси Y (север) по часовой
        # Преобразуем в ENU-yaw (от оси X против часовой), как наш /odom:
        target_yaw = math.pi / 2.0 - bearing
        # --- ошибка по углу ---
        err = math.atan2(math.sin(target_yaw - self.yaw),
                         math.cos(target_yaw - self.yaw))

        cmd = Twist()
        if abs(err) > self.yaw_tol:
            # Сначала разворачиваемся к цели:
            cmd.angular.z = self.k_ang * err
        else:
            # Едем к цели:
            speed = self.k_lin * dist
            if dist < self.stop_dist:
                speed *= dist / self.stop_dist   # плавное торможение
            cmd.linear.x = min(speed, self.max_speed)
            cmd.angular.z = self.k_ang * err

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
