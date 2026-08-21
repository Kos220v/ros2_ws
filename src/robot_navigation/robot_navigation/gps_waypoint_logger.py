#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
gps_waypoint_logger — запись маршрута прогулкой.

Самый надёжный способ получить маршрут — не выковыривать координаты из
Google Maps (там они относятся к спутниковому снимку, который смещён
относительно реальности на метры), а прокатить робота по маршруту руками
с пульта и записать те координаты, которые видит ЕГО СОБСТВЕННЫЙ приёмник.
Тогда систематическая ошибка антенны и приёмника взаимно сокращается, и робот
проезжает маршрут заметно точнее.

ИСПОЛЬЗОВАНИЕ
-------------
1. Запустите основной стек робота (bringup.launch.py) и дождитесь фикса GNSS.
2. В отдельном терминале:

       ros2 run robot_navigation gps_waypoint_logger \\
           --ros-args -p output_file:=/home/pi/route.yaml

3. Возите робота пультом. В каждой нужной точке останавливайтесь и вызывайте:

       ros2 service call /gps_waypoint_logger/log_waypoint \\
           std_srvs/srv/Trigger

4. Готовый файл скормите командиру маршрута параметром waypoints_file.

Узел пишет файл после каждой точки, поэтому внезапная разрядка аккумулятора
не потеряет всё, что вы уже наездили.
"""

import math
import os

import rclpy
from rclpy.node import Node

import yaml

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_srvs.srv import Trigger


def quaternion_to_yaw(q):
    """Извлекает угол поворота вокруг вертикальной оси из кватерниона."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GpsWaypointLogger(Node):

    def __init__(self):
        super().__init__('gps_waypoint_logger')

        self.declare_parameter(
            'output_file', os.path.expanduser('~/gps_waypoints.yaml'))

        # По умолчанию берём ОТФИЛЬТРОВАННУЮ позицию (/gps/filtered от
        # navsat_transform): она заметно спокойнее сырого фикса, потому что
        # прошла через EKF вместе с одометрией и IMU.
        self.declare_parameter('gps_topic', '/gps/filtered')

        # Резервный источник, если фильтрованный поток ещё не пошёл.
        self.declare_parameter('fallback_gps_topic', '/gps/fix')

        # Откуда брать курс робота, чтобы записать его в точку.
        self.declare_parameter('odom_topic', '/odometry/global')

        # Не записывать точку, если она ближе этого расстояния к предыдущей:
        # слишком частые точки заставляют Nav2 тормозить у каждой из них.
        self.declare_parameter('min_spacing_m', 1.0)

        self._path = self.get_parameter('output_file').value
        self._min_spacing = float(self.get_parameter('min_spacing_m').value)

        self._fix = None
        self._fix_fallback = None
        self._yaw = 0.0
        self._waypoints = []

        self.create_subscription(
            NavSatFix, self.get_parameter('gps_topic').value,
            self._on_fix, 10)
        self.create_subscription(
            NavSatFix, self.get_parameter('fallback_gps_topic').value,
            self._on_fix_fallback, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, 10)

        self.create_service(Trigger, '~/log_waypoint', self._srv_log)
        self.create_service(Trigger, '~/undo_waypoint', self._srv_undo)

        self.get_logger().info(
            f'Запись маршрута в {self._path}. '
            f'Команда для записи точки: ros2 service call '
            f'/gps_waypoint_logger/log_waypoint std_srvs/srv/Trigger')

    def _on_fix(self, msg):
        self._fix = msg

    def _on_fix_fallback(self, msg):
        self._fix_fallback = msg

    def _on_odom(self, msg):
        self._yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def _current_fix(self):
        """Возвращает лучший доступный фикс и его источник."""
        if self._fix is not None:
            return self._fix, 'filtered'
        if self._fix_fallback is not None:
            return self._fix_fallback, 'raw'
        return None, None

    def _srv_log(self, request, response):
        fix, source = self._current_fix()

        if fix is None:
            response.success = False
            response.message = (
                'Нет данных GNSS. Проверьте, что запущен драйвер GPS '
                'и есть фикс.')
            self.get_logger().error(response.message)
            return response

        if fix.status.status == NavSatStatus.STATUS_NO_FIX:
            response.success = False
            response.message = 'Приёмник ещё не поймал спутники — точка не записана.'
            self.get_logger().error(response.message)
            return response

        # Проверка минимального расстояния до предыдущей точки
        if self._waypoints and self._min_spacing > 0.0:
            prev = self._waypoints[-1]
            dist = self._distance(
                prev['latitude'], prev['longitude'],
                fix.latitude, fix.longitude)
            if dist < self._min_spacing:
                response.success = False
                response.message = (
                    f'Точка в {dist:.1f} м от предыдущей, '
                    f'минимум {self._min_spacing:.1f} м. Отъедьте дальше.')
                self.get_logger().warning(response.message)
                return response

        self._waypoints.append({
            'latitude': float(fix.latitude),
            'longitude': float(fix.longitude),
            'yaw': round(float(self._yaw), 4),
        })

        try:
            self._save()
        except Exception as exc:
            self._waypoints.pop()
            response.success = False
            response.message = f'Не удалось записать файл: {exc}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f'Точка {len(self._waypoints)} записана '
            f'({fix.latitude:.7f}, {fix.longitude:.7f}), источник: {source}')
        self.get_logger().info(response.message)
        return response

    def _srv_undo(self, request, response):
        if not self._waypoints:
            response.success = False
            response.message = 'Список точек пуст'
            return response

        removed = self._waypoints.pop()
        self._save()
        response.success = True
        response.message = (
            f'Удалена последняя точка '
            f'({removed["latitude"]:.7f}, {removed["longitude"]:.7f}). '
            f'Осталось: {len(self._waypoints)}')
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _distance(lat1, lon1, lat2, lon2):
        earth_r = 6371000.0
        p1 = math.radians(lat1)
        p2 = math.radians(lat2)
        dlat = p2 - p1
        dlon = math.radians(lon2 - lon1)
        x = dlon * math.cos((p1 + p2) / 2.0)
        return earth_r * math.hypot(x, dlat)

    def _save(self):
        directory = os.path.dirname(os.path.abspath(self._path))
        if directory:
            os.makedirs(directory, exist_ok=True)

        # Пишем через временный файл: если питание пропадёт в момент записи,
        # уже накопленный маршрут не превратится в обрезанный мусор.
        tmp = self._path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            f.write('# Маршрут, записанный gps_waypoint_logger.\n')
            f.write('# Координаты сняты приёмником самого робота.\n')
            yaml.safe_dump(
                {'waypoints': self._waypoints},
                f, allow_unicode=True, sort_keys=False, default_flow_style=False)
        os.replace(tmp, self._path)


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
