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
  * расстояние и курс на текущую точку по ТЕКУЩИМ координатам GPS;
  * поворот = bearing - текущий курс (из /odom);
  * скорость = min(max_speed, k_lin * расстояние до точки);
  * при попадании в radius точки — переходит к следующей.

Позиция /odometry/global не используется для расстояния до waypoint: она
может дрейфовать. От неё используется только ориентация (yaw).

Курс робота берётся из /odom (yaw), т.к. он у нас стабильный (от магнитометра).
"""

import math
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8
from std_srvs.srv import Trigger

import yaml

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
        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('gps_topic', '/fix')
        # Старый GPS fix нельзя считать текущей позицией робота.
        self.declare_parameter('gps_timeout_sec', 2.0)
        # Целевая команда идёт на ВХОД obstacle_avoider (он публикует /cmd_vel/auto,
        # который слушает cmd_switcher). По умолчанию — /cmd_vel/auto_goal.
        self.declare_parameter('cmd_topic', '/cmd_vel/auto_goal')
        self.declare_parameter('home_latitude', 0.0)
        self.declare_parameter('home_longitude', 0.0)
        self.declare_parameter('max_speed', 0.8)        # м/с
        self.declare_parameter('k_lin', 0.5)            # коэфф. скорости к цели
        self.declare_parameter('k_ang', 1.5)            # коэфф. поворота
        self.declare_parameter('max_angular', 1.0)      # рад/с — ОГРАНИЧЕНИЕ поворота
        self.declare_parameter('yaw_tolerance_deg', 10.0)  # порог поворота перед движением
        self.declare_parameter('turn_correct_deg', 5.0)  # жёсткая коррекция курса: при
                                                         # |err|>5° стоп + разворот к цели
        self.declare_parameter('stop_dist', 1.0)        # тормозим за N метров до точки
        self.declare_parameter('loop', False)           # зациклить маршрут
        self.declare_parameter('use_yaw_from_imu', True)  # yaw из /odom
        # Сглаживание целевого курса (0..1): low-pass на bearing.
        # Убирает флип знака ошибки при |err|~180° из-за шума GPS-позиции
        # (иначе робот «крутится влево-вправо» на месте у старта).
        self.declare_parameter('bearing_smoothing', 0.3)
        # Поправка поворота системы координат цели (град).
        # Если map-система (/odometry/global) повёрнута относительно
        # «восток/север» (например, на 180° — азимут робота не сходится
        # с телефоном), цель «оказывается не там». Параметр доворачивает
        # вектор цели. 180.0 — если цель «зеркалится» (сзади вместо спереди).
        self.declare_parameter('goal_yaw_offset_deg', 0.0)
        # Знак команды поворота: 1.0 или -1.0.
        # Если робот крутится в противоположную сторону от команды
        # (при ang<0 yaw растёт — робот «убегает» от цели и делает круги),
        # поставьте -1.0.
        self.declare_parameter('angular_z_sign', 1.0)

        # --- РЕЖИМ СУХОГО ПРОГОНА (для отладки) ---
        # Если true — команды НЕ публикуются, робот остаётся под управлением
        # с пульта. Логи выводятся полностью.
        self.declare_parameter('dry_run', True)

        wp_file = self.get_parameter('waypoints_file').value
        self.home_lat = float(self.get_parameter('home_latitude').value)
        self.home_lon = float(self.get_parameter('home_longitude').value)
        self.gps_timeout_sec = max(
            0.0, float(self.get_parameter('gps_timeout_sec').value))
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.k_lin = float(self.get_parameter('k_lin').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.yaw_tol = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))
        self.turn_correct_deg = float(self.get_parameter('turn_correct_deg').value)
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.bearing_smoothing = float(self.get_parameter('bearing_smoothing').value)
        self.goal_yaw_offset = math.radians(
            float(self.get_parameter('goal_yaw_offset_deg').value))
        self.angular_z_sign = float(self.get_parameter('angular_z_sign').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self._target_yaw_lp = None   # сглаженный целевой курс
        self._turn_dir = None        # зафиксированное направление разворота (при |err|~180°)
        self.turn_enter_deg = 100.0  # входим в «разворот», если |err| > 100°
        self.turn_exit_deg = 80.0    # выходим, когда |err| < 80° (гистерезис)

        # --- маршрут -------------------------------------------------------
        # Храним географические координаты цели. Расстояние и направление
        # считаются от последнего валидного /fix, а не от /odometry/global.
        self.waypoints = []   # список (latitude, longitude, radius_m)
        if wp_file and os.path.exists(wp_file):
            with open(wp_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            for wp in data.get('waypoints', []):
                lat = float(wp['lat'])
                lon = float(wp['lon'])
                r = float(wp.get('radius', 2.0))
                self.waypoints.append((lat, lon, r))
        else:
            self.get_logger().error(f"Файл маршрута не найден: {wp_file}")

        self.get_logger().info(
            f"Маршрут: {len(self.waypoints)} точек, старт "
            f"({self.home_lat:.6f}, {self.home_lon:.6f}), "
            f"max_speed={self.max_speed} м/с")
        self.get_logger().info(
            f"Конфигурация: k_lin={self.k_lin}, k_ang={self.k_ang}, "
            f"max_angular={self.max_angular} рад/с, "
            f"yaw_tol={math.degrees(self.yaw_tol):.0f}°, "
            f"bearing_smoothing={self.bearing_smoothing}"
        )
        if self.dry_run:
            self.get_logger().warning(
                "РЕЖИМ DRY RUN: команды НЕ публикуются, робот остаётся под управлением с пульта. "
                "Наблюдайте за логами для отладки."
            )

        self.idx = 0
        self.pose_xy = (0.0, 0.0)  # оставлено для диагностики /odometry/global
        self.yaw = 0.0
        self.gps = None             # текущая валидная GPS-координата (lat, lon)
        self._gps_time = None       # время получения последнего валидного fix
        # ВАЖНО: автопилот стартует ВЫКЛЮЧЕННЫМ — после перезагрузки стека
        # робот всегда в ручном режиме и не поедет сам. Включается:
        #   * по режиму с пульта (подписка на /control_mode, режим AUTO), или
        #   * сервисом ~/enable (внешний оператор/приложение).
        self.active = False
        self.auto_mode_reason = None   # кто включил: 'rc' | 'service' | None

        # --- pub/sub/service ----------------------------------------------
        self.pub_cmd = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
        self.sub_odom = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.sub_gps = self.create_subscription(
            NavSatFix, self.get_parameter('gps_topic').value, self._on_gps, 10)
        # Режим с пульта (публикует elrs_receiver в /control_mode):
        #   AUTO(0) -> автопилот включён; MANUAL(1) и другие -> выключен.
        # Это гарантирует: после перезагрузки стека (режим MANUAL) робот
        # не поедет сам, а при включённом пульте следует выбранному режиму.
        self.sub_mode = self.create_subscription(
            Int8, '/control_mode', self._on_control_mode, 10)
        self.srv_enable = self.create_service(Trigger, '~/enable', self._cb_enable)
        self.srv_disable = self.create_service(Trigger, '~/disable', self._cb_disable)
        # Калибровка направления: разверните робота носом на СЕВЕР (азимут 0°)
        # и вызовите сервис — follower сам вычислит поправку map-системы.
        self.srv_calib_north = self.create_service(Trigger, '~/calibrate_north',
                                                   self._cb_calibrate_north)
        self.timer = self.create_timer(0.1, self._tick)   # 10 Гц

    # --- режим с пульта / сервисы включения-выключения ---------------------
    def _on_control_mode(self, msg):
        """Синхронизация с режимом, выбранным на пульте (elrs_receiver).

        AUTO (0) — автопилот включён; MANUAL (1), AVOID (2), RETURN_HOME (3) —
        автопилот по маршруту выключен (роботом управляет оператор/другие
        узлы). При выключенном пульте режим запоминается в elrs_receiver и
        сюда повторно не приходит — автопилот продолжает как был.
        """
        if msg.data == 0:   # AUTO
            if not self.active:
                self.active = True
                self.auto_mode_reason = 'rc'
                self.get_logger().info(
                    "Автопилот ВКЛЮЧЁН (режим AUTO с пульта)")
        else:               # MANUAL / AVOID / RETURN_HOME
            if self.active:
                self.active = False
                self.auto_mode_reason = None
                if not self.dry_run:
                    self.pub_cmd.publish(Twist())   # немедленная остановка
                self.get_logger().info(
                    f"Автопилот ВЫКЛЮЧЕН (режим {msg.data} с пульта)")

    def _cb_enable(self, req, res):
        self.active = True
        self.auto_mode_reason = 'service'
        res.success = True
        res.message = "Waypoint follower включён"
        self.get_logger().info("Автопилот ВКЛЮЧЁН (сервис)")
        return res

    def _cb_disable(self, req, res):
        self.active = False
        self.auto_mode_reason = None
        if not self.dry_run:
            self.pub_cmd.publish(Twist())   # остановка
        res.success = True
        res.message = "Waypoint follower выключен"
        self.get_logger().info("Автопилот ВЫКЛЮЧЕН (сервис)")
        return res

    def _cb_calibrate_north(self, req, res):
        """Робот должен смотреть на СЕВЕР (азимут 0° по телефону/компасу).
        Вычисляет поправку map-системы: goal_yaw_offset = yaw_map - 90°."""
        # map-система: когда робот на севере (истинный азимут 0°, ENU-yaw 90°),
        # map показывает self.yaw. Значит, чтобы цель «на севере» дала err=0,
        # нужно: target_yaw(goal) = goal_enu_geo + offset, при goal=север
        # (enu=90°) → offset = self.yaw - 90°.
        offset = self.yaw - math.pi / 2.0
        self.goal_yaw_offset = math.atan2(math.sin(offset), math.cos(offset))
        self._target_yaw_lp = None   # сбросить сглаживание
        deg = math.degrees(self.goal_yaw_offset)
        self.get_logger().info(
            f"Калибровка севера: yaw_map={math.degrees(self.yaw):.1f}° → "
            f"goal_yaw_offset={deg:.1f}°")
        res.success = True
        res.message = f"goal_yaw_offset_deg = {deg:.1f}"
        return res

    # --- одометрия ----------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        self.pose_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_gps(self, msg: NavSatFix):
        """Сохраняет только валидный GPS fix для расчёта дистанции.

        NavSatFix со status < 0 либо NaN-координатами не должен заменять
        последнюю позицию: иначе робот может получить фиктивную дистанцию до
        waypoint или начать движение без определения позиции.
        """
        lat, lon = msg.latitude, msg.longitude
        if (msg.status.status >= 0 and math.isfinite(lat) and math.isfinite(lon)
                and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            self.gps = (lat, lon)
            self._gps_time = self.get_clock().now()

    # --- главный цикл -------------------------------------------------------
    def _tick(self):
        if not self.active:
            return
        if self.idx >= len(self.waypoints):
            # Маршрут пройден
            if not self.dry_run:
                self.pub_cmd.publish(Twist())
            if self.loop:
                self.idx = 0
            else:
                self.get_logger().info("Маршрут завершён")
            return

        target_lat, target_lon, radius = self.waypoints[self.idx]
        gps_fresh = (
            self._gps_time is not None
            and (self.get_clock().now() - self._gps_time).nanoseconds / 1e9
            <= self.gps_timeout_sec
        )
        if self.gps is None or not gps_fresh:
            # Не используем (0, 0), home_lat/home_lon или /odom как подмену
            # текущей GPS-позиции. До первого или при устаревшем fix движение
            # небезопасно.
            if not self.dry_run:
                self.pub_cmd.publish(Twist())
            self.get_logger().warning(
                "Нет свежего валидного GPS fix: расстояние до waypoint не "
                "рассчитано, команда движения не выдаётся",
                throttle_duration_sec=5.0)
            return

        current_lat, current_lon = self.gps
        # ENU-вектор ИМЕННО от текущего GPS fix до целевой GPS-точки.
        # Его длина — дистанция до waypoint, по ней определяются скорость и
        # факт достижения; /odometry/global здесь не участвует.
        gx, gy = latlon_to_xy(current_lat, current_lon, target_lat, target_lon)
        dist = math.hypot(gx, gy)

        # --- достигли точки? ---
        if dist < radius:
            self.get_logger().info(f"Точка {self.idx + 1}/{len(self.waypoints)} "
                                   f"достигнута по GPS (dist={dist:.1f} м)")
            self.idx += 1
            self._target_yaw_lp = None
            self._turn_dir = None
            return

        # --- курс на цель (ENU: x-восток, y-север) ---
        # goal_yaw_offset используется только для согласования GPS ENU и yaw
        # из /odom, если их системы отсчёта развёрнуты относительно друг друга.
        if self.goal_yaw_offset:
            c, s = math.cos(self.goal_yaw_offset), math.sin(self.goal_yaw_offset)
            gx, gy = c * gx - s * gy, s * gx + c * gy
        bearing = math.atan2(gx, gy)   # угол от оси Y (север) по часовой
        # Преобразуем в ENU-yaw (от оси X против часовой), как наш /odom:
        target_yaw = math.pi / 2.0 - bearing
        # --- сглаживание целевого курса (low-pass) ---
        # Убирает флип знака при |err|~180° из-за шума GPS-позиции:
        if self.bearing_smoothing > 0.0:
            if self._target_yaw_lp is None:
                self._target_yaw_lp = target_yaw
            else:
                d = math.atan2(math.sin(target_yaw - self._target_yaw_lp),
                               math.cos(target_yaw - self._target_yaw_lp))
                self._target_yaw_lp += self.bearing_smoothing * d
            target_yaw = self._target_yaw_lp

        # --- ошибка по углу ---
        err = math.atan2(math.sin(target_yaw - self.yaw),
                         math.cos(target_yaw - self.yaw))
        err_deg = math.degrees(err)

        # --- разворот на 180° (цель позади): фиксируем направление ---
        # При |err|~180° знак err флипается от малейшего шума/поворота,
        # и робот осциллирует «влево-вправо». Решение: один раз выбираем
        # направление и крутим в него, пока не довернёмся (гистерезис).
        if self._turn_dir is None and abs(err_deg) > self.turn_enter_deg:
            self._turn_dir = 1 if err_deg >= 0 else -1
        if self._turn_dir is not None and abs(err_deg) < self.turn_exit_deg:
            self._turn_dir = None

        cmd = Twist()
        if self._turn_dir is not None:
            # Крутим в зафиксированном направлении (макс. 1.0 рад/с):
            cmd.angular.z = self._turn_dir * self.max_angular
        elif abs(err) > self.yaw_tol:
            # Сначала разворачиваемся к цели (с ограничением скорости):
            cmd.angular.z = max(-self.max_angular,
                                min(self.max_angular, self.k_ang * err))
        else:
            # Едем к цели:
            speed = self.k_lin * dist
            if dist < self.stop_dist:
                speed *= dist / self.stop_dist   # плавное торможение
            cmd.linear.x = min(speed, self.max_speed)
            cmd.angular.z = max(-self.max_angular,
                                min(self.max_angular, self.k_ang * err))

        # --- ЖЁСТКАЯ КОРРЕКЦИЯ КУРСА ---
        # Если курс отклонился от цели более чем на turn_correct_deg (5°),
        # останавливаемся (v=0) и разворачиваемся к цели. Это устраняет
        # «рыскание» и неточность прохождения точек (робот едет только
        # когда точно смотрит на цель).
        if abs(err_deg) > self.turn_correct_deg and abs(err) <= self.yaw_tol:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_angular,
                                min(self.max_angular, self.k_ang * err))

        # Инверсия знака поворота (если робот крутится в другую сторону):
        cmd.angular.z *= self.angular_z_sign

        # --- Публикация команды (только если НЕ dry_run) ---
        if not self.dry_run:
            self.pub_cmd.publish(cmd)
        # else: ничего не публикуем, робот остаётся под управлением пульта

        # --- чистый статус-лог (каждые ~1 сек при 10 Гц) ---
        # Статус: расстояние всегда получено от текущего GPS fix.
        self._log_tick = getattr(self, '_log_tick', 0) + 1
        if self._log_tick % 10 == 0:
            self.get_logger().info(
                f"WP[{self.idx + 1}/{len(self.waypoints)}] "
                f"GPS=({current_lat:.7f},{current_lon:.7f}) | "
                f"target=({target_lat:.7f},{target_lon:.7f}) | "
                f"dist_gps={dist:5.1f}m "
                f"yaw={math.degrees(self.yaw):+7.1f}° | "
                f"ang_err={math.degrees(err):+6.1f}° | "
                f"cmd: v={cmd.linear.x:.2f} w={cmd.angular.z:+.2f}"
                + (" [DRY RUN - не отправлено]" if self.dry_run else "")
            )


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