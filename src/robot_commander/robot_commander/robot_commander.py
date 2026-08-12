#!/usr/bin/env python3
"""
Robot Commander - Главный узел управления роботом.

Работает как источник команд для мультиплексора (cmd_switcher).
Публикует в: /cmd_vel/auto, /cmd_vel/home

====================================================================
ИЗМЕНЕНИЯ В ЭТОЙ ВЕРСИИ
====================================================================
1. ИСПРАВЛЕН фильтр GPS: используется `>= NavSatStatus.STATUS_FIX`
2. ИСПРАВЛЕНА посадка на курс (return-home): используется heading
3. ДОБАВЛЕНО движение по маршруту из YAML-файла
4. ДОБАВЛЕНА подписка на /set_route для динамической загрузки маршрута
5. ИСПРАВЛЕНА проблема с "кручением на месте":
   - УМЕНЬШЕНА максимальная угловая скорость (0.4 rad/s)
   - УМЕНЬШЕН коэффициент усиления (0.8)
   - УВЕЛИЧЕНА мертвая зона (5°)
   - Робот всегда движется вперед даже при повороте
6. ДОБАВЛЕНА ДИАГНОСТИКА:
   - Публикация диагностического топика /navigation_diagnostics
   - Отображение ошибки курса и расстояния до цели
   - Цветовая индикация состояния (зеленый/желтый/красный)
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, NavSatFix, NavSatStatus
from std_msgs.msg import String, Bool, Int8, Float32MultiArray, Float64
import yaml

# Попытка импорта geographiclib
try:
    from geographiclib.geodesic import Geodesic
    HAS_GEOGRAPHICLIB = True
    print("Using geographiclib for navigation")
except ImportError:
    HAS_GEOGRAPHICLIB = False
    print("Using built-in haversine formulas for navigation")


def haversine_distance(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371000.0
    return c * r


def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360.0) % 360.0


def distance_between(lat1, lon1, lat2, lon2):
    """Расстояние между двумя точками (м). Использует geographiclib, если доступен."""
    if HAS_GEOGRAPHICLIB:
        return Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2)['s12']
    return haversine_distance(lat1, lon1, lat2, lon2)


def bearing_between(lat1, lon1, lat2, lon2):
    """Азимут (0..360°, по часовой стрелке от севера) от точки 1 к точке 2."""
    if HAS_GEOGRAPHICLIB:
        return Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2)['azi1'] % 360.0
    return calculate_bearing(lat1, lon1, lat2, lon2)


def normalize_angle_deg(angle):
    """Приводит угол к диапазону (-180, 180] с защитой от nan и inf."""
    if not math.isfinite(angle):
        return 0.0
    a = angle % 360.0
    if a > 180.0:
        a -= 360.0
    return a


class ControlModes:
    AUTO = 0
    MANUAL = 1
    AVOID = 2
    RETURN_HOME = 3


class RobotCommander(Node):
    def __init__(self):
        super().__init__('robot_commander')

        # --- Параметры ---
        self.declare_parameter('auto_speed', 0.3)
        self.declare_parameter('obstacle_timeout', 5.0)
        self.declare_parameter('avoid_speed', 0.15)
        self.declare_parameter('avoid_turn_rate', 0.5)
        self.declare_parameter('return_home_speed', 0.25)
        self.declare_parameter('home_latitude', 0.0)
        self.declare_parameter('home_longitude', 0.0)
        self.declare_parameter('arrival_distance', 1.0)
        self.declare_parameter('gps_timeout', 5.0)

        # --- Параметры следования по курсу ---
        self.declare_parameter('heading_topic', '/heading_deg')
        self.declare_parameter('heading_timeout', 1.0)
        self.declare_parameter('steering_kp', 0.8)  # УМЕНЬШЕНО: было 1.5
        self.declare_parameter('max_turn_rate', 0.4)  # УМЕНЬШЕНО: было 1.5

        # --- Параметры маршрута ---
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('route_loop', False)
        self.declare_parameter('route_complete_action', 'return_home')

        self.auto_speed = self.get_parameter('auto_speed').value
        self.obstacle_timeout = self.get_parameter('obstacle_timeout').value
        self.avoid_speed = self.get_parameter('avoid_speed').value
        self.avoid_turn_rate = self.get_parameter('avoid_turn_rate').value
        self.return_home_speed = self.get_parameter('return_home_speed').value
        self.arrival_distance = self.get_parameter('arrival_distance').value
        self.gps_timeout = self.get_parameter('gps_timeout').value

        self.home_latitude = self.get_parameter('home_latitude').value
        self.home_longitude = self.get_parameter('home_longitude').value

        self.heading_timeout = self.get_parameter('heading_timeout').value
        self.steering_kp = self.get_parameter('steering_kp').value
        self.max_turn_rate = self.get_parameter('max_turn_rate').value

        self.route_loop = self.get_parameter('route_loop').value
        self.route_complete_action = self.get_parameter('route_complete_action').value

        self.current_mode = ControlModes.MANUAL

        # --- Publishers ---
        self.pub_auto = self.create_publisher(Twist, '/cmd_vel/auto', 10)
        self.pub_home = self.create_publisher(Twist, '/cmd_vel/home', 10)

        self.mode_pub = self.create_publisher(Int8, 'robot_mode', 10)
        self.mode_status_pub = self.create_publisher(String, 'robot_mode_status', 10)
        self.home_distance_pub = self.create_publisher(Float32MultiArray, '/home_distance', 10)
        self.route_status_pub = self.create_publisher(String, '/route_status', 10)
        
        # ДИАГНОСТИКА: публикация навигационных данных
        self.diagnostics_pub = self.create_publisher(String, '/navigation_diagnostics', 10)

        self.mode_command_pub = self.create_publisher(Int8, '/control_mode_command', 10)

        # --- Subscribers ---
        self.scan_sub = self.create_subscription(LaserScan, 'scan_reliable', self.scan_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.mode_sub = self.create_subscription(Int8, '/control_mode', self.mode_sync_callback, 10)
        self.mode_command_sub = self.create_subscription(String, 'set_mode', self.set_mode_command_callback, 10)

        heading_topic = self.get_parameter('heading_topic').value
        self.heading_sub = self.create_subscription(Float64, heading_topic, self.heading_callback, 10)

        self.set_route_sub = self.create_subscription(String, '/set_route', self.set_route_callback, 10)

        # --- State variables ---
        self.obstacle_detected = False
        self.last_obstacle_time = self.get_clock().now()
        self.current_gps = None
        self.current_scan = None
        self.last_gps_time = self.get_clock().now()
        self.home_arrived = False
        self.auto_switch_enabled = True

        self.current_heading = None
        self.last_heading_time = None

        # --- Маршрут ---
        self.waypoints = []
        self.current_wp_idx = 0
        self.route_finished = False
        self._load_waypoints()

        # --- Timers ---
        self.timer = self.create_timer(0.1, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.diagnostics_timer = self.create_timer(0.5, self.publish_diagnostics)  # Диагностика 2 раза в секунду

        self.get_logger().info('Robot Commander started (Split Mode Output)')
        if self.home_latitude != 0.0 or self.home_longitude != 0.0:
            self.get_logger().info(f'HOME position: {self.home_latitude:.6f}, {self.home_longitude:.6f}')
        else:
            self.get_logger().warn('HOME position not set!')

        if self.waypoints:
            self.get_logger().info(f'✅ Route loaded: {len(self.waypoints)} waypoint(s)')
            for i, wp in enumerate(self.waypoints):
                self.get_logger().info(f'  WP {i+1}: lat={wp["lat"]:.6f}, lon={wp["lon"]:.6f}, radius={wp["radius"]:.2f}m')
        else:
            self.get_logger().warn(
                '❌ No waypoints loaded (waypoints_file пуст или не найден). '
                'В режиме AUTO робот будет просто ехать прямо. '
                'Задайте параметр waypoints_file, чтобы включить движение по маршруту.'
            )

        self.publish_mode_status()

    # ------------------------------------------------------------ загрузка маршрута
    def _load_waypoints(self):
        path = self.get_parameter('waypoints_file').value
        if not path:
            return

        if not os.path.isfile(path):
            self.get_logger().error(f'waypoints_file "{path}" не найден. Маршрут не загружен.')
            return

        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Не удалось прочитать waypoints_file: {e}')
            return

        raw_points = None
        if isinstance(data, dict) and 'waypoints' in data:
            raw_points = data['waypoints']
        elif isinstance(data, list):
            raw_points = data

        if not raw_points:
            self.get_logger().error('waypoints_file не содержит точек (ожидается ключ "waypoints").')
            return

        waypoints = []
        for i, wp in enumerate(raw_points):
            try:
                lat = float(wp['lat'])
                lon = float(wp['lon'])
                radius = float(wp.get('radius', self.arrival_distance))
                waypoints.append({'lat': lat, 'lon': lon, 'radius': radius})
            except (KeyError, TypeError, ValueError) as e:
                self.get_logger().error(f'Точка #{i} в waypoints_file некорректна ({e}), пропущена.')

        self.waypoints = waypoints

    def set_route_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().error(f'/set_route: некорректный JSON ({e})')
            return

        raw_points = data.get('waypoints') if isinstance(data, dict) else None
        if not raw_points:
            self.get_logger().error('/set_route: нет ключа "waypoints" или список пуст')
            return

        waypoints = []
        for i, wp in enumerate(raw_points):
            try:
                lat = float(wp['lat'])
                lon = float(wp['lon'])
                radius = float(wp.get('radius', self.arrival_distance))
                waypoints.append({'lat': lat, 'lon': lon, 'radius': radius})
            except (KeyError, TypeError, ValueError) as e:
                self.get_logger().error(f'/set_route: точка #{i} некорректна ({e}), пропущена.')

        if not waypoints:
            self.get_logger().error('/set_route: после проверки не осталось валидных точек, маршрут не изменён.')
            return

        if isinstance(data, dict) and data.get('route_complete_action') in ('stop', 'loop', 'return_home'):
            self.route_complete_action = data['route_complete_action']

        self.waypoints = waypoints
        self.current_wp_idx = 0
        self.route_finished = False
        self.get_logger().info(f'/set_route: новый маршрут принят, точек: {len(waypoints)}')
        
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  WP {i+1}: lat={wp["lat"]:.6f}, lon={wp["lon"]:.6f}, radius={wp["radius"]:.2f}m')

        ack = String()
        ack.data = f'Route updated: {len(waypoints)} waypoint(s) loaded'
        self.route_status_pub.publish(ack)

    # ------------------------------------------------------------ прочее состояние
    def get_mode_name(self, mode=None):
        if mode is None:
            mode = self.current_mode
        modes = {0: 'AUTO', 1: 'MANUAL', 2: 'AVOID', 3: 'RETURN_HOME'}
        return modes.get(mode, 'UNKNOWN')

    def publish_mode_status(self):
        status_msg = String()
        status_msg.data = f"Current mode: {self.get_mode_name()}"
        self.mode_status_pub.publish(status_msg)

        mode_msg = Int8()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)

    def publish_status(self):
        if self.current_mode == ControlModes.RETURN_HOME and self.current_gps:
            distance = distance_between(
                self.current_gps.latitude, self.current_gps.longitude,
                self.home_latitude, self.home_longitude
            ) if self.home_latitude != 0.0 or self.home_longitude != 0.0 else float('inf')

            dist_msg = Float32MultiArray()
            dist_msg.data = [distance]
            self.home_distance_pub.publish(dist_msg)

            if distance < self.arrival_distance and not self.home_arrived:
                self.home_arrived = True
                self.get_logger().info('ARRIVED HOME! Switching to AUTO mode')
                self.current_mode = ControlModes.AUTO
                self.publish_mode_status()

        elif self.current_mode == ControlModes.AUTO and self.waypoints:
            status = String()
            if self.route_finished:
                status.data = 'Route finished'
            elif self.current_gps and self.current_wp_idx < len(self.waypoints):
                wp = self.waypoints[self.current_wp_idx]
                d = distance_between(self.current_gps.latitude, self.current_gps.longitude, wp['lat'], wp['lon'])
                status.data = f'Waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}, distance={d:.1f}m'
            else:
                status.data = 'Waiting for GPS fix...'
            self.route_status_pub.publish(status)

    # ------------------------------------------------------------ ДИАГНОСТИКА
    def publish_diagnostics(self):
        """Публикация диагностической информации о навигации"""
        if self.current_mode not in [ControlModes.AUTO, ControlModes.RETURN_HOME]:
            return
            
        if not self.current_gps or self.current_heading is None:
            return
            
        # Определяем текущую цель
        target_lat = None
        target_lon = None
        target_radius = None
        target_desc = ""
        
        if self.current_mode == ControlModes.AUTO and self.waypoints and not self.route_finished:
            if self.current_wp_idx < len(self.waypoints):
                wp = self.waypoints[self.current_wp_idx]
                target_lat = wp['lat']
                target_lon = wp['lon']
                target_radius = wp['radius']
                target_desc = f"WP {self.current_wp_idx + 1}/{len(self.waypoints)}"
        elif self.current_mode == ControlModes.RETURN_HOME:
            if self.home_latitude != 0.0 or self.home_longitude != 0.0:
                target_lat = self.home_latitude
                target_lon = self.home_longitude
                target_radius = self.arrival_distance
                target_desc = "HOME"
        
        if target_lat is None:
            return
            
        # Расчеты
        distance = distance_between(
            self.current_gps.latitude, self.current_gps.longitude,
            target_lat, target_lon
        )
        
        target_bearing = bearing_between(
            self.current_gps.latitude, self.current_gps.longitude,
            target_lat, target_lon
        )
        
        heading_error = normalize_angle_deg(target_bearing - self.current_heading)
        abs_error = abs(heading_error)
        
        # Определяем статус
        if abs_error < 5.0:
            status = "✅ НА ЦЕЛЬ" if distance < target_radius else "✅ КУРС OK"
            color = "🟢"
        elif abs_error < 30.0:
            status = "⚠️ КОРРЕКТИРУЕТСЯ"
            color = "🟡"
        else:
            status = "🔴 ПОВОРОТ"
            color = "🔴"
        
        # Формируем сообщение диагностики
        diag_msg = String()
        diag_msg.data = (
            f"{color} [{self.get_mode_name()}] {target_desc}\n"
            f"  📏 Расстояние: {distance:.1f} м (порог: {target_radius:.1f} м)\n"
            f"  🧭 Курс: {self.current_heading:.1f}° → Цель: {target_bearing:.1f}°\n"
            f"  📐 Ошибка: {heading_error:+.1f}° ({abs_error:.1f}°)\n"
            f"  {status}"
        )
        
        self.diagnostics_pub.publish(diag_msg)
        
        # Также выводим в лог с меньшей частотой
        self.get_logger().info(
            f"{color} {target_desc}: dist={distance:.1f}m, "
            f"heading={self.current_heading:.1f}°, target={target_bearing:.1f}°, "
            f"error={heading_error:+.1f}°"
        )

    # ------------------------------------------------------------ callbacks
    def mode_sync_callback(self, msg: Int8):
        new_mode = msg.data
        if new_mode != self.current_mode and new_mode in [0, 1, 2, 3]:
            self.current_mode = new_mode
            self.get_logger().warn(f'Mode changed to: {self.get_mode_name()}')
            self.publish_mode_status()

            if new_mode != ControlModes.AVOID:
                self.obstacle_detected = False

            if new_mode == ControlModes.RETURN_HOME:
                self.home_arrived = False

    def set_mode_command_callback(self, msg: String):
        mode_map = {'auto': 0, 'manual': 1, 'avoid': 2, 'return_home': 3, 'home': 3}
        if msg.data.lower() in mode_map:
            new_mode = mode_map[msg.data.lower()]
            if new_mode != self.current_mode:
                cmd_msg = Int8()
                cmd_msg.data = new_mode
                self.mode_command_pub.publish(cmd_msg)

    def scan_callback(self, msg: LaserScan):
        self.current_scan = msg

        if self.current_mode != ControlModes.AUTO:
            return

        num_points = len(msg.ranges)
        center_start = int(num_points * 0.33)
        center_end = int(num_points * 0.67)

        center_distances = msg.ranges[center_start:center_end]
        valid_distances = [d for d in center_distances if msg.range_min < d < msg.range_max]

        if valid_distances:
            min_distance = min(valid_distances)
            if min_distance < 0.5:
                if not self.obstacle_detected:
                    self.get_logger().warn(f'Obstacle at {min_distance:.2f}m, switching to AVOID')
                    self.obstacle_detected = True
                    self.last_obstacle_time = self.get_clock().now()

                    cmd_msg = Int8()
                    cmd_msg.data = ControlModes.AVOID
                    self.mode_command_pub.publish(cmd_msg)
            else:
                if self.obstacle_detected and self.auto_switch_enabled:
                    time_since = (self.get_clock().now() - self.last_obstacle_time).nanoseconds / 1e9
                    if time_since > self.obstacle_timeout:
                        self.obstacle_detected = False
                        self.get_logger().info('Obstacle cleared, switching to AUTO')
                        cmd_msg = Int8()
                        cmd_msg.data = ControlModes.AUTO
                        self.mode_command_pub.publish(cmd_msg)

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.current_gps = msg
            self.last_gps_time = self.get_clock().now()

    def heading_callback(self, msg: Float64):
        self.current_heading = msg.data
        self.last_heading_time = self.get_clock().now()

    def _heading_is_fresh(self):
        if self.current_heading is None or self.last_heading_time is None:
            return False
        age = (self.get_clock().now() - self.last_heading_time).nanoseconds / 1e9
        return age < self.heading_timeout

    # ------------------------------------------------------------ навигация (ИСПРАВЛЕНА)
    def _steer_towards(self, target_lat, target_lon, speed, radius):
        """
        Возвращает (Twist, arrived: bool) для движения к точке.
        ИСПРАВЛЕНО: 
        - Уменьшена максимальная угловая скорость
        - Увеличен допуск на ошибку курса
        - Робот всегда движется вперед
        - Плавная остановка поворота
        """
        cmd = Twist()

        if not self.current_gps:
            return cmd, False

        distance = distance_between(
            self.current_gps.latitude, self.current_gps.longitude, target_lat, target_lon
        )

        if distance < radius:
            return cmd, True

        if not self._heading_is_fresh():
            return cmd, False

        target_bearing = bearing_between(
            self.current_gps.latitude, self.current_gps.longitude, target_lat, target_lon
        )

        heading_error = normalize_angle_deg(target_bearing - self.current_heading)
        abs_error = abs(heading_error)

        # ===== БАЗОВАЯ СКОРОСТЬ =====
        # Всегда едем вперед, даже при повороте
        linear_speed = speed
        
        # Замедляемся при приближении к цели
        if distance < 5.0:
            linear_speed *= (distance / 5.0)
        linear_speed = max(0.05, linear_speed)

        # ===== РАСЧЕТ ПОВОРОТА =====
        # Если ошибка маленькая - едем прямо
        if abs_error < 5.0:  # УВЕЛИЧЕНО: было 2.0
            cmd.linear.x = linear_speed
            cmd.angular.z = 0.0
            return cmd, False

        # П-регулятор с уменьшенным коэффициентом
        error_rad = math.radians(heading_error)
        angular_z = self.steering_kp * error_rad
        
        # Жесткое ограничение скорости поворота (МАКСИМУМ 0.4 rad/s)
        angular_z = max(-self.max_turn_rate, min(self.max_turn_rate, angular_z))
        
        # Плавное уменьшение поворота при приближении к цели
        if distance < 3.0:
            angular_z *= (distance / 3.0)

        # ===== ФИНАЛЬНЫЕ КОМАНДЫ =====
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_z
        
        return cmd, False

    def calculate_return_home_command(self):
        cmd, arrived = self._steer_towards(
            self.home_latitude, self.home_longitude, self.return_home_speed, self.arrival_distance
        )
        return cmd

    def calculate_route_command(self):
        """Следование по маршруту (список GPS-точек)."""
        if self.route_finished or self.current_wp_idx >= len(self.waypoints):
            self.route_finished = True
            return Twist()

        wp = self.waypoints[self.current_wp_idx]
        cmd, arrived = self._steer_towards(wp['lat'], wp['lon'], self.auto_speed, wp['radius'])

        if arrived:
            self.get_logger().info(f'✅ Waypoint {self.current_wp_idx + 1}/{len(self.waypoints)} reached')
            self.current_wp_idx += 1

            if self.current_wp_idx >= len(self.waypoints):
                if self.route_loop or self.route_complete_action == 'loop':
                    self.get_logger().info('Route complete — looping back to waypoint 1')
                    self.current_wp_idx = 0
                elif self.route_complete_action == 'return_home':
                    self.get_logger().info('Route complete — switching to RETURN_HOME')
                    self.route_finished = True
                    cmd_msg = Int8()
                    cmd_msg.data = ControlModes.RETURN_HOME
                    self.mode_command_pub.publish(cmd_msg)
                else:
                    self.get_logger().info('Route complete — stopping')
                    self.route_finished = True

        return cmd

    def calculate_avoidance_command(self):
        cmd = Twist()

        if not self.current_scan:
            cmd.linear.x = self.avoid_speed
            return cmd

        num_points = len(self.current_scan.ranges)
        left_start = int(num_points * 0.5)
        left_end = int(num_points * 0.67)
        right_start = int(num_points * 0.83)
        right_end = int(num_points * 1.0)

        left_distances = [d for d in self.current_scan.ranges[left_start:left_end]
                           if self.current_scan.range_min < d < self.current_scan.range_max]
        right_distances = [d for d in self.current_scan.ranges[right_start:right_end]
                            if self.current_scan.range_min < d < self.current_scan.range_max]

        left_clear = max(left_distances) if left_distances else 0.0
        right_clear = max(right_distances) if right_distances else 0.0

        cmd.linear.x = self.avoid_speed

        if left_clear > right_clear:
            cmd.angular.z = self.avoid_turn_rate
        else:
            cmd.angular.z = -self.avoid_turn_rate

        return cmd

    def control_loop(self):
        """
        Главный цикл управления.
        ВАЖНО: В режиме MANUAL этот узел НЕ публикует команды.
        """
        if self.current_mode == ControlModes.MANUAL:
            return

        cmd = Twist()
        target_publisher = None

        if self.current_mode == ControlModes.AUTO:
            if self.current_gps:
                if self.waypoints:
                    cmd = self.calculate_route_command()
                else:
                    cmd.linear.x = self.auto_speed
                    cmd.angular.z = 0.0
                target_publisher = self.pub_auto
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        elif self.current_mode == ControlModes.AVOID:
            cmd = self.calculate_avoidance_command()
            target_publisher = self.pub_home

        elif self.current_mode == ControlModes.RETURN_HOME:
            if self.current_gps:
                cmd = self.calculate_return_home_command()
                target_publisher = self.pub_home
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        if target_publisher:
            target_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = RobotCommander()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()