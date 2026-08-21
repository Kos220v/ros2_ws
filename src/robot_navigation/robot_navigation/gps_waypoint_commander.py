#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
gps_waypoint_commander — проезд маршрута из GPS-точек через Nav2.

ЧТО ЭТО ТАКОЕ
-------------
Тонкий «диспетчер» поверх Nav2. Он НЕ управляет моторами и НЕ считает курс —
всю грязную работу (планирование пути, объезд препятствий, регулирование
скорости, восстановление после застревания) делает Nav2. Задача этого узла —
прочитать маршрут из YAML и отдать его экшену /follow_gps_waypoints, а затем
рассказывать в лог, как идут дела.

Именно так и задумано в Nav2: GPS-координаты переводятся в декартовы точки
фрейма map сервисом /fromLL узла navsat_transform, после чего робот едет по ним
обычным планировщиком.

ФОРМАТ ФАЙЛА МАРШРУТА
---------------------
    waypoints:
      - latitude: 56.299145
        longitude: 43.923061
        yaw: 0.0        # необязательно, желаемый курс в точке (радианы, ENU)
      - latitude: 56.299500
        longitude: 43.923800

Для совместимости со старым форматом проекта также понимаются ключи
lat / lon.

БЕЗОПАСНОСТЬ ЗАПУСКА
--------------------
Узел стартует ВЫКЛЮЧЕННЫМ. Робот никогда не поедет сам после перезагрузки
Raspberry Pi. Поехать можно тремя способами:
  * перевести тумблер на пульте в режим AUTO (топик /control_mode == 0);
  * вызвать сервис  ros2 service call /gps_waypoint_commander/start_route
                    std_srvs/srv/Trigger;
  * запустить узел с параметром autostart:=true (только для отладки на
    стенде, не для реального выезда).

Перевод пульта в любой другой режим немедленно отменяет маршрут.
"""

import math
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import yaml

from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPose
from nav2_msgs.action import FollowGPSWaypoints
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Int8
from std_srvs.srv import Trigger


# Режимы, которые публикует elrs_receiver в /control_mode
MODE_AUTO = 0

# Половина скользящего окна global_costmap (120×120 м) с запасом.
# Робот всегда в центре окна, поэтому цель дальше ~60 м планировщик
# не увидит и вернёт GOAL_OUTSIDE_MAP (204).
DEFAULT_MAX_WP_SEP = 50.0
# Бытовой GNSS 2.5–5 м, xy_goal_tolerance 1.5 м — ближе бессмысленно.
DEFAULT_MIN_WP_SEP = 3.0

# Коды отказа Nav2 (Jazzy). Источник — nav2_msgs/action/*.action.
# NavigateToPose пробрасывает код вложенного действия (планировщик,
# контроллер, recovery), поэтому здесь собраны все диапазоны, которые
# могут прийти в FollowGPSWaypoints.error_code и MissedWaypoint.error_code.
NAV2_ERROR_CODES = {
    0:   ('NONE', 'успех'),
    # FollowPath
    100: ('UNKNOWN', 'неизвестная ошибка контроллера FollowPath'),
    101: ('INVALID_CONTROLLER', 'неизвестный или не загружен плагин контроллера'),
    102: ('TF_ERROR', 'нет трансформа TF — проверьте дерево map→odom→base_link'),
    103: ('INVALID_PATH', 'путь пустой или некорректный'),
    104: ('PATIENCE_EXCEEDED', 'контроллер исчерпал запас попыток'),
    105: ('FAILED_TO_MAKE_PROGRESS',
          'робот не продвигается (застрял, буксование, progress_checker)'),
    106: ('NO_VALID_CONTROL', 'контроллер не смог посчитать допустимую скорость'),
    107: ('CONTROLLER_TIMED_OUT', 'контроллер не уложился во время'),
    # ComputePathToPose / ComputePathThroughPoses
    200: ('UNKNOWN', 'неизвестная ошибка планировщика'),
    201: ('INVALID_PLANNER', 'неизвестный или не загружен плагин планировщика'),
    202: ('TF_ERROR', 'нет трансформа TF при планировании пути'),
    203: ('START_OUTSIDE_MAP',
          'старт вне костмапа — окно не покрывает робота'),
    204: ('GOAL_OUTSIDE_MAP',
          'цель вне скользящего костмапа: поставьте точки ближе '
          'или увеличьте окно global_costmap'),
    205: ('START_OCCUPIED', 'робот стоит в занятой ячейке костмапа'),
    206: ('GOAL_OCCUPIED', 'цель в занятой ячейке — препятствие на точке'),
    207: ('TIMEOUT', 'планировщик не уложился во время'),
    208: ('NO_VALID_PATH',
          'нет проходимого пути: препятствие, неизвестное пространство '
          'или цель за краем окна'),
    # Humble/Iron FollowPath (на случай другой сборки Nav2)
    300: ('UNKNOWN', 'неизвестная ошибка контроллера FollowPath'),
    301: ('INVALID_CONTROLLER', 'неизвестный или не загружен плагин контроллера'),
    302: ('TF_ERROR', 'нет трансформа TF — проверьте дерево map→odom→base_link'),
    303: ('INVALID_PATH', 'путь пустой или некорректный'),
    304: ('PATIENCE_EXCEEDED', 'контроллер исчерпал запас попыток'),
    305: ('FAILED_TO_MAKE_PROGRESS',
          'робот не продвигается (застрял, буксование, progress_checker)'),
    306: ('NO_VALID_CONTROL', 'контроллер не смог посчитать допустимую скорость'),
    # FollowGPSWaypoints / FollowWaypoints
    600: ('UNKNOWN', 'неизвестная ошибка waypoint_follower'),
    601: ('TASK_EXECUTOR_FAILED',
          'плагин в точке маршрута (wait_at_waypoint) завершился с ошибкой'),
    602: ('NO_VALID_WAYPOINTS', 'список точек пуст или все точки некорректны'),
    603: ('STOP_ON_MISSED_WAYPOINT',
          'stop_on_failure=true: маршрут остановлен на первой непройденной точке'),
    # Spin
    700: ('UNKNOWN', 'неизвестная ошибка разворота Spin'),
    701: ('TIMEOUT', 'разворот Spin не уложился во время'),
    702: ('TF_ERROR', 'нет трансформа TF во время разворота'),
    703: ('COLLISION_AHEAD', 'разворот прерван: препятствие впереди'),
    # BackUp
    710: ('UNKNOWN', 'неизвестная ошибка отъезда BackUp'),
    711: ('TIMEOUT', 'отъезд BackUp не уложился во время'),
    712: ('TF_ERROR', 'нет трансформа TF во время отъезда'),
    713: ('INVALID_INPUT', 'некорректные параметры отъезда BackUp'),
    714: ('COLLISION_AHEAD', 'отъезд прерван: препятствие'),
    # DriveOnHeading
    720: ('UNKNOWN', 'неизвестная ошибка DriveOnHeading'),
    721: ('TIMEOUT', 'DriveOnHeading не уложился во время'),
    722: ('TF_ERROR', 'нет трансформа TF в DriveOnHeading'),
    723: ('COLLISION_AHEAD', 'DriveOnHeading прерван: препятствие впереди'),
    724: ('INVALID_INPUT', 'некорректные параметры DriveOnHeading'),
}

GOAL_STATUS_TEXT = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def decode_nav2_error(code):
    """Человекочитаемая расшифровка кода отказа Nav2."""
    try:
        code = int(code)
    except (TypeError, ValueError):
        return f'{code} (не удалось разобрать код)'
    name, meaning = NAV2_ERROR_CODES.get(
        code, (None, 'неизвестный код Nav2'))
    if name is None:
        return f'{code} ({meaning})'
    return f'{code} {name} — {meaning}'


def yaw_to_quaternion(yaw):
    """Кватернион поворота вокруг вертикальной оси (2D-случай)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GpsWaypointCommander(Node):

    def __init__(self):
        super().__init__('gps_waypoint_commander')

        self.declare_parameter('waypoints_file', '')
        # Сколько раз повторить маршрут: 0 — проехать один раз.
        self.declare_parameter('number_of_loops', 0)
        # Автостарт без подтверждения — ТОЛЬКО для отладки на стенде.
        self.declare_parameter('autostart', False)
        # Реагировать на переключатель режимов на пульте.
        self.declare_parameter('use_rc_mode', True)
        # Не стартовать, пока GNSS не выдаст валидный фикс.
        self.declare_parameter('require_gps_fix', True)
        # Сколько секунд ждать поднятия Nav2 перед тем, как ругаться.
        self.declare_parameter('action_wait_timeout', 60.0)
        # Максимальное расстояние между соседними точками, м. Должно быть
        # меньше половины скользящего global_costmap (окно 120 м → ~50 м).
        self.declare_parameter('max_waypoint_separation', DEFAULT_MAX_WP_SEP)
        # Минимальное расстояние между соседними точками, м.
        self.declare_parameter('min_waypoint_separation', DEFAULT_MIN_WP_SEP)

        self._loops = int(self.get_parameter('number_of_loops').value)
        self._use_rc = bool(self.get_parameter('use_rc_mode').value)
        self._require_fix = bool(self.get_parameter('require_gps_fix').value)

        # --- маршрут ---------------------------------------------------------
        self._waypoints = self._load_waypoints(
            self.get_parameter('waypoints_file').value)

        # --- состояние -------------------------------------------------------
        self._goal_handle = None
        self._running = False
        self._last_fix = None
        self._current_wp = 0

        # --- интерфейсы ------------------------------------------------------
        self._client = ActionClient(
            self, FollowGPSWaypoints, 'follow_gps_waypoints')

        self.create_subscription(NavSatFix, '/gps/fix', self._on_fix, 10)

        if self._use_rc:
            self.create_subscription(
                Int8, '/control_mode', self._on_control_mode, 10)

        self.create_service(
            Trigger, '~/start_route', self._srv_start)
        self.create_service(
            Trigger, '~/cancel_route', self._srv_cancel)

        self.get_logger().info(
            f'Загружено точек маршрута: {len(self._waypoints)}. '
            f'Повторов маршрута: {self._loops}. '
            f'Ожидание команды на старт '
            f'(пульт в AUTO либо сервис ~/start_route).')

        if bool(self.get_parameter('autostart').value):
            self.get_logger().warning(
                'autostart=true — маршрут стартует автоматически! '
                'Убедитесь, что робот стоит на подставке или на открытом '
                'безопасном участке.')
            self.create_timer(5.0, self._autostart_once)

    # ------------------------------------------------------------------ загрузка
    def _load_waypoints(self, path):
        """Читает YAML и возвращает список GeoPose."""
        if not path:
            self.get_logger().error(
                'Параметр waypoints_file не задан — ехать некуда.')
            return []

        if not os.path.exists(path):
            self.get_logger().error(f'Файл маршрута не найден: {path}')
            return []

        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f'Не удалось прочитать {path}: {exc}')
            return []

        raw = data.get('waypoints') or []
        if not raw:
            self.get_logger().warning(
                f'В файле {path} маршрут пуст. Запишите точки '
                f'gps_waypoint_logger или впишите их вручную.')
            return []

        poses = []
        for i, wp in enumerate(raw, start=1):
            # Поддерживаем и новый формат (latitude/longitude), и старый
            # формат этого проекта (lat/lon).
            lat = wp.get('latitude', wp.get('lat'))
            lon = wp.get('longitude', wp.get('lon'))

            if lat is None or lon is None:
                self.get_logger().error(
                    f'Точка №{i} пропущена: нет latitude/longitude.')
                continue

            lat = float(lat)
            lon = float(lon)

            # Защита от классической опечатки — перепутанных местами
            # широты и долготы, из-за которой робот уезжает в другую страну.
            if not (-90.0 <= lat <= 90.0):
                self.get_logger().error(
                    f'Точка №{i}: широта {lat} вне диапазона -90..90. '
                    f'Возможно, широта и долгота перепутаны местами.')
                continue
            if not (-180.0 <= lon <= 180.0):
                self.get_logger().error(
                    f'Точка №{i}: долгота {lon} вне диапазона -180..180.')
                continue

            pose = GeoPose()
            pose.position.latitude = lat
            pose.position.longitude = lon
            # Высоту всегда обнуляем: в конфиге navsat_transform включён
            # zero_altitude, и стек работает строго в 2D.
            pose.position.altitude = 0.0

            qx, qy, qz, qw = yaw_to_quaternion(float(wp.get('yaw', 0.0)))
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            poses.append(pose)

        if len(poses) >= 2:
            self._check_spacings(poses)

        return poses

    def _check_spacings(self, poses):
        """Проверяет интервалы между соседними GPS-точками."""
        max_sep = float(self.get_parameter('max_waypoint_separation').value)
        min_sep = float(self.get_parameter('min_waypoint_separation').value)
        total = 0.0
        too_far = 0
        too_close = 0

        for i in range(len(poses) - 1):
            dist = self._distance(poses[i], poses[i + 1])
            total += dist
            self.get_logger().info(
                f'  участок {i + 1}→{i + 2}: {dist:.1f} м')

            if dist < min_sep:
                too_close += 1
                self.get_logger().warning(
                    f'Точки {i + 1} и {i + 2} слишком близко ({dist:.1f} м, '
                    f'минимум {min_sep:.0f} м). Бытовой GNSS даёт 2.5–5 м, '
                    f'допуск прибытия 1.5 м — робот может «проскочить» '
                    f'обе сразу.')
            if dist > max_sep:
                too_far += 1
                self.get_logger().warning(
                    f'Точки {i + 1} и {i + 2} слишком далеко ({dist:.1f} м, '
                    f'лимит {max_sep:.0f} м). Глобальный костмап — скользящее '
                    f'окно 120×120 м, робот в центре, до края ~60 м. '
                    f'Планировщик не увидит цель '
                    f'(код 204 GOAL_OUTSIDE_MAP). Поставьте промежуточную точку.')

        self.get_logger().info(
            f'Длина маршрута: {total:.0f} м по {len(poses)} точкам.')
        if too_far or too_close:
            self.get_logger().warning(
                f'Проверка интервалов: слишком далеко {too_far}, '
                f'слишком близко {too_close}. Маршрут всё равно загружен.')

    @staticmethod
    def _distance(a: GeoPose, b: GeoPose):
        """Расстояние между двумя GeoPose в метрах (плоская аппроксимация)."""
        earth_r = 6371000.0
        lat1 = math.radians(a.position.latitude)
        lat2 = math.radians(b.position.latitude)
        dlat = lat2 - lat1
        dlon = math.radians(b.position.longitude - a.position.longitude)
        x = dlon * math.cos((lat1 + lat2) / 2.0)
        return earth_r * math.hypot(x, dlat)

    # ------------------------------------------------------------------ входы
    def _on_fix(self, msg: NavSatFix):
        self._last_fix = msg

    def _has_fix(self):
        if self._last_fix is None:
            return False, 'нет сообщений в /gps/fix'
        if self._last_fix.status.status == NavSatStatus.STATUS_NO_FIX:
            return False, 'приёмник ещё не поймал спутники (STATUS_NO_FIX)'
        if math.isnan(self._last_fix.latitude) or \
                math.isnan(self._last_fix.longitude):
            return False, 'координаты равны NaN'
        return True, ''

    def _on_control_mode(self, msg: Int8):
        if msg.data == MODE_AUTO:
            if not self._running:
                self.get_logger().info('Пульт переведён в AUTO — стартуем.')
                self._start_route()
        else:
            if self._running:
                self.get_logger().info(
                    f'Пульт переведён в режим {msg.data} — маршрут отменён.')
                self._cancel_route()

    def _autostart_once(self):
        # Таймер одноразовый: гасим его сразу после первого срабатывания.
        for timer in list(self.timers):
            if timer.callback == self._autostart_once:
                timer.cancel()
        self._start_route()

    # ------------------------------------------------------------------ сервисы
    def _srv_start(self, request, response):
        ok, reason = self._start_route()
        response.success = ok
        response.message = reason
        return response

    def _srv_cancel(self, request, response):
        self._cancel_route()
        response.success = True
        response.message = 'Маршрут отменён'
        return response

    # ------------------------------------------------------------------ маршрут
    def _start_route(self):
        if self._running:
            return False, 'Маршрут уже выполняется'

        if not self._waypoints:
            msg = 'Маршрут пуст — проверьте waypoints_file'
            self.get_logger().error(msg)
            return False, msg

        if self._require_fix:
            ok, reason = self._has_fix()
            if not ok:
                msg = f'Старт отменён: {reason}'
                self.get_logger().error(msg)
                return False, msg

        if self._last_fix is not None:
            first = self._waypoints[0]
            here = GeoPose()
            here.position.latitude = float(self._last_fix.latitude)
            here.position.longitude = float(self._last_fix.longitude)
            dist0 = self._distance(here, first)
            max_sep = float(self.get_parameter('max_waypoint_separation').value)
            self.get_logger().info(
                f'До первой точки маршрута: {dist0:.1f} м')
            if dist0 > max_sep:
                self.get_logger().warning(
                    f'Первая точка в {dist0:.1f} м — за краем скользящего '
                    f'костмапа (~60 м от робота). Планировщик, скорее всего, '
                    f'вернёт 204 GOAL_OUTSIDE_MAP. Подъедьте ближе или '
                    f'добавьте промежуточную точку.')

        timeout = float(self.get_parameter('action_wait_timeout').value)
        self.get_logger().info(
            'Ожидание экшен-сервера /follow_gps_waypoints...')
        if not self._client.wait_for_server(timeout_sec=timeout):
            msg = ('Nav2 не отвечает: экшен /follow_gps_waypoints недоступен. '
                   'Проверьте, что поднят waypoint_follower и что '
                   'lifecycle-менеджер перевёл узлы в состояние active.')
            self.get_logger().error(msg)
            return False, msg

        goal = FollowGPSWaypoints.Goal()
        goal.gps_poses = self._waypoints
        goal.number_of_loops = self._loops
        goal.goal_index = 0

        self._running = True
        self._current_wp = 0

        future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

        msg = f'Маршрут отправлен в Nav2: {len(self._waypoints)} точек'
        self.get_logger().info(msg)
        return True, msg

    def _cancel_route(self):
        self._running = False
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                'Nav2 ОТКЛОНИЛ маршрут. Самая частая причина — недоступен '
                'сервис /fromLL, то есть не запущен navsat_transform или он '
                'ещё не получил GPS-фикс.')
            self._running = False
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Nav2 принял маршрут, робот поехал.')
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        idx = feedback_msg.feedback.current_waypoint
        if idx != self._current_wp:
            self._current_wp = idx
            self.get_logger().info(
                f'Едем к точке {idx + 1}/{len(self._waypoints)}')

    def _on_result(self, future):
        self._running = False
        self._goal_handle = None

        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Не удалось получить результат маршрута: {exc}')
            return

        status = int(getattr(wrapped, 'status', GoalStatus.STATUS_UNKNOWN))
        status_name = GOAL_STATUS_TEXT.get(status, f'код {status}')
        result = wrapped.result

        code = getattr(result, 'error_code', 0) or 0
        err_msg = (getattr(result, 'error_msg', '') or '').strip()
        missed = list(getattr(result, 'missed_waypoints', []) or [])

        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Маршрут отменён.')
            return

        if status == GoalStatus.STATUS_SUCCEEDED and not missed and not code:
            self.get_logger().info('Маршрут пройден полностью.')
            return

        parts = [f'Nav2 завершил маршрут со статусом {status_name}']
        if code:
            parts.append(f'код отказа: {decode_nav2_error(code)}')
        if err_msg:
            parts.append(err_msg)
        log = self.get_logger().error if status == GoalStatus.STATUS_ABORTED \
            else self.get_logger().warning
        log('. '.join(parts) + '.')

        if not missed:
            return

        self.get_logger().warning(
            f'Не пройдено точек: {len(missed)}.')
        for mw in missed:
            self.get_logger().warning(f'  {self._format_missed(mw)}')

    @staticmethod
    def _format_missed(mw):
        """Одна непройденная точка: индекс + расшифровка кода Nav2."""
        if isinstance(mw, int):
            return f'точка {mw + 1}: индекс без кода ошибки'

        idx = getattr(mw, 'index', None)
        label = f'точка {int(idx) + 1}' if isinstance(idx, int) else 'точка ?'

        bits = []
        code = getattr(mw, 'error_code', None)
        if code not in (None, 0):
            bits.append(decode_nav2_error(code))
        msg = getattr(mw, 'error_msg', None)
        if msg:
            bits.append(str(msg))
        if not bits:
            bits.append('причина не указана')
        return f'{label}: {"; ".join(bits)}'


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointCommander()
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
