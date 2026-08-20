#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
nav_preflight_check — проверка готовности стека ПЕРЕД выездом.

Автономный робот на улице ломается дорого. Этот узел за 15 секунд проверяет
всё, что обычно и оказывается причиной неудачного заезда, и печатает понятный
чеклист. Запускать после bringup, до того как переводить пульт в AUTO:

    ros2 run robot_navigation nav_preflight_check

Что проверяется:
  1. Идут ли данные со всех датчиков (GNSS, IMU, магнитометр, лидар, одометрия).
  2. Есть ли валидный GPS-фикс и сколько спутников.
  3. Содержит ли /imu/data ориентацию (иначе EKF не получит курс).
  4. Собрана ли TF-цепочка map -> odom -> base_link.
  5. Публикуется ли /odometry/gps (то есть работает ли navsat_transform).
  6. Поднят ли экшен Nav2 /follow_gps_waypoints.
  7. Согласован ли курс IMU с направлением движения по GPS (самая коварная
     ошибка: перепутанные оси магнитометра или неучтённое склонение).
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav2_msgs.action import FollowGPSWaypoints
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, MagneticField, NavSatFix, NavSatStatus

import tf2_ros


GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
RESET = '\033[0m'


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PreflightCheck(Node):

    def __init__(self):
        super().__init__('nav_preflight_check')

        self.declare_parameter('duration', 15.0)
        # Nav2 можно намеренно не запускать (use_navigation:=false),
        # когда отлаживается только локализация. Тогда его отсутствие
        # не должно выглядеть как поломка.
        self.declare_parameter('expect_nav2', True)

        self._failures = []
        self._warnings = []

        self._counts = {}
        self._last = {}

        self._subscribe(NavSatFix, '/gps/fix', qos_profile_sensor_data)
        self._subscribe(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self._subscribe(MagneticField, '/imu/mag', 10)
        self._subscribe(Imu, '/imu/data', 10)
        self._subscribe(Odometry, '/odom', 10)
        self._subscribe(Odometry, '/odometry/local', 10)
        self._subscribe(Odometry, '/odometry/global', 10)
        self._subscribe(Odometry, '/odometry/gps', 10)
        self._subscribe(LaserScan, '/scan_reliable', qos_profile_sensor_data)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._client = ActionClient(
            self, FollowGPSWaypoints, 'follow_gps_waypoints')

        duration = float(self.get_parameter('duration').value)
        self.get_logger().info(
            f'Сбор данных {duration:.0f} секунд, не трогайте робота...')
        self._timer = self.create_timer(duration, self._report)

    def _subscribe(self, msg_type, topic, qos):
        self._counts[topic] = 0

        def cb(msg, t=topic):
            self._counts[t] += 1
            self._last[t] = msg

        self.create_subscription(msg_type, topic, cb, qos)

    # ------------------------------------------------------------------ вывод
    def _line(self, ok, title, detail=''):
        """Печатает пункт чеклиста и запоминает результат для итога."""
        if ok is True:
            mark = f'{GREEN}[ OK ]{RESET}'
        elif ok is None:
            mark = f'{YELLOW}[ ?? ]{RESET}'
            self._warnings.append(title)
        else:
            mark = f'{RED}[FAIL]{RESET}'
            self._failures.append(title)
        print(f'{mark} {title}')
        if detail:
            for row in detail.split('\n'):
                print(f'       {row}')

    def _report(self):
        self._timer.cancel()
        duration = float(self.get_parameter('duration').value)

        print('\n' + '=' * 72)
        print('  ПРОВЕРКА ГОТОВНОСТИ К АВТОНОМНОМУ ЗАЕЗДУ')
        print('=' * 72)

        self._check_sensors(duration)
        self._check_gps()
        self._check_orientation()
        self._check_tf()
        self._check_navsat()
        self._check_nav2()
        self._check_heading_consistency()

        self._summary()
        rclpy.shutdown()

    def _summary(self):
        print()
        print('=' * 72)
        if self._failures:
            print(f'  {RED}ИТОГ: провалено пунктов — '
                  f'{len(self._failures)}{RESET}')
            for title in self._failures:
                print(f'    - {title}')
            print()
            print('  ВЫЕЗЖАТЬ НЕЛЬЗЯ, пока это не устранено.')
        elif self._warnings:
            print(f'  {YELLOW}ИТОГ: замечаний — {len(self._warnings)}{RESET}')
            for title in self._warnings:
                print(f'    - {title}')
            print()
            print('  Критичных отказов нет, но перечисленное стоит проверить.')
        else:
            print(f'  {GREEN}ИТОГ: все пункты пройдены.{RESET}')
            print('  Можно переводить пульт в AUTO.')
        print('=' * 72)
        print()

    def _check_sensors(self, duration):
        print('\n--- Датчики ---')
        expectations = {
            '/gps/fix': 0.5,
            '/imu/data_raw': 20.0,
            '/imu/mag': 5.0,
            '/imu/data': 20.0,
            '/odom': 5.0,
            '/scan_reliable': 3.0,
        }
        hints = {
            '/gps/fix': 'Проверьте питание и порт GNSS (nmea_navsat_driver).',
            '/imu/data_raw': 'Не запущен mpu6050_control либо не сделан '
                             'ремап /imu/data -> /imu/data_raw.',
            '/imu/mag': 'Не запущен compass_control или mag_declination_node.',
            '/imu/data': 'Не запущен imu_filter_madgwick — курса не будет.',
            '/odom': 'Не запущен robot_odom (или kolesa_control не публикует '
                     '/joint_states).',
            '/scan_reliable': 'Не запущен лидар или relay_reliable.',
        }

        for topic, expected_hz in expectations.items():
            count = self._counts.get(topic, 0)
            hz = count / duration
            if count == 0:
                self._line(False, f'{topic}: нет данных', hints[topic])
            elif hz < expected_hz * 0.5:
                self._line(
                    None,
                    f'{topic}: {hz:.1f} Гц (ожидалось ~{expected_hz:.0f} Гц)',
                    'Частота занижена. Проверьте загрузку CPU и шину I2C/USB.')
            else:
                self._line(True, f'{topic}: {hz:.1f} Гц')

    def _check_gps(self):
        print('\n--- GNSS ---')
        fix = self._last.get('/gps/fix')
        if fix is None:
            self._line(False, 'GPS-фикс', 'Сообщений нет вообще.')
            return

        if fix.status.status == NavSatStatus.STATUS_NO_FIX:
            self._line(
                False, 'GPS-фикс отсутствует',
                'Приёмник видит спутники, но решения нет. Вынесите робота на\n'
                'открытое место и подождите: холодный старт занимает до 2 минут.')
            return

        cov = fix.position_covariance[0]
        acc = math.sqrt(cov) if cov > 0 else float('nan')

        detail = f'широта {fix.latitude:.7f}, долгота {fix.longitude:.7f}'
        if not math.isnan(acc):
            detail += f'\nоценка точности по горизонтали: ~{acc:.1f} м'

        if not math.isnan(acc) and acc > 10.0:
            self._line(
                None, 'GPS-фикс есть, но точность плохая',
                detail + '\nПри точности хуже 10 м робот будет вилять. '
                         'Дождитесь большего числа спутников.')
        else:
            self._line(True, 'GPS-фикс валиден', detail)

    def _check_orientation(self):
        print('\n--- Ориентация ---')
        imu = self._last.get('/imu/data')
        if imu is None:
            self._line(
                False, 'Нет /imu/data',
                'Без абсолютного курса уличная навигация невозможна.')
            return

        if imu.orientation_covariance[0] < 0:
            self._line(
                False, '/imu/data не содержит ориентацию',
                'Похоже, в /imu/data попадает сырой поток MPU6050, а не выход\n'
                'imu_filter_madgwick. Проверьте ремапы в launch-файле.')
            return

        yaw = quaternion_to_yaw(imu.orientation)
        azimuth = (90.0 - math.degrees(yaw)) % 360.0
        self._line(
            True, 'Ориентация публикуется',
            f'курс ENU: {math.degrees(yaw):+.1f}° '
            f'(азимут по компасу: {azimuth:.0f}°)\n'
            f'СВЕРЬТЕ азимут с компасом в телефоне. Расхождение больше 15°\n'
            f'означает неверную калибровку магнитометра или склонение.')

    def _check_tf(self):
        print('\n--- Дерево TF ---')
        for parent, child in (('map', 'odom'), ('odom', 'base_link')):
            try:
                self._tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time())
                self._line(True, f'{parent} -> {child}')
            except Exception as exc:
                hint = ('map -> odom публикует ekf_filter_node_map.'
                        if parent == 'map'
                        else 'odom -> base_link публикует ekf_filter_node_odom.\n'
                             'Убедитесь, что publish_tf выключен у robot_odom, '
                             'иначе трансформ публикуют двое.')
                self._line(False, f'{parent} -> {child} отсутствует',
                           f'{hint}\n{exc}')

    def _check_navsat(self):
        print('\n--- navsat_transform ---')
        if self._counts.get('/odometry/gps', 0) > 0:
            self._line(True, '/odometry/gps публикуется')
        else:
            self._line(
                False, '/odometry/gps молчит',
                'navsat_transform не смог связать GPS с фреймом map.\n'
                'Обычные причины: нет фикса, нет курса в /imu/data,\n'
                'либо не пришла отфильтрованная одометрия /odometry/global.')

        if self._counts.get('/odometry/global', 0) > 0:
            self._line(True, '/odometry/global публикуется (глобальный EKF)')
        else:
            self._line(False, '/odometry/global молчит',
                       'Не работает ekf_filter_node_map.')

    def _check_nav2(self):
        print('\n--- Nav2 ---')
        expect = bool(self.get_parameter('expect_nav2').value)

        if self._client.wait_for_server(timeout_sec=5.0):
            self._line(True, 'Экшен /follow_gps_waypoints доступен')
        elif not expect:
            self._line(
                None, 'Nav2 не запущен (проверка отключена параметром)',
                'Это ожидаемо при expect_nav2:=false.')
        else:
            self._line(
                False, 'Экшен /follow_gps_waypoints недоступен',
                'Если вы запускали bringup с use_navigation:=false — это\n'
                'ожидаемо, Nav2 просто не поднимали. Тогда запустите проверку\n'
                'так: ros2 run robot_navigation nav_preflight_check \\\n'
                '        --ros-args -p expect_nav2:=false\n'
                '\n'
                'Иначе Nav2 не поднялся или lifecycle-менеджер не активировал\n'
                'узлы. Смотрите: ros2 lifecycle get /waypoint_follower')

    def _check_heading_consistency(self):
        print('\n--- Согласованность курса и GPS ---')
        imu = self._last.get('/imu/data')
        odom = self._last.get('/odometry/global')

        if imu is None or odom is None:
            self._line(None, 'Проверка невозможна',
                       'Нужны и /imu/data, и /odometry/global.')
            return

        yaw_imu = quaternion_to_yaw(imu.orientation)
        yaw_odom = quaternion_to_yaw(odom.pose.pose.orientation)
        diff = math.degrees(
            math.atan2(math.sin(yaw_imu - yaw_odom),
                       math.cos(yaw_imu - yaw_odom)))

        if abs(diff) < 15.0:
            self._line(True, f'Курс EKF совпадает с IMU (расхождение {diff:+.1f}°)')
        else:
            self._line(
                False, f'Курс EKF расходится с IMU на {diff:+.1f}°',
                'EKF ещё не сошёлся либо конфликтуют источники курса.\n'
                'Проедьте 10-20 метров по прямой и проверьте снова:\n'
                'фильтру нужно движение, чтобы связать курс с перемещением.')


def main(args=None):
    rclpy.init(args=args)
    node = PreflightCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
