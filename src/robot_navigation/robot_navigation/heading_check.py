#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
heading_check — сверка курса робота с реальным азимутом.

ЗАЧЕМ
-----
Даже после калибровки магнитометра остаётся ещё одна поправка: плата
магнитометра почти никогда не смотрит ровно туда же, куда «нос» робота.
Эта утилита сравнивает курс, который выдаёт робот, с настоящим азимутом,
и печатает готовое значение угла монтажа.

ВАЖНО: сначала калибровка, потом угол монтажа. Пока магнитометр не
откалиброван, ошибка курса МЕНЯЕТСЯ при повороте робота, и никакой
постоянной поправкой её не убрать — вы просто подгоните робота под одно
направление, а во всех остальных он будет врать по-разному.

КАК ПОЛЬЗОВАТЬСЯ
----------------
1. Поставьте робота на ровное открытое место.

2. Определите, куда смотрит его «нос». Возьмите телефон с компасом,
   положите его НА робота вдоль корпуса, носом вперёд, и запишите азимут.
   Телефон при этом уберите от металла робота на 20-30 см и держите
   горизонтально.

3. Запустите:

       ros2 run robot_navigation heading_check \\
           --ros-args -p true_azimuth_deg:=210

4. Утилита усреднит курс за несколько секунд и напечатает поправку.

5. Впишите её в config/mag_calibration.yaml, перезапустите робота
   и ПРОВЕРЬТЕ ЕЩЁ РАЗ, развернув робота в другую сторону. Если после
   поправки курс сходится во всех направлениях — калибровка удалась.
   Если сходится только в одном — вернитесь к mag_calibrator.

ОБОЗНАЧЕНИЯ
-----------
Азимут — угол по часовой стрелке от направления на север:
    север 0°, восток 90°, юг 180°, запад 270°.
Именно так показывает компас в телефоне.

ROS внутри использует другое соглашение (ENU yaw): отсчёт против часовой
стрелки от направления на восток. Пересчёт: азимут = 90° - yaw.
"""

import math
import statistics

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu


GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BOLD = '\033[1m'
RESET = '\033[0m'


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def circular_mean_deg(angles_deg):
    """Среднее для углов. Обычное среднее для 359° и 1° дало бы 180°."""
    s = sum(math.sin(math.radians(a)) for a in angles_deg)
    c = sum(math.cos(math.radians(a)) for a in angles_deg)
    return math.degrees(math.atan2(s, c)) % 360.0


class HeadingCheck(Node):

    def __init__(self):
        super().__init__('heading_check')

        # Отрицательное значение = азимут не задан, работаем в режиме
        # простого показа курса.
        self.declare_parameter('true_azimuth_deg', -1.0)
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('imu_topic', '/imu/data')
        # Текущее значение из конфига — чтобы напечатать НОВОЕ, а не дельту.
        self.declare_parameter('current_mounting_yaw_deg', 0.0)

        self._true_az = float(self.get_parameter('true_azimuth_deg').value)
        self._duration = float(self.get_parameter('duration').value)
        self._current_mount = float(
            self.get_parameter('current_mounting_yaw_deg').value)

        self._azimuths = []
        self._no_orientation = False

        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu, 10)

        print()
        print(f'{BOLD}СВЕРКА КУРСА{RESET}')
        print('=' * 66)
        if self._true_az < 0:
            print('Азимут не задан — просто показываю текущий курс робота.')
            print('Чтобы получить поправку, укажите реальный азимут:')
            print('  --ros-args -p true_azimuth_deg:=<градусы по компасу>')
        else:
            print(f'Реальный азимут (задан вами): {self._true_az:.1f}°')
        print(f'Измеряю {self._duration:.0f} секунд. Робота НЕ ДВИГАТЬ.')
        print('=' * 66)

        self.create_timer(self._duration, self._finish)

    def _on_imu(self, msg: Imu):
        if msg.orientation_covariance[0] < 0:
            self._no_orientation = True
            return
        yaw = quaternion_to_yaw(msg.orientation)
        self._azimuths.append((90.0 - math.degrees(yaw)) % 360.0)

    def _finish(self):
        print()

        if self._no_orientation and not self._azimuths:
            print(f'{RED}В /imu/data нет ориентации.{RESET}')
            print('Похоже, туда попадает сырой поток MPU6050, а не выход')
            print('imu_filter_madgwick. Проверьте, что запущена локализация.')
            rclpy.shutdown()
            return

        if len(self._azimuths) < 5:
            print(f'{RED}Слишком мало данных ({len(self._azimuths)}).{RESET}')
            print('Проверьте, что публикуется /imu/data:')
            print('  ros2 topic hz /imu/data')
            rclpy.shutdown()
            return

        mean_az = circular_mean_deg(self._azimuths)

        # Разброс: показывает, насколько курс дрожит от помех.
        deviations = [
            abs((a - mean_az + 180.0) % 360.0 - 180.0) for a in self._azimuths]
        spread = statistics.mean(deviations)

        print('=' * 66)
        print(f'{BOLD}РЕЗУЛЬТАТ{RESET}')
        print('=' * 66)
        print(f'Робот показывает азимут: {mean_az:.1f}°')
        print(f'Дрожание курса:          ±{spread:.1f}° '
              f'({len(self._azimuths)} измерений)')

        if spread > 5.0:
            print(f'{YELLOW}Курс заметно дрожит.{RESET} Обычно это наводка '
                  f'от силовой части.')
            print('Попробуйте обесточить моторы и повторить: если дрожание')
            print('исчезнет, магнитометр надо унести дальше от проводов.')

        if self._true_az < 0:
            print()
            print('Сравните это значение с компасом в телефоне.')
            print('Если расходится — перезапустите с параметром:')
            print(f'  --ros-args -p true_azimuth_deg:=<реальный азимут>')
            print('=' * 66)
            rclpy.shutdown()
            return

        # Ошибка в диапазоне -180..+180
        error = (self._true_az - mean_az + 180.0) % 360.0 - 180.0

        print(f'Реальный азимут:         {self._true_az:.1f}°')
        print(f'Ошибка:                  {error:+.1f}°')
        print()

        if abs(error) < 5.0:
            print(f'{GREEN}Курс совпадает. Поправка не нужна.{RESET}')
            print('=' * 66)
            rclpy.shutdown()
            return

        # Поворот вектора поля на +φ увеличивает азимут на φ,
        # поэтому нужная добавка равна самой ошибке.
        new_mount = (self._current_mount + error + 180.0) % 360.0 - 180.0

        print('=' * 66)
        print(f'{BOLD}ВПИШИТЕ В config/mag_calibration.yaml{RESET}')
        print('=' * 66)
        print()
        print('mag_declination_node:')
        print('  ros__parameters:')
        print(f'    mounting_yaw_deg: {new_mount:.2f}')
        print()
        print('=' * 66)
        print('ПОСЛЕ ЭТОГО ОБЯЗАТЕЛЬНО ПРОВЕРЬТЕ ЕЩЁ РАЗ:')
        print()
        print('  1. Перезапустите робота.')
        print('  2. Разверните его в ДРУГУЮ сторону (например, на 90°).')
        print('  3. Запустите heading_check с новым азимутом.')
        print()
        print('Если курс сошёлся и во второй раз — всё готово.')
        print('Если во второй раз ошибка снова большая, значит магнитометр')
        print('не откалиброван: ошибка меняется при повороте, и постоянной')
        print('поправкой её не убрать. Вернитесь к mag_calibrator.')
        print('=' * 66)

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HeadingCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nПрервано пользователем.')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
