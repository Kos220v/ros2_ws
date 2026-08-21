#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mag_declination_node — обработка данных магнитометра.

Узел выполняет ЧЕТЫРЕ преобразования подряд, превращая сырое магнитное поле
в вектор, из которого imu_filter_madgwick получит истинный курс:

    /imu/mag_raw
        |
        1. вычитание смещения нуля      (hard iron)
        2. выравнивание масштаба осей   (soft iron)
        3. поворот на угол монтажа      (mounting_yaw_deg)
        4. поворот на магнитное склонение (declination_deg)
        |
    /imu/mag  -->  imu_filter_madgwick  -->  /imu/data

ПОЧЕМУ КАЛИБРОВКА ЗДЕСЬ, А НЕ В ДРАЙВЕРЕ compass_control
--------------------------------------------------------
У драйвера свои параметры mag_hard_iron_*, но они задаются в «сырых»
единицах АЦП (LSB), а в топик он публикует уже переведённое значение в
теслах. Смешивать две системы единиц — верный способ ошибиться на порядок.
Здесь всё считается в теслах, ровно в тех же числах, которые печатает
утилита калибровки. Параметры драйвера при этом остаются нулевыми.

ЗАЧЕМ НУЖЕН КАЖДЫЙ ШАГ
----------------------
1. Смещение нуля (hard iron). Металл корпуса, аккумулятор и постоянные
   магниты моторов создают собственное поле, которое едет вместе с роботом.
   Оно смещает центр «шара» измерений из нуля. Именно это самая частая
   причина ошибки курса на десятки градусов, причём ошибка МЕНЯЕТСЯ при
   повороте робота — по одному замеру её не вычислить.

2. Масштаб осей (soft iron). Железо рядом с датчиком искажает поле
   неодинаково по осям, и «шар» превращается в эллипсоид. Выравнивание
   масштабов возвращает ему форму шара.

3. Угол монтажа. Магнитометр — отдельная плата, и его оси почти никогда
   не смотрят туда же, куда оси робота. Обычно это поворот, кратный 90°.

4. Магнитное склонение. Угол между магнитным и географическим полюсом,
   в средней полосе 11-13°. Без него робот систематически уходит вбок
   от линии маршрута.

ЗНАКИ ПОВОРОТОВ
---------------
Поворот вектора поля на угол +φ вокруг оси Z УМЕНЬШАЕТ вычисленный курс
(yaw) на φ, то есть УВЕЛИЧИВАЕТ азимут на φ. Отсюда правило подбора угла
монтажа: mounting_yaw_deg = (истинный азимут) - (показанный азимут).
Именно по этой формуле считает утилита heading_check.

КАК КАЛИБРОВАТЬ
---------------
    ros2 run robot_navigation mag_calibrator     # шаг 1: смещение и масштаб
    ros2 run robot_navigation heading_check ...  # шаг 2: угол монтажа

Подробности — в README пакета, раздел «Ввод в эксплуатацию».
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import MagneticField


class MagDeclinationNode(Node):
    """Калибровка, выравнивание и склонение для вектора магнитного поля."""

    def __init__(self):
        super().__init__('mag_declination_node')

        # --- склонение (задаётся аргументом launch, зависит от местности) ---
        self.declare_parameter('declination_deg', 0.0)

        # --- угол монтажа платы магнитометра относительно осей робота -------
        self.declare_parameter('mounting_yaw_deg', 0.0)

        # --- калибровка: смещение нуля, тесла (свойство конкретного робота) -
        self.declare_parameter('hard_iron_x', 0.0)
        self.declare_parameter('hard_iron_y', 0.0)
        self.declare_parameter('hard_iron_z', 0.0)

        # --- калибровка: масштаб осей (безразмерный) ------------------------
        self.declare_parameter('soft_iron_scale_x', 1.0)
        self.declare_parameter('soft_iron_scale_y', 1.0)
        self.declare_parameter('soft_iron_scale_z', 1.0)

        self.declare_parameter('input_topic', '/imu/mag_raw')
        self.declare_parameter('output_topic', '/imu/mag')

        # Отбрасывать заведомо битые измерения (обрыв I2C даёт нулевой вектор,
        # а сильная наводка от моторов — неправдоподобно большой модуль поля).
        # Магнитное поле Земли: 25-65 мкТл. Границы взяты с большим запасом,
        # потому что проверка идёт по СЫРОМУ вектору, ещё со смещением нуля.
        self.declare_parameter('min_field_strength', 1.0e-6)
        self.declare_parameter('max_field_strength', 1000.0e-6)

        g = self.get_parameter

        decl_deg = float(g('declination_deg').value)
        mount_deg = float(g('mounting_yaw_deg').value)

        self._offset = (
            float(g('hard_iron_x').value),
            float(g('hard_iron_y').value),
            float(g('hard_iron_z').value),
        )
        self._scale = (
            float(g('soft_iron_scale_x').value),
            float(g('soft_iron_scale_y').value),
            float(g('soft_iron_scale_z').value),
        )

        # Оба поворота вокруг Z складываются в один — считаем синус и косинус
        # суммарного угла один раз, а не на каждом сообщении.
        total_deg = mount_deg + decl_deg
        total_rad = math.radians(total_deg)
        self._cos = math.cos(total_rad)
        self._sin = math.sin(total_rad)

        self._min_field = float(g('min_field_strength').value)
        self._max_field = float(g('max_field_strength').value)

        in_topic = g('input_topic').value
        out_topic = g('output_topic').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(MagneticField, out_topic, qos)
        self._sub = self.create_subscription(
            MagneticField, in_topic, self._on_mag, qos)

        self._rejected = 0
        self._accepted = 0
        self._report_timer = self.create_timer(10.0, self._report)

        # --- стартовая сводка: видно, что реально применяется ---------------
        self.get_logger().info(f'{in_topic} -> {out_topic}')
        self.get_logger().info(
            f'Склонение: {decl_deg:+.2f}°, угол монтажа: {mount_deg:+.2f}°, '
            f'суммарный поворот: {total_deg:+.2f}°')

        calibrated = any(abs(v) > 1e-12 for v in self._offset) or \
            any(abs(s - 1.0) > 1e-9 for s in self._scale)

        if calibrated:
            self.get_logger().info(
                f'Калибровка: смещение '
                f'({self._offset[0] * 1e6:+.2f}, {self._offset[1] * 1e6:+.2f}, '
                f'{self._offset[2] * 1e6:+.2f}) мкТл, масштаб '
                f'({self._scale[0]:.3f}, {self._scale[1]:.3f}, '
                f'{self._scale[2]:.3f})')
        else:
            self.get_logger().warning(
                'Магнитометр НЕ ОТКАЛИБРОВАН (смещение нуля равно нулю). '
                'Курс будет врать на десятки градусов, причём по-разному '
                'в разные стороны. Запустите: '
                'ros2 run robot_navigation mag_calibrator')

        if abs(decl_deg) < 1e-9:
            self.get_logger().warning(
                'declination_deg = 0. Магнитное склонение НЕ учитывается! '
                'Робот будет систематически уходить вбок от линии маршрута. '
                'Узнайте склонение для своего района на '
                'https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml')

    def _on_mag(self, msg: MagneticField):
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z

        strength = math.sqrt(mx * mx + my * my + mz * mz)
        if not (self._min_field <= strength <= self._max_field):
            self._rejected += 1
            return

        # 1) смещение нуля, 2) масштаб осей
        cx = (mx - self._offset[0]) * self._scale[0]
        cy = (my - self._offset[1]) * self._scale[1]
        cz = (mz - self._offset[2]) * self._scale[2]

        out = MagneticField()
        out.header = msg.header

        # 3) и 4) единый поворот вокруг Z: угол монтажа + склонение
        out.magnetic_field.x = cx * self._cos - cy * self._sin
        out.magnetic_field.y = cx * self._sin + cy * self._cos
        out.magnetic_field.z = cz
        out.magnetic_field_covariance = msg.magnetic_field_covariance

        self._accepted += 1
        self._pub.publish(out)

    def _report(self):
        total = self._accepted + self._rejected
        if total == 0:
            self.get_logger().warning(
                'Нет данных с магнитометра. Проверьте, что запущен '
                'compass_control и что он публикует в /imu/mag_raw '
                '(параметр topic).')
            return

        if self._rejected > 0:
            share = 100.0 * self._rejected / total
            log = self.get_logger().warning if share > 5.0 \
                else self.get_logger().info
            log(f'Отбраковано {self._rejected}/{total} измерений магнитометра '
                f'({share:.1f}%). Причина обычно одна: наводка от силовых '
                f'проводов или моторов. Отнесите магнитометр дальше от них.')

        self._accepted = 0
        self._rejected = 0


def main(args=None):
    rclpy.init(args=args)
    node = MagDeclinationNode()
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
