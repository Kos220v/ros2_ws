#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mag_declination_node — учёт магнитного склонения.

ЗАЧЕМ ЭТОТ УЗЕЛ НУЖЕН
---------------------
Магнитометр показывает направление на МАГНИТНЫЙ северный полюс, а GPS работает
в системе координат, привязанной к ГЕОГРАФИЧЕСКОМУ (истинному) северу. Угол
между ними называется магнитным склонением и в средней полосе России достигает
11-13°. Если его не учесть, робот будет считать, что едет прямо на точку, а по
GPS будет систематически уходить вбок примерно на 20 см на каждый метр пути —
классическая причина «робот кружит вокруг точки и не может доехать».

ПОЧЕМУ ПОПРАВКА ЗДЕСЬ, А НЕ В navsat_transform
----------------------------------------------
У navsat_transform есть параметр magnetic_declination_radians, но он влияет
только на разовый расчёт связи map <-> UTM. Сам же курс, которым пользуются оба
EKF и Nav2, остался бы магнитным. Получилось бы, что позиция робота считается в
истинном ENU, а его курс — в магнитном, и фильтр вечно борется сам с собой.
Поэтому склонение снимается ДО фильтра ориентации: тогда весь стек целиком
работает в истинном ENU и остаётся согласованным.

КАК ЭТО РАБОТАЕТ
----------------
Узел разворачивает измеренный вектор магнитного поля вокруг оси Z на угол
склонения. После разворота вектор указывает на ИСТИННЫЙ север, и
imu_filter_madgwick выдаёт сразу истинный курс.

Знак: восточное склонение (магнитный север правее истинного) задаётся
ПОЛОЖИТЕЛЬНЫМ числом — так его и публикуют геофизические сервисы.

Вход:  /imu/mag_raw (sensor_msgs/MagneticField от compass_control)
Выход: /imu/mag     (sensor_msgs/MagneticField, скорректированный)

ГДЕ ВЗЯТЬ ЗНАЧЕНИЕ СКЛОНЕНИЯ
----------------------------
Калькулятор NOAA: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
Введите широту/долготу места испытаний и текущую дату, возьмите поле
"Declination". Склонение меняется примерно на 0.1° в год, так что раз в
несколько лет значение стоит обновлять.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import MagneticField


class MagDeclinationNode(Node):
    """Разворачивает вектор магнитного поля на угол склонения."""

    def __init__(self):
        super().__init__('mag_declination_node')

        # Склонение в ГРАДУСАХ, восточное — положительное.
        # По умолчанию 0.0: пока значение не задано, узел работает как простой
        # ретранслятор и честно предупреждает об этом в лог.
        self.declare_parameter('declination_deg', 0.0)

        self.declare_parameter('input_topic', '/imu/mag_raw')
        self.declare_parameter('output_topic', '/imu/mag')

        # Отбрасывать заведомо битые измерения (обрыв I2C даёт нулевой вектор,
        # а сильная наводка от моторов — неправдоподобно большой модуль поля).
        # Магнитное поле Земли: 25-65 мкТл = 25e-6 .. 65e-6 Тл.
        self.declare_parameter('min_field_strength', 5.0e-6)
        self.declare_parameter('max_field_strength', 300.0e-6)

        decl_deg = float(self.get_parameter('declination_deg').value)
        self._decl = math.radians(decl_deg)
        self._cos = math.cos(self._decl)
        self._sin = math.sin(self._decl)

        self._min_field = float(self.get_parameter('min_field_strength').value)
        self._max_field = float(self.get_parameter('max_field_strength').value)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

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
        # Раз в 10 секунд сообщаем, сколько измерений отбраковано, — это
        # лучший индикатор наводок от силовой части робота.
        self._report_timer = self.create_timer(10.0, self._report)

        if abs(decl_deg) < 1e-9:
            self.get_logger().warning(
                'declination_deg = 0. Магнитное склонение НЕ учитывается! '
                'Робот будет систематически уходить вбок от линии маршрута. '
                'Узнайте склонение для своего района на '
                'https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml '
                'и задайте параметр declination_deg.')
        else:
            self.get_logger().info(
                f'Магнитное склонение: {decl_deg:+.2f}° '
                f'({self._decl:+.5f} рад). {in_topic} -> {out_topic}')

    def _on_mag(self, msg: MagneticField):
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z

        strength = math.sqrt(mx * mx + my * my + mz * mz)
        if not (self._min_field <= strength <= self._max_field):
            self._rejected += 1
            return

        out = MagneticField()
        out.header = msg.header

        # Поворот вокруг Z на +склонение: вектор, смотревший на магнитный
        # север, начинает смотреть на истинный.
        out.magnetic_field.x = mx * self._cos - my * self._sin
        out.magnetic_field.y = mx * self._sin + my * self._cos
        out.magnetic_field.z = mz
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
            level = self.get_logger().warning if share > 5.0 \
                else self.get_logger().info
            level(
                f'Отбраковано {self._rejected}/{total} измерений магнитометра '
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
