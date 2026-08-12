#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 нода одометрии и IMU для робота на базе Raspberry Pi.

Аппаратура (драйвер — в imu_driver.py):
  * MPU6050 (акселерометр + гироскоп), адрес 0x68, шина I2C-1
  * Магнитометр (опционально): QMC5883L (0x0D) или HMC5883L (0x1E)

Автокалибровка при запуске (calibration_samples сэмплов):
  * гироскоп: смещение (bias) усредняется в покое и вычитается из показаний
    (критично для курса — убирает основной источник дрейфа yaw);
  * акселерометр: если робот неподвижно лежит плашмя (z-вверх),
    вычисляется смещение относительно g = (0, 0, +9.81);
  * магнитометр: аппаратной калибровки нет — при необходимости добавьте
    hard/soft-iron калибровку перед использованием.

Публикации:
  * /imu/data  (sensor_msgs/Imu)   — ориентация, угловая скорость, ускорение
  * /odom      (nav_msgs/Odometry) — позиция из joint_states + курс от EKF
  * TF odom -> base_link (если publish_tf := true)

Подписки:
  * /joint_states (sensor_msgs/JointState) — положение X/Y из суставов

Параметры:
  * robot_x_joint        (str)   — имя сустава по оси X ("robot_x")
  * robot_y_joint        (str)   — имя сустава по оси Y ("robot_y")
  * odom_frame           (str)   — кадр одометрии ("odom")
  * base_frame           (str)   — кадр робота    ("base_link")
  * publish_tf           (bool)  — публиковать TF (true)
  * imu_rate             (float) — частота опроса IMU, Гц (50.0)
  * ekf_frame            (str)   — конвенция кадра EKF ("ENU", REP-103)
  * calibration_samples  (int)   — сэмплов для автокалибровки (100)
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from ahrs.filters import EKF

from robot_odom.imu_driver import HardwareIMU


def quat_from_rpy(roll, pitch, yaw):
    """Кватернион [x, y, z, w] из углов Эйлера (рад)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


def quat_wxyz_from_rpy(roll, pitch, yaw):
    """Кватернион в порядке [w, x, y, z] (как хранит EKF) из углов RPY."""
    q = quat_from_rpy(roll, pitch, yaw)   # [x, y, z, w]
    return np.array([q[3], q[0], q[1], q[2]])


def rpy_from_quat(q):
    """Углы RPY (рад) из кватерниона [w, x, y, z]."""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_angle(a):
    """Приводит угол к диапазону (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def tilt_compensated_heading(roll, pitch, mag):
    """
    Магнитный курс робота с компенсацией наклона (roll/pitch).

    Возвращает угол (рад) в той же конвенции, что yaw EKF в ENU:
    угол от оси X против часовой (ahrs ENU: q=identity -> ось X тела = оси X мира).

    mag — вектор магнитометра в осях робота, уже выровненный
    (rotate_xy + mag_z_invert применены в драйвере).
    """
    mx, my, mz = float(mag[0]), float(mag[1]), float(mag[2])
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    # Горизонтальные компоненты поля в осях тела (после проекции на горизонт):
    xh = mx * cp + my * sr * sp + mz * cr * sp
    yh = my * cr - mz * sr
    # Курс в конвенции «от севера по часовой» (как компас телефона)...
    heading_cw_north = math.atan2(yh, xh)
    # ...переводим в ENU-yaw ahrs: ось X мира -> восток, yaw = угол от X CCW:
    return math.pi / 2.0 - heading_cw_north


class OdomNode(Node):
    def __init__(self):
        super().__init__("odom_node")

        # --- параметры -----------------------------------------------------
        self.declare_parameter("robot_x_joint", "robot_x")
        self.declare_parameter("robot_y_joint", "robot_y")
        # Источник yaw для /odom:
        #   'imu'   — курс от EKF (гироскоп+компас) — может дрейфовать;
        #   'wheel' — колёсный курс из joint_states (robot_yaw_joint) —
        #             стабилен при неподвижном роботе (из энкодеров kolesa_control);
        #   'mag'   — курс ТОЛЬКО от магнитометра (с компенсацией наклона по
        #             акселерометру), гироскоп в курсе не участвует вообще.
        self.declare_parameter("robot_yaw_joint", "robot_yaw")
        self.declare_parameter("yaw_source", "imu")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("imu_rate", 50.0)        # частота опроса IMU, Гц
        self.declare_parameter("ekf_frame", "ENU")      # ENU — конвенция ROS (REP-103)
        self.declare_parameter("calibration_samples", 100)
        # Поворот осей магнитометра относительно MPU6050/робота (град),
        # измеряется утилитой imu_check --heading. Компас — отдельный датчик.
        self.declare_parameter("mag_yaw_offset_deg", 0.0)
        # Диагностика/фикс перевёрнутой ориентации (Z вниз в RViz):
        # use_magnetometer=false — временно отключить компас (EKF только acc+gyro);
        # mag_z_invert=true — инвертировать ось Z магнитометра (QMC выдаёт
        # mz «вниз» в стиле NED, а ENU-модель ahrs ожидает mz «вверх»).
        self.declare_parameter("use_magnetometer", True)
        self.declare_parameter("mag_z_invert", False)
        # Режим «только курс»: компас влияет ТОЛЬКО на yaw (компенсированным
        # доп.фильтром), а уровень (roll/pitch) всегда строго по акселерометру.
        # Нужен, когда полный 3D-вектор магнитометра портит уровень
        # (hard-iron от моторов, перекос осей компаса, робота нельзя вращать).
        self.declare_parameter("mag_yaw_only", False)
        self.declare_parameter("mag_yaw_gain", 0.01)   # коэфф. компл. фильтра (на тик, 50 Гц)
        # Зона нечувствительности (град): не подтягивать yaw к магнитному
        # курсу, если расхождение меньше порога. Убирает джиттер от шума
        # магнитометра (моторы/лидар рядом), сохраняя отслеживание поворотов.
        self.declare_parameter("mag_yaw_deadzone_deg", 1.0)
        # Калибровка магнитометра — из imu_check --calibrate-mag.
        self.declare_parameter("mag_hard_iron_x", 0.0)
        self.declare_parameter("mag_hard_iron_y", 0.0)
        self.declare_parameter("mag_hard_iron_z", 0.0)
        self.declare_parameter("mag_scale_x", 1.0)
        self.declare_parameter("mag_scale_y", 1.0)
        self.declare_parameter("mag_scale_z", 1.0)
        # Сглаживание курса (0..1): экспоненциальное среднее по yaw.
        # 0 — выкл; 0.2 — лёгкое; 0.05 — сильное (плавный, но инертный курс).
        # Полезно в режиме без компаса (курс от гироскопа), чтобы убрать
        # джиттер от шума гироскопа.
        self.declare_parameter("yaw_smoothing", 0.0)
        # Фиксированная поправка курса (град): добавляется к итоговому yaw.
        # Удобно для компенсации остаточного смещения (hard-iron/неточный
        # mag_yaw_offset). Положительная — по часовой, отрицательная — против.
        self.declare_parameter("yaw_bias_deg", 0.0)
        # Инверсия акселерометра для ENU-модели ahrs (см. imu_driver).
        # Если при плашмя RViz показывает робота «вверх ногами» — true.
        self.declare_parameter("acc_invert", False)
        # Монтажный наклон платы IMU относительно base_link (град, RPY как в URDF).
        # Если робот стоит ровно, а RViz показывает наклон — плата установлена
        # с перекосом. Измеряется: ros2 run robot_odom imu_check --calibrate-mount
        self.declare_parameter("imu_mount_roll_deg", 0.0)
        self.declare_parameter("imu_mount_pitch_deg", 0.0)
        self.declare_parameter("imu_mount_yaw_deg", 0.0)
        # Фреймы датчиков (для robot_localization и будущего GPS)
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("mag_gps_frame", "mag_gps_link")
        self.declare_parameter("laser_frame", "laser_frame")
        # Монтаж MPU6050 относительно base_link (м, град)
        self.declare_parameter("imu_offset_x", 0.0)
        self.declare_parameter("imu_offset_y", 0.0)
        self.declare_parameter("imu_offset_z", 0.0)
        self.declare_parameter("imu_offset_roll", 0.0)
        self.declare_parameter("imu_offset_pitch", 0.0)
        self.declare_parameter("imu_offset_yaw", 0.0)
        # Монтаж корпуса GPS+компас относительно base_link (м, град).
        # Точка приёма GPS = центр корпуса (антенна встроенная).
        self.declare_parameter("mag_gps_offset_x", 0.0)
        self.declare_parameter("mag_gps_offset_y", 0.0)
        self.declare_parameter("mag_gps_offset_z", 0.0)
        self.declare_parameter("mag_gps_offset_yaw", 0.0)
        # Монтаж лидара относительно base_link (м, град) — лидар НЕ в центре.
        self.declare_parameter("laser_offset_x", 0.0)
        self.declare_parameter("laser_offset_y", 0.0)
        self.declare_parameter("laser_offset_z", 0.0)
        self.declare_parameter("laser_offset_yaw", 0.0)
        # Публикация статических TF (чтобы не дублировать издателей):
        # laser по умолчанию выключен — его обычно публикует launch/драйвер.
        self.declare_parameter("publish_imu_tf", True)
        self.declare_parameter("publish_mag_gps_tf", True)
        self.declare_parameter("publish_laser_tf", False)

        self.x_name = self.get_parameter("robot_x_joint").value
        self.y_name = self.get_parameter("robot_y_joint").value
        self.yaw_name = self.get_parameter("robot_yaw_joint").value
        self.yaw_source = str(self.get_parameter("yaw_source").value).lower()
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.imu_rate = float(self.get_parameter("imu_rate").value)
        self.ekf_frame = self.get_parameter("ekf_frame").value
        self.calibration_samples = int(self.get_parameter("calibration_samples").value)
        self.mag_yaw_offset_deg = float(self.get_parameter("mag_yaw_offset_deg").value)
        self.use_magnetometer = bool(self.get_parameter("use_magnetometer").value)
        self.mag_z_invert = bool(self.get_parameter("mag_z_invert").value)
        self.mag_yaw_only = bool(self.get_parameter("mag_yaw_only").value)
        self.mag_yaw_gain = float(self.get_parameter("mag_yaw_gain").value)
        self.mag_yaw_deadzone_deg = float(self.get_parameter("mag_yaw_deadzone_deg").value)
        self.yaw_bias_deg = float(self.get_parameter("yaw_bias_deg").value)
        self.mag_hard_iron = np.array([
            float(self.get_parameter("mag_hard_iron_x").value),
            float(self.get_parameter("mag_hard_iron_y").value),
            float(self.get_parameter("mag_hard_iron_z").value),
        ])
        self.mag_scale = np.array([
            float(self.get_parameter("mag_scale_x").value),
            float(self.get_parameter("mag_scale_y").value),
            float(self.get_parameter("mag_scale_z").value),
        ])
        self.yaw_smoothing = float(self.get_parameter("yaw_smoothing").value)
        self._yaw_smoothed = None
        self.acc_invert = bool(self.get_parameter("acc_invert").value)
        self.mount_roll = float(self.get_parameter("imu_mount_roll_deg").value)
        self.mount_pitch = float(self.get_parameter("imu_mount_pitch_deg").value)
        self.mount_yaw = float(self.get_parameter("imu_mount_yaw_deg").value)

        self.imu_frame = self.get_parameter("imu_frame").value
        self.mag_gps_frame = self.get_parameter("mag_gps_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self._sensor_offsets = {
            "imu": (float(self.get_parameter("imu_offset_x").value),
                    float(self.get_parameter("imu_offset_y").value),
                    float(self.get_parameter("imu_offset_z").value),
                    float(self.get_parameter("imu_offset_roll").value),
                    float(self.get_parameter("imu_offset_pitch").value),
                    float(self.get_parameter("imu_offset_yaw").value)),
            "mag_gps": (float(self.get_parameter("mag_gps_offset_x").value),
                        float(self.get_parameter("mag_gps_offset_y").value),
                        float(self.get_parameter("mag_gps_offset_z").value),
                        0.0, 0.0,
                        float(self.get_parameter("mag_gps_offset_yaw").value)),
            "laser": (float(self.get_parameter("laser_offset_x").value),
                      float(self.get_parameter("laser_offset_y").value),
                      float(self.get_parameter("laser_offset_z").value),
                      0.0, 0.0,
                      float(self.get_parameter("laser_offset_yaw").value)),
        }
        self._sensor_tf_enabled = {
            "imu": bool(self.get_parameter("publish_imu_tf").value),
            "mag_gps": bool(self.get_parameter("publish_mag_gps_tf").value),
            "laser": bool(self.get_parameter("publish_laser_tf").value),
        }

        self.imu = HardwareIMU(
            bus_num=1,
            logger=self.get_logger(),
            mag_yaw_offset_deg=self.mag_yaw_offset_deg,
            mag_z_invert=self.mag_z_invert,
            acc_invert=self.acc_invert,
            imu_mount_roll_deg=self.mount_roll,
            imu_mount_pitch_deg=self.mount_pitch,
            imu_mount_yaw_deg=self.mount_yaw,
            mag_hard_iron_x=float(self.mag_hard_iron[0]),
            mag_hard_iron_y=float(self.mag_hard_iron[1]),
            mag_hard_iron_z=float(self.mag_hard_iron[2]),
            mag_scale_x=float(self.mag_scale[0]),
            mag_scale_y=float(self.mag_scale[1]),
            mag_scale_z=float(self.mag_scale[2]),
        )
        if self.imu.mag_type is None:
            self.get_logger().info("Магнитометр: не найден")
        else:
            mode = "yaw-only" if self.mag_yaw_only else "используется"
            if not self.use_magnetometer:
                mode = "ОТКЛЮЧЁН — только acc+gyro"
            self.get_logger().info(
                f"Магнитометр: {self.imu.mag_type} ({mode}, "
                f"yaw_offset={self.mag_yaw_offset_deg:g}°, "
                f"z_invert={'да' if self.mag_z_invert else 'нет'})"
            )
        self.get_logger().info(
            f"yaw_source: {self.yaw_source} (robot_yaw_joint='{self.yaw_name}')"
        )
        self.get_logger().info(
            f"IMU-конфигурация: acc_invert={'ДА' if self.acc_invert else 'нет'} | "
            f"use_magnetometer={'да' if self.use_magnetometer else 'нет'} | "
            f"ekf_frame={self.ekf_frame} | "
            f"mount=({self.mount_roll:g},{self.mount_pitch:g},{self.mount_yaw:g})° | "
            f"yaw_bias={self.yaw_bias_deg:g}° | "
            f"yaw_smoothing={self.yaw_smoothing:g} | "
            f"hard_iron=({self.mag_hard_iron[0]:.0f},{self.mag_hard_iron[1]:.0f},{self.mag_hard_iron[2]:.0f})"
        )

        # Автокалибровка при старте (робот должен лежать неподвижно плашмя).
        # Ошибку не проглатываем молча: если I2C не работает — лучше упасть сразу.
        self.imu.calibrate(samples=self.calibration_samples, logger=self.get_logger())

        # Диагностика знака акселерометра: после калибровки робот должен
        # лежать плашмя, поэтому Z ≈ ±9.81. Если acc_invert=ДА, ожидается Z<0
        # (ENU-модель ahrs ждёт вектор гравитации вниз).
        acc0, _, _ = self.imu.get_data()
        self.get_logger().info(
            f"ACC после калибровки (робот плашмя): "
            f"X={acc0[0]:+.2f} Y={acc0[1]:+.2f} Z={acc0[2]:+.2f} "
            f"| acc_invert={'ДА' if self.acc_invert else 'нет'}"
        )

        # --- ВАЖНО про ahrs.filters.EKF (v0.4) -----------------------------
        # 1) Параметр mag в КОНСТРУКТОРЕ — это не данные, а флаг наличия
        #    магнитометра: от него зависит размерность вектора измерений (3 или 6)
        #    в функции h(q). Если передавать mag в update(), а в конструкторе
        #    оставить None — упадёт ValueError о несовместимости размерностей.
        # 2) В update() НЕЛЬЗЯ передавать нулевой вектор магнитометра —
        #    будет ValueError "Invalid geomagnetic field". Передавайте None.
        # 3) frequency задаёт dt = 1/frequency (по умолчанию 100 Гц!).
        #    Здесь частота таймера imu_rate, поэтому явно передаём её.
        # Магнитометр в EKF используем ТОЛЬКО если он есть, включён и НЕ в
        # режиме «только курс» (в yaw-only уровень должен быть чисто от acc).
        self._mag_full = bool(self.imu.mag_type is not None
                              and self.use_magnetometer
                              and not self.mag_yaw_only)
        self.ekf = EKF(
            frequency=self.imu_rate,
            frame=self.ekf_frame,
            mag=np.zeros(3) if self._mag_full else None,
        )
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

        self.current_yaw = 0.0
        self.current_gyro = np.array([0.0, 0.0, 0.0])
        self.current_acc = np.array([0.0, 0.0, 0.0])

        self.imu_timer = self.create_timer(1.0 / self.imu_rate, self._imu_update_callback)
        self.sub = self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu/data", 10)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            # Статические TF: монтаж датчиков относительно base_link.
            # Каждый физически отдельный датчик — свой фрейм.
            from tf2_ros import StaticTransformBroadcaster
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)
            self._publish_static_tfs()

        self.prev_time = None
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = None         # для сглаженной угловой скорости (twist)
        self.prev_yaw_wheel = None   # для twist при yaw_source='wheel'
        self._last_imu_time = None   # для измерения реального dt между тиками
        self._acc_lp = None          # низкочастотный фильтр acc (для yaw_source='mag')

    # --- статические TF датчиков ------------------------------------------

    def _publish_static_tfs(self):
        """Публикует base_link -> imu_link и base_link -> mag_gps_link
        (позиция и ориентация монтажа). Вызывается один раз при старте."""
        tfs = []
        child_map = {"imu": self.imu_frame,
                     "mag_gps": self.mag_gps_frame,
                     "laser": self.laser_frame}
        published = []
        for name, (x, y, z, roll, pitch, yaw) in self._sensor_offsets.items():
            if not self._sensor_tf_enabled.get(name, True):
                continue
            child = child_map[name]
            published.append(child)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            q = quat_from_rpy(
                math.radians(roll), math.radians(pitch), math.radians(yaw))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            tfs.append(t)
        self.static_tf_broadcaster.sendTransform(tfs)
        if published:
            self.get_logger().info(
                "Статические TF: " + ", ".join(
                    f"{self.base_frame} -> {f}" for f in published))
        else:
            self.get_logger().info("Статические TF не публикуются.")

    # --- IMU ---------------------------------------------------------------

    def _imu_update_callback(self):
        try:
            acc, gyro, mag = self.imu.get_data()
        except Exception as e:
            self.get_logger().error(f"Ошибка чтения IMU: {e}")
            return

        self.get_logger().debug(
            f"ACC(м/с²): X={acc[0]:.2f} Y={acc[1]:.2f} Z={acc[2]:.2f} | "
            f"GYR(рад/с): X={gyro[0]:.3f} Y={gyro[1]:.3f} Z={gyro[2]:.3f} | "
            f"MAG: X={mag[0]:.1f} Y={mag[1]:.1f} Z={mag[2]:.1f}"
        )

        # Реальный dt между тиками: таймер ROS не гарантирует точный период,
        # а чтение I2C — блокирующее. Значение зажимаем в разумные пределы.
        now = time.perf_counter()
        dt = None
        if self._last_imu_time is not None:
            dt = min(max(now - self._last_imu_time, 1e-3), 0.1)
        self._last_imu_time = now

        if self.yaw_source == 'mag':
            # Курс ТОЛЬКО от магнитометра (с компенсацией наклона), уровень
            # от акселерометра (низкочастотный фильтр). Гироскоп в курсе
            # не участвует — дрейфа нет.
            if not (self.imu.mag_type is not None and self.use_magnetometer):
                self.get_logger().warning(
                    "yaw_source='mag', но магнитометр недоступен — "
                    "использую EKF (гироскоп+акселерометр)")
                self._mag_only_fallback(gyro, acc, dt)
            else:
                self._update_yaw_from_mag(acc, mag)
        else:
            # Магнитометра нет/выключен/yaw-only — передаём None, а не нулевой вектор
            mag_arg = mag if self._mag_full else None
            try:
                self.q = self.ekf.update(self.q, gyro, acc, mag_arg, dt=dt)
            except ValueError as e:
                self.get_logger().warning(f"Пропуск обновления EKF: {e}")

            # Режим «только курс»: уровень уже получен от EKF (acc+gyro) без маг.
            # Теперь подтягиваем yaw к магнитному курсу комплементарным фильтром,
            # чтобы компас не влиял на roll/pitch вообще. Зона нечувствительности
            # отсекает шум магнитометра (иначе yaw дёргается на ровном месте).
            if self.mag_yaw_only and self.use_magnetometer and self.imu.mag_type is not None:
                roll, pitch, _ = rpy_from_quat(self.q)
                yaw_mag = tilt_compensated_heading(roll, pitch, mag)
                yaw_now = self._quat_yaw(self.q)
                diff = wrap_angle(yaw_mag - yaw_now)
                if abs(math.degrees(diff)) > self.mag_yaw_deadzone_deg:
                    yaw_fused = yaw_now + self.mag_yaw_gain * diff
                    self.q = quat_wxyz_from_rpy(roll, pitch, yaw_fused)

            # Фиксированная поправка курса (компенсация остаточного смещения yaw)
            if self.yaw_bias_deg:
                roll, pitch, yaw = rpy_from_quat(self.q)
                yaw = wrap_angle(yaw + math.radians(self.yaw_bias_deg))
                self.q = quat_wxyz_from_rpy(roll, pitch, yaw)

            # Экспоненциальное сглаживание курса (убирает джиттер от шума
            # гироскопа/магнитометра; yaw становится плавным, но чуть инертным)
            if self.yaw_smoothing > 0.0:
                roll, pitch, yaw = rpy_from_quat(self.q)
                if self._yaw_smoothed is None:
                    self._yaw_smoothed = yaw
                else:
                    self._yaw_smoothed = self._yaw_smoothed + \
                        self.yaw_smoothing * wrap_angle(yaw - self._yaw_smoothed)
                self.q = quat_wxyz_from_rpy(roll, pitch, self._yaw_smoothed)

        self.current_yaw = self._quat_yaw(self.q)
        self.current_gyro = gyro
        self.current_acc = acc

        # --- публикация /imu/data -----------------------------------------
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame

        imu_msg.orientation = Quaternion(x=self.q[1], y=self.q[2], z=self.q[3], w=self.q[0])
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]

        self.pub_imu.publish(imu_msg)

    def _update_yaw_from_mag(self, acc, mag):
        """Курс ТОЛЬКО от магнитометра (с компенсацией наклона).
        Уровень (roll/pitch) — из низкочастотно-фильтрованного акселерометра.
        Гироскоп в курсе не участвует — дрейфа нет."""
        if self._acc_lp is None:
            self._acc_lp = acc.copy()
        else:
            a = 0.2   # ~0.25 c при 50 Гц
            self._acc_lp = a * acc + (1.0 - a) * self._acc_lp
        a = self._acc_lp
        norm = np.linalg.norm(a)
        if norm < 1e-6:
            return
        # Уровень из acc (acc_invert применён: плашмя -> (0,0,-9.81)):
        pitch = math.asin(max(-1.0, min(1.0, a[0] / norm)))
        roll = math.atan2(-a[1], -a[2])
        # Курс из магнитометра с компенсацией наклона:
        yaw = tilt_compensated_heading(roll, pitch, mag)
        # Сглаживание курса (убирает шум магнитометра):
        if self.yaw_smoothing > 0.0:
            if self._yaw_smoothed is None:
                self._yaw_smoothed = yaw
            else:
                self._yaw_smoothed = self._yaw_smoothed + \
                    self.yaw_smoothing * wrap_angle(yaw - self._yaw_smoothed)
            yaw = self._yaw_smoothed
        if self.yaw_bias_deg:
            yaw = wrap_angle(yaw + math.radians(self.yaw_bias_deg))
        self.q = quat_wxyz_from_rpy(roll, pitch, yaw)

    def _mag_only_fallback(self, gyro, acc, dt):
        """Запасной путь для yaw_source='mag' без магнитометра: обычный EKF."""
        try:
            self.q = self.ekf.update(self.q, gyro, acc, None, dt=dt)
        except ValueError as e:
            self.get_logger().warning(f"Пропуск обновления EKF: {e}")
        if self.yaw_smoothing > 0.0:
            roll, pitch, yaw = rpy_from_quat(self.q)
            if self._yaw_smoothed is None:
                self._yaw_smoothed = yaw
            else:
                self._yaw_smoothed = self._yaw_smoothed + \
                    self.yaw_smoothing * wrap_angle(yaw - self._yaw_smoothed)
            self.q = quat_wxyz_from_rpy(roll, pitch, self._yaw_smoothed)
        if self.yaw_bias_deg:
            roll, pitch, yaw = rpy_from_quat(self.q)
            yaw = wrap_angle(yaw + math.radians(self.yaw_bias_deg))
            self.q = quat_wxyz_from_rpy(roll, pitch, yaw)

    @staticmethod
    def _quat_yaw(q):
        """Рыскание (yaw) из кватерниона [w, x, y, z]."""
        w, x, y, z = q
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # --- одометрия ---------------------------------------------------------

    def _on_joint_states(self, msg: JointState):
        try:
            idx_x = msg.name.index(self.x_name)
            idx_y = msg.name.index(self.y_name)
        except ValueError:
            return

        if len(msg.position) <= max(idx_x, idx_y):
            return

        x = float(msg.position[idx_x])
        y = float(msg.position[idx_y])

        # --- источник yaw ---------------------------------------------------
        # 'wheel': колёсный курс из joint_states (стабилен при неподвижном
        # роботе; не дрейфует, в отличие от гироскопа). Уровень (roll/pitch)
        # берём из EKF.
        if self.yaw_source == 'wheel':
            try:
                idx_yaw = msg.name.index(self.yaw_name)
                yaw = float(msg.position[idx_yaw])
            except ValueError:
                self.get_logger().warning(
                    f"Сустав '{self.yaw_name}' не найден в joint_states — "
                    "использую yaw от IMU")
                yaw = self.current_yaw
            roll, pitch, _ = rpy_from_quat(self.q)
            self.q = quat_wxyz_from_rpy(roll, pitch, yaw)
        else:
            yaw = self.current_yaw

        now = time.time()
        vx = vy = 0.0

        if self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 1e-6:
                vx = (x - self.prev_x) / dt
                vy = (y - self.prev_y) / dt

        self.prev_time = now
        self.prev_x = x
        self.prev_y = y

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        q_msg = Quaternion(x=self.q[1], y=self.q[2], z=self.q[3], w=self.q[0])
        odom.pose.pose.orientation = q_msg

        # Мировая скорость (из суставов) -> локальная (в системе base_link)
        odom.twist.twist.linear.x = vx * math.cos(yaw) + vy * math.sin(yaw)
        odom.twist.twist.linear.y = -vx * math.sin(yaw) + vy * math.cos(yaw)
        # Угловая скорость по yaw — производная курса (не сырой гироскоп,
        # иначе шум/дрейф по Z попадает в twist и odom «крутится»).
        prev_yaw_ref = self.prev_yaw_wheel if self.yaw_source == 'wheel' else self.prev_yaw
        if prev_yaw_ref is not None and dt > 1e-6:
            odom.twist.twist.angular.z = wrap_angle(yaw - prev_yaw_ref) / dt
        else:
            odom.twist.twist.angular.z = 0.0
        if self.yaw_source == 'wheel':
            self.prev_yaw_wheel = yaw
        else:
            self.prev_yaw = yaw

        # Ковариации нужны robot_localization.
        # Диагональ матрицы 6x6: индексы 0, 7, 14, 21, 28, 35.
        odom.pose.covariance[0] = 0.01    # X
        odom.pose.covariance[7] = 0.01    # Y
        odom.pose.covariance[35] = 0.05   # Yaw
        odom.twist.covariance[0] = 0.05   # vx
        odom.twist.covariance[7] = 0.05   # vy
        odom.twist.covariance[35] = 0.05  # wz

        self.pub_odom.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation = q_msg
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy уже мог сам завершить контекст (например, при остановке через
        # launch по SIGINT). Повторный shutdown бросает RCLError — игнорируем.
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
