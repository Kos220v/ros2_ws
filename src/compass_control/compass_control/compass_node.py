#!/usr/bin/env python3
"""
Нода compass_control (QMC5883L / HMC5883L-совместимый компас на I2C).

ИСПРАВЛЕНО / ДОБАВЛЕНО (см. правки для проекта ROS2 гусеничного робота):
  1. Раньше нода публиковала ТОЛЬКО сырое магнитное поле (/magnetic_field) —
     эти данные никто в проекте не использовал. Компас был физически
     подключён, но фактически не участвовал в навигации.
  2. Теперь нода САМА считает курс (heading) из магнитометра, калибрует
     жёсткие/мягкие искажения (hard-iron/soft-iron) и сглаживает курс
     комплементарным фильтром с гироскопом (Z) из /imu.
  3. Публикует:
       /magnetic_field  (sensor_msgs/MagneticField) — как раньше, сырые данные
       /heading_deg     (std_msgs/Float64)          — курс 0..360°, по часовой
                                                        стрелке от СЕВЕРА
                                                        (обычный компасный
                                                        азимут, как bearing
                                                        в robot_commander)
       /imu/data        (sensor_msgs/Imu)           — "виртуальный" IMU только
                                                        с ориентацией по курсу
                                                        (рысканье/yaw), для
                                                        robot_localization EKF.
                                                        yaw = 0 когда робот
                                                        смотрит на север
                                                        (соглашение как в
                                                        примере robot_localization,
                                                        см. navsat_transform.yaml
                                                        -> yaw_offset).

КАЛИБРОВКА (см. также README проекта):
  - mag_offset_x/y (hard-iron): постройте "восьмёрку" — медленно поверните
    робота на 360° на месте, посмотрите min/max по X и Y в /magnetic_field,
    offset = (min+max)/2 по каждой оси.
  - declination_deg: используйте только если НЕ используете
    navsat_transform_node (там есть свой magnetic_declination_radians).
    Если оба применить — курс собьётся вдвое, поэтому по умолчанию 0.0.
  - invert_yaw: включите, если при повороте робота влево /heading_deg
    растёт вместо уменьшения (значит датчик установлен зеркально).
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Imu
from std_msgs.msg import Header, Float64
import smbus


def normalize_deg(angle):
    """Нормализует угол в диапазон [0, 360)."""
    return angle % 360.0


def shortest_angle_diff_deg(target, current):
    """Кратчайшая разница угла (target - current) в диапазоне (-180, 180]."""
    diff = (target - current + 180.0) % 360.0 - 180.0
    return diff


class CompassNode(Node):
    def __init__(self):
        super().__init__('compass')

        # --- Параметры датчика ---
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', 0x0D)
        self.declare_parameter('publish_frequency', 50.0)
        self.declare_parameter('frame_id', 'imu_link')

        # --- Калибровка (hard-iron, см. README) ---
        self.declare_parameter('mag_offset_x', 0.0)   # в единицах результата read_int16 (raw LSB)
        self.declare_parameter('mag_offset_y', 0.0)
        self.declare_parameter('mag_scale_x', 1.0)     # soft-iron (обычно можно оставить 1.0)
        self.declare_parameter('mag_scale_y', 1.0)

        # --- Настройка курса ---
        self.declare_parameter('declination_deg', 0.0)   # см. предупреждение в докстринге
        self.declare_parameter('mounting_offset_deg', 0.0)  # если ось "вперёд" датчика не совпадает с носом робота
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('complementary_alpha', 0.98)  # доверие гироскопу между обновлениями магнитометра
        self.declare_parameter('imu_topic', '/imu')          # источник gyro.z (см. mpu6050_control)
        self.declare_parameter('imu_timeout', 0.5)

        bus_num = self.get_parameter('i2c_bus').value
        addr = self.get_parameter('address').value
        freq = self.get_parameter('publish_frequency').value
        self.frame_id = self.get_parameter('frame_id').value

        self.offset_x = float(self.get_parameter('mag_offset_x').value)
        self.offset_y = float(self.get_parameter('mag_offset_y').value)
        self.scale_x = float(self.get_parameter('mag_scale_x').value)
        self.scale_y = float(self.get_parameter('mag_scale_y').value)

        self.declination_deg = float(self.get_parameter('declination_deg').value)
        self.mounting_offset_deg = float(self.get_parameter('mounting_offset_deg').value)
        self.invert_yaw = bool(self.get_parameter('invert_yaw').value)
        self.alpha = float(self.get_parameter('complementary_alpha').value)
        imu_topic = self.get_parameter('imu_topic').value
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)

        self.bus = None
        try:
            self.get_logger().info(f"Opening I2C bus {bus_num}...")
            self.bus = smbus.SMBus(bus_num)
            self.init_compass(addr)
            self.get_logger().info(f"Compass initialized at 0x{addr:02X}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to init compass: {e}")
            raise RuntimeError("Compass Init Failed")

        self.addr = addr

        # --- Публикации ---
        self.mag_pub = self.create_publisher(MagneticField, 'magnetic_field', 10)
        self.heading_pub = self.create_publisher(Float64, 'heading_deg', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        # --- Гироскоп для комплементарного фильтра ---
        self.gyro_z = 0.0
        self.last_imu_time = None
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        # --- Состояние фильтра курса ---
        self.fused_heading_deg = None   # None пока нет первого валидного отсчёта магнитометра
        self.last_tick_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / freq, self.timer_callback)
        self.get_logger().info("Compass Node running: /magnetic_field, /heading_deg, /imu/data")
        if self.offset_x == 0.0 and self.offset_y == 0.0:
            self.get_logger().warn(
                "mag_offset_x/y = 0.0 — калибровка hard-iron не выполнена. "
                "Курс может быть неточным. См. README (калибровка компаса)."
            )

    def init_compass(self, addr):
        # Control Register 1: Continuous, 200Hz, 2Gauss, 8x Oversample
        self.bus.write_byte_data(addr, 0x09, 0x41)
        # Control Register 2: Reset
        self.bus.write_byte_data(addr, 0x0A, 0x01)
        time.sleep(0.1)

    def read_int16(self, reg):
        try:
            low = self.bus.read_byte_data(self.addr, reg)
            high = self.bus.read_byte_data(self.addr, reg + 1)
            value = (high << 8) | low
            if value > 32767:
                value -= 65536
            return value
        except Exception:
            return None

    def imu_callback(self, msg: Imu):
        self.gyro_z = msg.angular_velocity.z
        self.last_imu_time = self.get_clock().now()

    def timer_callback(self):
        x_raw = self.read_int16(0x01)
        y_raw = self.read_int16(0x03)
        z_raw = self.read_int16(0x05)

        if x_raw is None or y_raw is None or z_raw is None:
            self.get_logger().warn('Compass read failed, skipping this cycle', throttle_duration_sec=5.0)
            return

        SCALE_LSB_PER_GAUSS = 1220.0
        GAUSS_TO_TESLA = 1e-4

        mag_x = (x_raw / SCALE_LSB_PER_GAUSS) * GAUSS_TO_TESLA
        mag_y = (y_raw / SCALE_LSB_PER_GAUSS) * GAUSS_TO_TESLA
        mag_z = (z_raw / SCALE_LSB_PER_GAUSS) * GAUSS_TO_TESLA

        now = self.get_clock().now()

        # --- Публикация сырых данных (как раньше) ---
        mag_msg = MagneticField()
        mag_msg.header = Header()
        mag_msg.header.stamp = now.to_msg()
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z
        self.mag_pub.publish(mag_msg)

        # --- Расчёт курса из магнитометра (tilt-uncompensated: считаем, что робот едет по ровной поверхности) ---
        cal_x = (x_raw - self.offset_x) * self.scale_x
        cal_y = (y_raw - self.offset_y) * self.scale_y

        # Компасный азимут (0=север, растёт по часовой стрелке), стандартная формула для QMC5883L
        heading_mag_rad = math.atan2(cal_y, cal_x)
        heading_mag_deg = normalize_deg(math.degrees(heading_mag_rad))
        heading_mag_deg = normalize_deg(heading_mag_deg + self.mounting_offset_deg + self.declination_deg)
        if self.invert_yaw:
            heading_mag_deg = normalize_deg(-heading_mag_deg)

        # --- Комплементарный фильтр: интегрируем гироскоп между обновлениями магнитометра ---
        dt = (now - self.last_tick_time).nanoseconds / 1e9
        self.last_tick_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / max(self.get_parameter('publish_frequency').value, 1.0)

        gyro_fresh = (
            self.last_imu_time is not None
            and (now - self.last_imu_time).nanoseconds / 1e9 < self.imu_timeout
        )

        if self.fused_heading_deg is None:
            # Первый валидный отсчёт — инициализируем фильтр напрямую магнитометром
            self.fused_heading_deg = heading_mag_deg
        else:
            if gyro_fresh:
                # gyro_z в рад/с, ROS-конвенция (против часовой = +).
                # Компасный азимут растёт ПО часовой, поэтому вычитаем.
                predicted_deg = self.fused_heading_deg - math.degrees(self.gyro_z) * dt
            else:
                predicted_deg = self.fused_heading_deg

            # Комплементарный фильтр в виде векторов, чтобы избежать разрыва на границе 0/360.
            a = self.alpha
            pred_rad = math.radians(predicted_deg)
            meas_rad = math.radians(heading_mag_deg)
            vx = a * math.cos(pred_rad) + (1.0 - a) * math.cos(meas_rad)
            vy = a * math.sin(pred_rad) + (1.0 - a) * math.sin(meas_rad)
            self.fused_heading_deg = normalize_deg(math.degrees(math.atan2(vy, vx)))

        # --- Публикация курса (простой топик, используется robot_commander) ---
        heading_msg = Float64()
        heading_msg.data = self.fused_heading_deg
        self.heading_pub.publish(heading_msg)

        # --- Публикация "виртуального" IMU только с yaw для EKF/navsat_transform ---
        # Соглашение: yaw = 0 когда робот смотрит на север (как ожидает
        # navsat_transform_node с yaw_offset=pi/2, см. project_start/config/navsat_transform.yaml)
        yaw_rad = -math.radians(self.fused_heading_deg)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = self.frame_id
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        # roll/pitch неизвестны (большая дисперсия), yaw — уверенный (комплементарный фильтр)
        imu_msg.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 0.02,
        ]
        imu_msg.angular_velocity.z = self.gyro_z
        imu_msg.angular_velocity_covariance = [
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, 0.001,
        ]
        imu_msg.linear_acceleration_covariance[0] = -1.0  # ускорение не публикуем из этого узла
        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CompassNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
