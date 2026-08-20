#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header
import numpy as np
import sys
import os

# Импорт драйвера (предполагается, что файл imu_driver.py лежит в том же каталоге)
try:
    from imu_driver import HardwareIMU
except ImportError:
    # Если запускается как пакет, можно импортировать из модуля
    from .imu_driver import HardwareIMU


class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_control')

        # Параметры
        self.declare_parameter('bus_num', 1)
        self.declare_parameter('imu_mount_roll_deg', 0.0)
        self.declare_parameter('imu_mount_pitch_deg', 0.0)
        self.declare_parameter('imu_mount_yaw_deg', 0.0)
        self.declare_parameter('acc_invert', False)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('calibrate_on_start', True)
        self.declare_parameter('calibrate_samples', 100)
        self.declare_parameter('calibrate_interval', 0.01)

        # Ковариации (диагональные элементы)
        self.declare_parameter('angular_velocity_covariance', 1e-3)
        self.declare_parameter('linear_acceleration_covariance', 1e-3)

        bus_num = self.get_parameter('bus_num').value
        mount_roll = self.get_parameter('imu_mount_roll_deg').value
        mount_pitch = self.get_parameter('imu_mount_pitch_deg').value
        mount_yaw = self.get_parameter('imu_mount_yaw_deg').value
        acc_invert = self.get_parameter('acc_invert').value
        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = self.get_parameter('publish_rate_hz').value
        calibrate_on_start = self.get_parameter('calibrate_on_start').value
        calibrate_samples = self.get_parameter('calibrate_samples').value
        calibrate_interval = self.get_parameter('calibrate_interval').value

        # Создаём драйвер
        try:
            self.imu = HardwareIMU(
                bus_num=bus_num,
                logger=self.get_logger(),
                acc_invert=acc_invert,
                imu_mount_roll_deg=mount_roll,
                imu_mount_pitch_deg=mount_pitch,
                imu_mount_yaw_deg=mount_yaw
            )
        except Exception as e:
            self.get_logger().error(f'Ошибка инициализации драйвера: {e}')
            rclpy.shutdown()
            sys.exit(1)

        # Автокалибровка
        if calibrate_on_start:
            self.imu.calibrate(
                samples=calibrate_samples,
                interval=calibrate_interval,
                logger=self.get_logger()
            )

        # Публикация в топик /imu/data
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        # Таймер для публикации с заданной частотой
        timer_period = 1.0 / rate_hz if rate_hz > 0 else 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'Нода MPU6050 запущена, частота публикации {rate_hz} Гц')

        # Сохраняем ковариации (диагональные, остальные нули)
        ang_cov = self.get_parameter('angular_velocity_covariance').value
        lin_cov = self.get_parameter('linear_acceleration_covariance').value
        self.angular_cov = np.diag([ang_cov, ang_cov, ang_cov]).flatten().tolist()
        self.linear_cov = np.diag([lin_cov, lin_cov, lin_cov]).flatten().tolist()

    def timer_callback(self):
        try:
            acc, gyro = self.imu.get_data()
        except Exception as e:
            self.get_logger().error(f'Ошибка чтения IMU: {e}')
            return

        # Формируем сообщение Imu
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Ориентация: пока оставляем неопределённой (можно будет добавить фильтр позже)
        # Устанавливаем ковариацию ориентации как -1 (означает "неизвестно")
        msg.orientation_covariance[0] = -1.0

        # Угловая скорость
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.angular_velocity_covariance = self.angular_cov

        # Линейное ускорение
        msg.linear_acceleration.x = float(acc[0])
        msg.linear_acceleration.y = float(acc[1])
        msg.linear_acceleration.z = float(acc[2])
        msg.linear_acceleration_covariance = self.linear_cov

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()