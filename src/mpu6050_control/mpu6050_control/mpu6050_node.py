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

        # --- Параметры восстановления связи -------------------------------
        # РАНЬШЕ: при любой ошибке инициализации узел вызывал sys.exit(1)
        # и умирал НАВСЕГДА. Достаточно было одного сбоя на шине I2C в
        # момент запуска (а он случается: длинные провода, помеха от
        # моторов, устройство ещё не поднялось после подачи питания) —
        # и весь стек продолжал работать БЕЗ ДАТЧИКА КУРСА. Внешне всё
        # выглядело нормально: остальные узлы живы, TF публикуется,
        # и только /imu/data_raw молчал.
        #
        # ТЕПЕРЬ: узел остаётся жив и повторяет попытки подключения.
        # Он же переподключается, если датчик отвалился уже на ходу.
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('max_read_errors', 25)
        # Адрес MPU6050 на шине. 0 = искать автоматически среди
        # 0x68 и 0x69 (адрес задаётся ногой AD0 на плате датчика).
        self.declare_parameter('i2c_address', 0)

        self._reconnect_interval = float(
            self.get_parameter('reconnect_interval').value)
        self._max_read_errors = int(
            self.get_parameter('max_read_errors').value)

        self._bus_num = bus_num
        addr = int(self.get_parameter('i2c_address').value)
        self._mpu_addr = addr if addr > 0 else None
        self._acc_invert = acc_invert
        self._mount = (mount_roll, mount_pitch, mount_yaw)
        self._calibrate_on_start = calibrate_on_start
        self._calibrate_samples = calibrate_samples
        self._calibrate_interval = calibrate_interval

        self.imu = None
        self._read_errors = 0
        self._init_attempts = 0

        # Публикация в топик /imu/data
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        # Таймер для публикации с заданной частотой
        timer_period = 1.0 / rate_hz if rate_hz > 0 else 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Первая попытка — сразу, дальше по таймеру переподключения
        self._try_connect()
        if self.imu is None:
            self._reconnect_timer = self.create_timer(
                self._reconnect_interval, self._try_connect)
        else:
            self._reconnect_timer = None

        self.get_logger().info(f'Нода MPU6050 запущена, частота публикации {rate_hz} Гц')

    def _try_connect(self):
        """Подключается к датчику. Вызывается повторно, пока не получится."""
        if self.imu is not None:
            return

        self._init_attempts += 1
        try:
            imu = HardwareIMU(
                bus_num=self._bus_num,
                logger=self.get_logger(),
                acc_invert=self._acc_invert,
                imu_mount_roll_deg=self._mount[0],
                imu_mount_pitch_deg=self._mount[1],
                imu_mount_yaw_deg=self._mount[2],
                mpu_addr=self._mpu_addr,
            )
        except Exception as e:
            # Первые сообщения подробные, дальше — редкие, чтобы не забить лог
            if self._init_attempts <= 3 or self._init_attempts % 12 == 0:
                self.get_logger().error(
                    f'Не удалось подключиться к MPU6050 '
                    f'(попытка {self._init_attempts}): {e}')
                self.get_logger().error(
                    'Проверьте: sudo i2cdetect -y 1 — должен быть адрес 0x68. '
                    'Если адреса нет, дело в питании или проводах датчика. '
                    f'Повторю через {self._reconnect_interval:.0f} с.')
            return

        self.imu = imu
        self._read_errors = 0

        # Калибровка нуля гироскопа. Робот в это время должен стоять.
        if self._calibrate_on_start:
            try:
                self.imu.calibrate(
                    samples=self._calibrate_samples,
                    interval=self._calibrate_interval,
                    logger=self.get_logger(),
                )
            except Exception as e:
                self.get_logger().warning(
                    f'Калибровка гироскопа не удалась: {e}. '
                    f'Работаем без неё — курс будет медленно уплывать.')

        if self._init_attempts > 1:
            self.get_logger().info(
                f'Связь с MPU6050 восстановлена '
                f'(с попытки {self._init_attempts}).')

        # Больше переподключаться не нужно
        if self._reconnect_timer is not None:
            self._reconnect_timer.cancel()
            self._reconnect_timer = None

    def _handle_read_error(self, exc):
        """Считает ошибки чтения и инициирует переподключение."""
        self._read_errors += 1

        if self._read_errors == 1:
            self.get_logger().error(f'Ошибка чтения IMU: {exc}')

        if self._read_errors >= self._max_read_errors:
            self.get_logger().error(
                f'{self._read_errors} ошибок чтения подряд — '
                f'датчик считается отвалившимся, переподключаюсь.')
            try:
                if hasattr(self.imu, 'close'):
                    self.imu.close()
            except Exception:
                pass
            self.imu = None
            self._read_errors = 0
            self._init_attempts = 0
            if self._reconnect_timer is None:
                self._reconnect_timer = self.create_timer(
                    self._reconnect_interval, self._try_connect)

        # Сохраняем ковариации (диагональные, остальные нули)
        ang_cov = self.get_parameter('angular_velocity_covariance').value
        lin_cov = self.get_parameter('linear_acceleration_covariance').value
        self.angular_cov = np.diag([ang_cov, ang_cov, ang_cov]).flatten().tolist()
        self.linear_cov = np.diag([lin_cov, lin_cov, lin_cov]).flatten().tolist()

    def timer_callback(self):
        # Связи ещё нет — молча ждём, о попытках сообщает _try_connect
        if self.imu is None:
            return

        try:
            acc, gyro = self.imu.get_data()
        except Exception as e:
            self._handle_read_error(e)
            return

        # Успешное чтение обнуляет счётчик: одиночные сбои шины не должны
        # накапливаться и приводить к ложному переподключению
        self._read_errors = 0

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