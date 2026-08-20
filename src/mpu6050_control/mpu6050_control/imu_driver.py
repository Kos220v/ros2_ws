#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Драйвер IMU: MPU6050 (акселерометр + гироскоп) по I2C.
Модуль не зависит от rclpy — его можно использовать и вне ROS 2.
"""

import math
import time

import numpy as np
import smbus2

# Чувствительности MPU6050 при заводских настройках (регистры 0x1B/0x1C не менялись)
MPU_ACCEL_SCALE = 16384.0   # LSB/g  при диапазоне ±2g
MPU_GYRO_SCALE = 131.0      # LSB/(°/s) при диапазоне ±250 °/s
GRAVITY = 9.81              # м/с²


def rot_from_rpy(roll, pitch, yaw):
    """Матрица поворота R из углов RPY (рад), конвенция как в URDF:
    v_base = R @ v_sensor."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]])
    Rz = np.array([[cy, -sy, 0.0],
                   [sy, cy, 0.0],
                   [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx


class HardwareIMU:
    """Низкоуровневый доступ к MPU6050 по I2C.

    imu_mount_roll_deg / imu_mount_pitch_deg / imu_mount_yaw_deg — монтажный
    наклон платы IMU относительно base_link (градусы, конвенция RPY как в URDF).
    Компенсируется поворотом acc/gyro в систему base_link ДО EKF.
    acc_invert — инвертирует акселерометр (если датчик выдаёт реакцию опоры, а не гравитацию).
    """

    def __init__(self, bus_num=1, logger=None,
                 acc_invert=False,
                 imu_mount_roll_deg=0.0, imu_mount_pitch_deg=0.0,
                 imu_mount_yaw_deg=0.0):
        try:
            self.bus = smbus2.SMBus(bus_num)
        except Exception as e:
            raise RuntimeError(
                f"Не удалось открыть шину I2C-{bus_num}: {e}\n"
                "Проверьте:\n"
                "  1) I2C включён: sudo raspi-config -> Interface Options -> I2C -> Enable\n"
                "  2) пользователь в группе i2c: sudo usermod -aG i2c $USER (перелогиниться)\n"
                "  3) устройство на шине: i2cdetect -y 1 (должен быть адрес 0x68)"
            ) from e
        self.logger = logger
        self.acc_invert = bool(acc_invert)
        # Монтажный наклон платы IMU относительно base_link.
        if any((imu_mount_roll_deg, imu_mount_pitch_deg, imu_mount_yaw_deg)):
            self.mount_rot = rot_from_rpy(
                math.radians(float(imu_mount_roll_deg or 0.0)),
                math.radians(float(imu_mount_pitch_deg or 0.0)),
                math.radians(float(imu_mount_yaw_deg or 0.0)),
            )
        else:
            self.mount_rot = None
        self.mpu_addr = 0x68

        # Результаты автокалибровки (вычитаются/прибавляются в get_data)
        self.gyro_bias = np.zeros(3)   # рад/с
        self.acc_bias = np.zeros(3)    # м/с²
        self.calibrated = False

        self._init_mpu()

    # --- инициализация -----------------------------------------------------

    def _init_mpu(self):
        """Пробуждение MPU6050 с ретраями (шина I2C бывает занята при старте)
        и проверкой WHO_AM_I. При устойчивом отказе — RuntimeError (fail-fast)."""
        try:
            # 1) Полный сброс устройства: бит DEVICE_RESET (0x80) в PWR_MGMT_1
            self.bus.write_byte_data(self.mpu_addr, 0x6B, 0x80)
            time.sleep(0.15)
            # 2) Вывод из sleep с ретраями (Errno 5 на старте — обычное дело)
            last_err = None
            for attempt in range(5):
                try:
                    self.bus.write_byte_data(self.mpu_addr, 0x6B, 0x00)
                    break
                except Exception as e:
                    last_err = e
                    time.sleep(0.1)
            else:
                raise RuntimeError(
                    f"MPU6050 не отвечает после 5 попыток пробуждения: {last_err}")
            time.sleep(0.1)
            # 3) Проверка WHO_AM_I (0x75): MPU6050 отвечает 0x68
            try:
                who = self.bus.read_byte_data(self.mpu_addr, 0x75)
                if self.logger:
                    self.logger.info(f"MPU6050: WHO_AM_I = 0x{who:02X}")
            except Exception as e:
                if self.logger:
                    self.logger.warning(f"Не удалось прочитать WHO_AM_I: {e}")
            # 4) Какой диапазон акселерометра стоит в ACCEL_CONFIG (0x1C)?
            try:
                fs = self.bus.read_byte_data(self.mpu_addr, 0x1C)
                fsr = {0x00: '±2g (16384 LSB/g)',
                       0x08: '±4g (8192 LSB/g)',
                       0x10: '±8g (4096 LSB/g)',
                       0x18: '±16g (2048 LSB/g)'}.get(
                           fs & 0x18, f'неизвестно (0x{fs:02X})')
                if self.logger:
                    self.logger.info(f"MPU6050: ACCEL_CONFIG = {fsr}")
                if (fs & 0x18) != 0x00:
                    if self.logger:
                        self.logger.error(
                            "Диапазон акселерометра НЕ ±2g! Показания будут "
                            "завышены. Сброс вернул регистр к ±2g — проверьте, "
                            "что его не меняет другой процесс/библиотека."
                        )
            except Exception as e:
                if self.logger:
                    self.logger.debug(f"ACCEL_CONFIG не прочитан: {e}")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Ошибка инициализации MPU6050: {e}")
            raise

    # --- чтение ------------------------------------------------------------

    def read_word_2c(self, addr, reg, little_endian=False):
        """Чтение 16-битного значения в дополнительном коде (two's complement)."""
        try:
            if little_endian:
                low = self.bus.read_byte_data(addr, reg)
                high = self.bus.read_byte_data(addr, reg + 1)
            else:
                high = self.bus.read_byte_data(addr, reg)
                low = self.bus.read_byte_data(addr, reg + 1)
            val = (high << 8) + low
            return -((65535 - val) + 1) if val >= 0x8000 else val
        except Exception:
            # Ошибка I2C: возвращаем 0, чтобы не ронять цикл (следите по логам)
            return 0

    def _read_mpu_raw(self):
        """Сырые значения MPU6050: (acc_raw, gyro_raw)."""
        ax_raw = self.read_word_2c(self.mpu_addr, 0x3B)
        ay_raw = self.read_word_2c(self.mpu_addr, 0x3D)
        az_raw = self.read_word_2c(self.mpu_addr, 0x3F)
        gx_raw = self.read_word_2c(self.mpu_addr, 0x43)
        gy_raw = self.read_word_2c(self.mpu_addr, 0x45)
        gz_raw = self.read_word_2c(self.mpu_addr, 0x47)
        return np.array([ax_raw, ay_raw, az_raw]), np.array([gx_raw, gy_raw, gz_raw])

    def read_raw_acc(self):
        """Сырое ускорение (м/с²) БЕЗ bias-коррекции и монтажного поворота.
        Нужно для диагностики монтажного наклона (imu_check --calibrate-mount)."""
        a_raw, _ = self._read_mpu_raw()
        return a_raw / MPU_ACCEL_SCALE * GRAVITY

    def calibrate(self, samples=100, interval=0.01, logger=None):
        """
        Автокалибровка смещений при старте.

        Робот должен лежать НЕПОДВИЖНО плашмя (z-вверх) во время калибровки.
        Гироскоп калибруется всегда (bias = среднее в покое);
        акселерометр — только если лежит ровно (проверка по модулю |g| и разбросу).
        """
        self.logger = logger or self.logger
        if self.logger:
            self.logger.info(
                f"Автокалибровка: собираю {samples} сэмплов. "
                "Робот должен лежать неподвижно плашмя..."
            )
        gyros, accs = [], []
        for _ in range(samples):
            a_raw, g_raw = self._read_mpu_raw()
            gyros.append(g_raw / MPU_GYRO_SCALE * (math.pi / 180.0))
            accs.append(a_raw / MPU_ACCEL_SCALE * GRAVITY)
            time.sleep(interval)
        gyros = np.array(gyros)
        accs = np.array(accs)

        # --- гироскоп: bias = среднее в покое ------------------------------
        self.gyro_bias = gyros.mean(axis=0)
        g_std = gyros.std(axis=0)
        if self.logger:
            self.logger.info(
                f"Смещение гироскопа (рад/с): {np.round(self.gyro_bias, 5)}"
            )
        if g_std.max() > 0.3:   # ~17 °/с — явно двигался
            if self.logger:
                self.logger.warning(
                    "Высокий разброс гироскопа во время калибровки — "
                    "возможно, робот двигался. Рекомендую повторить калибровку."
                )

        # --- акселерометр: калибруем, если робот неподвижен -----------------
        if self.mount_rot is not None:
            self.acc_bias = np.zeros(3)
            if self.logger:
                self.logger.info(
                    "Уровень компенсируется монтажным поворотом "
                    "(imu_mount_*_deg) — acc_bias по уровню не применяется."
                )
        else:
            g_mag = np.linalg.norm(accs, axis=1)
            g_mean = float(g_mag.mean())
            g_std = float(g_mag.std())
            flat = 8.0 < g_mean < 11.5 and g_std < 0.25
            if flat:
                # Вычитаем смещения по всем осям: при ровном роботе после этого
                # acc = (0, 0, +9.81) — кажущийся наклон от X/Y-смещений исчезает.
                self.acc_bias = np.array([0.0, 0.0, GRAVITY]) - accs.mean(axis=0)
                if self.logger:
                    self.logger.info(
                        f"Смещение акселерометра (м/с²): {np.round(self.acc_bias, 4)}"
                    )
                if abs(g_mean - GRAVITY) > 0.5:
                    if self.logger:
                        self.logger.warning(
                            f"|g| = {g_mean:.2f} м/с² (ожидалось ~9.81): возможна "
                            "ошибка масштаба датчика или вибрация во время "
                            "калибровки. Смещения X/Y вычтены, уровень корректен."
                        )
            else:
                self.acc_bias = np.zeros(3)
                reason = (f"разброс {g_std:.2f} м/с² (робот двигался/вибрировал)"
                          if g_std >= 0.25 else
                          f"|g| = {g_mean:.2f} м/с² вне диапазона 8..11.5")
                if self.logger:
                    self.logger.warning(
                        "Акселерометр не откалиброван: " + reason +
                        ". Проверьте, что робот лежит неподвижно плашмя на ровной "
                        "поверхности (и выключите вибрации: моторы/лидар)."
                    )

        self.calibrated = True
        if self.logger:
            self.logger.info("Автокалибровка завершена.")

    def get_data(self):
        """Возвращает кортеж (acc в м/с², gyro в рад/с),
        с вычтенными смещениями из автокалибровки и с учётом монтажного поворота."""
        a_raw, g_raw = self._read_mpu_raw()
        acc = a_raw / MPU_ACCEL_SCALE * GRAVITY + self.acc_bias
        gyro = g_raw / MPU_GYRO_SCALE * (math.pi / 180.0) - self.gyro_bias

        # Компенсация монтажного наклона платы: приводим показания к осям base_link
        if self.mount_rot is not None:
            acc = self.mount_rot @ acc
            gyro = self.mount_rot @ gyro

        if self.acc_invert:
            acc = -acc

        return acc, gyro