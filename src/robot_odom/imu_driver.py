#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Драйвер IMU: MPU6050 (акселерометр + гироскоп) и опционально магнитометр
QMC5883L (0x0D) / HMC5883L (0x1E) по I2C.

Модуль не зависит от rclpy — его можно использовать и вне ROS 2
(например, из диагностического скрипта imu_check).
"""

import math
import time

import numpy as np
import smbus2


# Чувствительности MPU6050 при заводских настройках (регистры 0x1B/0x1C не менялись)
MPU_ACCEL_SCALE = 16384.0   # LSB/g  при диапазоне ±2g
MPU_GYRO_SCALE = 131.0      # LSB/(°/s) при диапазоне ±250 °/s
GRAVITY = 9.81              # м/с²


def rotate_xy(mag, offset_deg):
    """Поворот вектора (X, Y) вокруг оси Z на offset_deg (градусы, против часовой
    в математической системе). Компенсирует разворот осей магнитометра
    относительно осей робота, когда компас — отдельное устройство."""
    if not offset_deg:
        return mag
    d = math.radians(offset_deg)
    c, s = math.cos(d), math.sin(d)
    mx, my = mag[0], mag[1]
    mag = mag.copy()
    mag[0] = mx * c - my * s
    mag[1] = mx * s + my * c
    return mag


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
    """Низкоуровневый доступ к MPU6050 и магнитометру по I2C.

    mag_yaw_offset_deg — поворот осей магнитометра вокруг Z относительно осей
    MPU6050/робота (градусы). Измеряется утилитой imu_check --heading.

    imu_mount_roll_deg / imu_mount_pitch_deg / imu_mount_yaw_deg — монтажный
    наклон платы IMU относительно base_link (градусы, конвенция RPY как в URDF).
    Компенсируется поворотом acc/gyro/mag в систему base_link ДО EKF.
    Измеряются утилитой imu_check --calibrate-mount (робот стоит ровно).
    """

    def __init__(self, bus_num=1, logger=None, mag_yaw_offset_deg=0.0,
                 mag_z_invert=False, acc_invert=False,
                 imu_mount_roll_deg=0.0, imu_mount_pitch_deg=0.0,
                 imu_mount_yaw_deg=0.0,
                 mag_hard_iron_x=0.0, mag_hard_iron_y=0.0, mag_hard_iron_z=0.0,
                 mag_scale_x=1.0, mag_scale_y=1.0, mag_scale_z=1.0):
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
        self.mag_yaw_offset_deg = float(mag_yaw_offset_deg or 0.0)
        self.mag_z_invert = bool(mag_z_invert)
        # Компенсация постоянного магнитного фона (hard-iron) в LSB.
        # Вычитается из показаний магнитометра. Измеряется утилитой
        # imu_check --calibrate-mag (вращение робота на 360°).
        self.mag_hard_iron = np.array([
            float(mag_hard_iron_x or 0.0),
            float(mag_hard_iron_y or 0.0),
            float(mag_hard_iron_z or 0.0),
        ])
        # Масштаб осей (soft-iron): mag_cal = (mag - hard_iron) * scale.
        # Измеряется той же утилитой (отношение размахов).
        self.mag_scale = np.array([
            float(mag_scale_x or 1.0),
            float(mag_scale_y or 1.0),
            float(mag_scale_z or 1.0),
        ])
        # ahrs EKF в конвенции ENU ожидает, что акселерометр при плашмя выдаёт
        # ВЕКТОР ГРАВИТАЦИИ [0,0,-9.81] («вниз»), а MPU6050 физически выдаёт
        # РЕАКЦИЮ ОПОРЫ [0,0,+9.81] («вверх»). Без согласования знака фильтр
        # переворачивает ориентацию (ось Z вниз в RViz), как только включается
        # магнитометр. acc_invert=true инвертирует acc перед EKF.
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
        self.mag_addr = None
        self.mag_type = None      # 'QMC' | 'HMC' | None

        # Результаты автокалибровки (вычитаются/прибавляются в get_data)
        self.gyro_bias = np.zeros(3)   # рад/с
        self.acc_bias = np.zeros(3)    # м/с²
        self.calibrated = False
        self._last_mag = np.zeros(3)   # последний удачный замер магнитометра

        self._init_mpu()
        self._init_mag()

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
            #    Драйвер работает с ±2g (16384 LSB/g). Если FS != ±2g —
            #    показания завышены в 2/4/8 раз (|g| >> 9.81 при ровном роботе).
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

    def _init_mag(self):
        # Пробуем QMC5883L (адрес 0x0D)
        try:
            self.bus.read_byte_data(0x0D, 0x00)
            self.mag_addr = 0x0D
            self.mag_type = 'QMC'
            # Control Register 1 (0x09) = 0x1D: continuous mode, 50 Гц, ±8G, OSR=512
            self.bus.write_byte_data(self.mag_addr, 0x09, 0x1D)
            return
        except Exception:
            pass
        # Пробуем HMC5883L (адрес 0x1E)
        try:
            self.bus.read_byte_data(0x1E, 0x03)
            self.mag_addr = 0x1E
            self.mag_type = 'HMC'
            # Config Register B (0x02) = 0x00: gain 1370 LSB/Gauss, ±1.3Ga
            self.bus.write_byte_data(self.mag_addr, 0x02, 0x00)
            return
        except Exception:
            pass
        if self.logger:
            self.logger.warning(
                "Магнитометр не найден (QMC5883L/HMC5883L). "
                "Курс будет дрейфовать — недоступна магнитная коррекция по рысканию."
            )

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
        """Сырые значения MPU6050 (без магнитометра): (acc_raw, gyro_raw)."""
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
        # Модуль |g| может отличаться от 9.81 из-за ошибки масштаба датчика
        # или вибрации. Для НАПРАВЛЕНИЯ (уровень) это не важно — EKF
        # нормализует вектор. Главное — малый разброс (неподвижен) и
        # правдоподобный модуль. Если разброс мал — смещения X/Y вычитаются,
        # и кажущийся наклон от некалиброванных смещений уходит.
        # Если задан монтажный поворот (imu_mount_*_deg) — выравнивание по
        # уровню делает ОН, а bias-форсирование уровня отключаем (иначе
        # двойная компенсация одного и того же).
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
        """Возвращает кортеж (acc в м/с², gyro в рад/с, mag в сырых LSB),
        с вычтенными смещениями из автокалибровки."""
        a_raw, g_raw = self._read_mpu_raw()
        acc = a_raw / MPU_ACCEL_SCALE * GRAVITY + self.acc_bias
        gyro = g_raw / MPU_GYRO_SCALE * (math.pi / 180.0) - self.gyro_bias

        # Компенсация монтажного наклона платы: приводим показания к осям base_link
        if self.mount_rot is not None:
            acc = self.mount_rot @ acc
            gyro = self.mount_rot @ gyro

        if self.acc_invert:
            acc = -acc

        mag = np.array([0.0, 0.0, 0.0])
        if self.mag_type == 'QMC':
            # QMC5883L: младший байт по младшему адресу (little-endian)
            mx = self.read_word_2c(self.mag_addr, 0x00, True)
            my = self.read_word_2c(self.mag_addr, 0x02, True)
            mz = self.read_word_2c(self.mag_addr, 0x04, True)
            mag = np.array([mx, my, mz], dtype=float)
        elif self.mag_type == 'HMC':
            # HMC5883L: старший байт по младшему адресу (big-endian).
            # Карта регистров: 0x03 = X MSB, 0x05 = Z MSB, 0x07 = Y MSB.
            mx = self.read_word_2c(self.mag_addr, 0x03)
            mz = self.read_word_2c(self.mag_addr, 0x05)
            my = self.read_word_2c(self.mag_addr, 0x07)
            mag = np.array([mx, my, mz], dtype=float)

        # Калибровка магнитометра: вычитание hard-iron + масштаб soft-iron
        # (см. imu_check --calibrate-mag)
        if np.any(self.mag_hard_iron):
            mag = mag - self.mag_hard_iron
        if np.any(self.mag_scale != 1.0):
            mag = mag * self.mag_scale
        # Сбой I2C: read_word_2c молча вернул 0 — подставляем последний
        # удачный замер (иначе нули рвут курс и портят калибровку).
        if np.all(mag == 0.0):
            mag = self._last_mag.copy()
        else:
            self._last_mag = mag.copy()

        # Компас — отдельное устройство со своими осями (например, в корпусе
        # GPS). Приводим его оси к осям MPU6050/робота, чтобы EKF-слияние
        # и публикуемый курс были корректными.
        mag = rotate_xy(mag, self.mag_yaw_offset_deg)
        # QMC5883L выдаёт mz положительную «вниз» (стиль NED), а ENU-модель
        # ahrs ожидает mz «вверх» (отрицательную при поле вниз).
        # Несогласованный знак Z переворачивает ориентацию EKF (Z вниз в RViz).
        if self.mag_z_invert:
            mag[2] = -mag[2]
        # Тот же монтажный поворот — для магнитометра (он жёстко на корпусе)
        if self.mount_rot is not None:
            mag = self.mount_rot @ mag

        return acc, gyro, mag
