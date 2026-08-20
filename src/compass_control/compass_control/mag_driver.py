#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Драйвер магнитометра QMC5883L (0x0D) или HMC5883L (0x1E) по I2C.
Поддерживает калибровку hard-iron / soft-iron, поворот осей XY и инверсию Z.
Не зависит от rclpy и MPU6050.
"""

import math
import time

import numpy as np
import smbus2


def rotate_xy(mag, offset_deg):
    """Поворот вектора (X, Y) вокруг оси Z на offset_deg (градусы, против часовой
    в математической системе). Компенсирует разворот осей магнитометра
    относительно осей робота."""
    if not offset_deg:
        return mag
    d = math.radians(offset_deg)
    c, s = math.cos(d), math.sin(d)
    mx, my = mag[0], mag[1]
    mag = mag.copy()
    mag[0] = mx * c - my * s
    mag[1] = mx * s + my * c
    return mag


class Magnetometer:
    """Низкоуровневый доступ к магнитометру QMC5883L или HMC5883L по I2C.

    Параметры:
        bus_num (int): номер шины I2C (обычно 1)
        logger: объект для логирования (опционально)
        mag_yaw_offset_deg (float): поворот осей магнитометра вокруг Z
                                    относительно осей робота (градусы)
        mag_z_invert (bool): инвертировать ось Z (ENU vs NED)
        mag_hard_iron_x/y/z (float): смещение hard-iron в LSB (вычитается)
        mag_scale_x/y/z (float): масштаб soft-iron (умножается после вычитания)
    """

    def __init__(self, bus_num=1, logger=None,
                 mag_yaw_offset_deg=0.0,
                 mag_z_invert=False,
                 mag_hard_iron_x=0.0, mag_hard_iron_y=0.0, mag_hard_iron_z=0.0,
                 mag_scale_x=1.0, mag_scale_y=1.0, mag_scale_z=1.0):
        try:
            self.bus = smbus2.SMBus(bus_num)
        except Exception as e:
            raise RuntimeError(
                f"Не удалось открыть шину I2C-{bus_num}: {e}\n"
                "Проверьте:\n"
                "  1) I2C включён: sudo raspi-config -> Interface Options -> I2C -> Enable\n"
                "  2) пользователь в группе i2c: sudo usermod -aG i2c $USER (перелогиниться)"
            ) from e

        self.logger = logger
        self.mag_yaw_offset_deg = float(mag_yaw_offset_deg or 0.0)
        self.mag_z_invert = bool(mag_z_invert)

        # Калибровка hard-iron (смещение) в LSB
        self.mag_hard_iron = np.array([
            float(mag_hard_iron_x or 0.0),
            float(mag_hard_iron_y or 0.0),
            float(mag_hard_iron_z or 0.0),
        ])
        # Калибровка soft-iron (масштаб)
        self.mag_scale = np.array([
            float(mag_scale_x or 1.0),
            float(mag_scale_y or 1.0),
            float(mag_scale_z or 1.0),
        ])

        self.mag_addr = None
        self.mag_type = None          # 'QMC' | 'HMC' | None
        self._last_mag = np.zeros(3)  # последний удачный замер

        self._init_mag()

    def _init_mag(self):
        """Автоопределение и инициализация магнитометра."""
        # Пробуем QMC5883L (адрес 0x0D)
        try:
            self.bus.read_byte_data(0x0D, 0x00)
            self.mag_addr = 0x0D
            self.mag_type = 'QMC'
            # Control Register 1 (0x09) = 0x1D: OSR=512, RNG=8G, ODR=50Hz
            self.bus.write_byte_data(self.mag_addr, 0x09, 0x1D)
            # Control Register 2 (0x0A) = 0x01: непрерывный режим
            self.bus.write_byte_data(self.mag_addr, 0x0A, 0x01)
            time.sleep(0.01)  # дать время на первый замер
            if self.logger:
                self.logger.info("Магнитометр QMC5883L обнаружен и инициализирован")
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
            if self.logger:
                self.logger.info("Магнитометр HMC5883L обнаружен и инициализирован")
            return
        except Exception:
            pass

        if self.logger:
            self.logger.warning(
                "Магнитометр не найден (QMC5883L/HMC5883L). "
                "Чтение будет возвращать нулевой вектор."
            )

    def read_word_2c(self, reg, little_endian=False):
        """Чтение 16-битного значения в дополнительном коде (two's complement).
           Предполагается, что self.mag_addr уже установлен."""
        if self.mag_addr is None:
            return 0
        try:
            if little_endian:
                low = self.bus.read_byte_data(self.mag_addr, reg)
                high = self.bus.read_byte_data(self.mag_addr, reg + 1)
            else:
                high = self.bus.read_byte_data(self.mag_addr, reg)
                low = self.bus.read_byte_data(self.mag_addr, reg + 1)
            val = (high << 8) + low
            return -((65535 - val) + 1) if val >= 0x8000 else val
        except Exception:
            # Ошибка I2C — возвращаем 0
            return 0

    def read_mag(self):
        """Возвращает калиброванный вектор магнитометра (в сырых LSB) с учётом
        hard-iron, soft-iron, поворота XY и инверсии Z.
        При ошибке чтения возвращает последний удачный вектор."""
        if self.mag_type is None:
            return self._last_mag.copy()

        mag = np.array([0.0, 0.0, 0.0])

        if self.mag_type == 'QMC':
            # QMC5883L: младший байт по младшему адресу (little-endian)
            mx = self.read_word_2c(0x00, True)
            my = self.read_word_2c(0x02, True)
            mz = self.read_word_2c(0x04, True)
            mag = np.array([mx, my, mz], dtype=float)
        elif self.mag_type == 'HMC':
            # HMC5883L: старший байт по младшему адресу (big-endian)
            # регистры: 0x03 = X MSB, 0x05 = Z MSB, 0x07 = Y MSB
            mx = self.read_word_2c(0x03)
            mz = self.read_word_2c(0x05)
            my = self.read_word_2c(0x07)
            mag = np.array([mx, my, mz], dtype=float)

        # Если чтение вернуло все нули (ошибка) — подставляем последний удачный замер
        if np.all(mag == 0.0):
            mag = self._last_mag.copy()
        else:
            self._last_mag = mag.copy()

        # Калибровка hard-iron и soft-iron
        if np.any(self.mag_hard_iron):
            mag = mag - self.mag_hard_iron
        if np.any(self.mag_scale != 1.0):
            mag = mag * self.mag_scale

        # Поворот осей XY
        mag = rotate_xy(mag, self.mag_yaw_offset_deg)

        # Инверсия оси Z для приведения к ENU (если требуется)
        if self.mag_z_invert:
            mag[2] = -mag[2]

        return mag

    def close(self):
        """Закрывает шину I2C (опционально)."""
        try:
            self.bus.close()
        except Exception:
            pass