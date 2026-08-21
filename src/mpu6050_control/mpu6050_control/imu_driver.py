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
                 imu_mount_yaw_deg=0.0,
                 mpu_addr=None):
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
        # Адрес на шине. У MPU6050 их два, и выбирается он НОГОЙ AD0:
        #   AD0 = GND (или не подключён) -> 0x68
        #   AD0 = VCC                    -> 0x69
        # Раньше здесь было жёстко 0x68, и плата с подтянутым AD0 просто
        # не находилась. Теперь при mpu_addr=None пробуются оба адреса.
        self.mpu_addr = None
        self._addr_candidates = [0x68, 0x69] if mpu_addr is None \
            else [int(mpu_addr)]

        # Результаты автокалибровки (вычитаются/прибавляются в get_data)
        self.gyro_bias = np.zeros(3)   # рад/с
        self.acc_bias = np.zeros(3)    # м/с²
        self.calibrated = False

        self._init_mpu()

    # --- инициализация -----------------------------------------------------

    def scan_bus(self):
        """Возвращает список адресов, которые отвечают на шине.

        Аналог `i2cdetect -y 1`, но доступный прямо из кода. Нужен для
        внятного сообщения об ошибке: одно дело "датчик не отвечает",
        и совсем другое — "на шине пусто" или "датчик отвечает, но по
        соседнему адресу".
        """
        found = []
        for addr in range(0x03, 0x78):
            try:
                # Чтение байта — самый безобидный способ проверить отклик.
                self.bus.read_byte(addr)
                found.append(addr)
            except Exception:
                pass
        return found

    def _find_mpu(self):
        """Ищет MPU6050 среди возможных адресов по регистру WHO_AM_I."""
        # WHO_AM_I (0x75) у разных клонов отличается:
        #   0x68 — оригинальный MPU6050
        #   0x69 — некоторые копии
        #   0x70 — MPU6500 / MPU9250
        #   0x71, 0x73 — прочие клоны
        known = {0x68, 0x69, 0x70, 0x71, 0x73, 0x98}

        last_err = None
        for addr in self._addr_candidates:
            # Читаем WHO_AM_I с повторами. Одиночный отказ на шине ничего
            # не доказывает: помеха от моторов, длинные провода или просто
            # коллизия с другим обращением к шине дают Errno 5 / Errno 121
            # на совершенно исправном датчике.
            who = None
            for _ in range(5):
                try:
                    who = self.bus.read_byte_data(addr, 0x75)
                    break
                except Exception as e:
                    last_err = e
                    time.sleep(0.05)

            if who is None:
                continue

            if who in known:
                if self.logger:
                    self.logger.info(
                        f'MPU6050 найден по адресу 0x{addr:02X} '
                        f'(WHO_AM_I = 0x{who:02X})')
                return addr

            if self.logger:
                self.logger.warning(
                    f'По адресу 0x{addr:02X} кто-то отвечает, но WHO_AM_I = '
                    f'0x{who:02X} — это не похоже на MPU6050.')

        # Не нашли: собираем внятную картину шины для сообщения об ошибке.
        present = self.scan_bus()
        lines = [f'MPU6050 не отвечает ни по одному из адресов: '
                 f'{", ".join(f"0x{a:02X}" for a in self._addr_candidates)}.']

        if not present:
            lines.append('')
            lines.append('На шине I2C НЕТ НИ ОДНОГО устройства.')
            lines.append('Значит, дело не в самом датчике, а в шине целиком:')
            lines.append('  * I2C выключен: sudo raspi-config -> Interface '
                         'Options -> I2C -> Enable')
            lines.append('  * не подключены SDA/SCL или нет подтяжки')
        else:
            lines.append('')
            lines.append('Отвечают адреса: '
                         + ', '.join(f'0x{a:02X}' for a in present))
            hints = {0x0D: 'магнитометр QMC5883L',
                     0x1E: 'магнитометр HMC5883L',
                     0x77: 'барометр BMP180/BMP280',
                     0x76: 'барометр BMP280',
                     0x68: 'MPU6050 (AD0 на землю)',
                     0x69: 'MPU6050 (AD0 на питание)'}
            for a in present:
                if a in hints:
                    lines.append(f'    0x{a:02X} — {hints[a]}')
            lines.append('')
            lines.append('Шина ЖИВА (раз другие устройства отвечают), '
                         'проблема только в MPU6050:')
            lines.append('  * проверьте питание VCC и землю именно этого '
                         'модуля')
            lines.append('  * проверьте пайку SDA/SCL на его плате')
            lines.append('  * если модуль совмещённый (GY-87 и подобные), '
                         'MPU мог выйти из строя отдельно от магнитометра')

        if last_err is not None:
            lines.append('')
            lines.append(f'Последняя ошибка шины: {last_err}')

        raise RuntimeError('\n'.join(lines))

    def _init_mpu(self):
        """Находит датчик, будит его и проверяет настройки.

        Шина I2C при старте бывает занята, поэтому пробуждение делается
        с повторами. Если датчик так и не отозвался — RuntimeError с
        разбором того, что вообще есть на шине.
        """
        # Сначала определяем адрес: он зависит от ноги AD0 на плате.
        self.mpu_addr = self._find_mpu()

        try:
            # 1) Полный сброс устройства: бит DEVICE_RESET (0x80) в PWR_MGMT_1
            #
            # ВНИМАНИЕ при отладке: во время сброса (около 150 мс) датчик не
            # отвечает НИКОМУ. Если параллельно запустить утилиту проверки
            # железа, она попадёт в это окно и объявит исправный датчик
            # неисправным. Поэтому i2c_check просит сначала остановить стек.
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