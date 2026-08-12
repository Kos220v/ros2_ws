# -*- coding: utf-8 -*-
"""
Тест автокалибровки HardwareIMU на эмулированной I2C-шине.

Класс HardwareIMU извлекается из imu_driver.py через AST, поэтому тест
работает и без rclpy (например, локально, если ROS 2 не установлен):

    pytest robot_odom/test/test_imu.py

В колконе запускается обычным образом:
    colcon test --packages-select robot_odom
"""

import ast
import math
import os

import numpy as np
import pytest


# --- фейковый smbus2 -------------------------------------------------------

class FakeSMBus2Module:
    """Эмуляция модуля smbus2 без установки пакета."""

    class SMBus:
        def __init__(self, bus_num):
            # MPU6050 в покое: acc = (0, 0, +1g), гироскоп имеет смещение 150 LSB по Z.
            self.regs = {
                0x3B: 0x00, 0x3C: 0x00,   # ax = 0
                0x3D: 0x00, 0x3E: 0x00,   # ay = 0
                0x3F: 0x40, 0x40: 0x00,   # az = 16384 (= 1g)
                0x43: 0x00, 0x44: 0x00,   # gx = 0
                0x45: 0x00, 0x46: 0x00,   # gy = 0
                0x47: 0x00, 0x48: 0x96,   # gz = 150 LSB
            }

        def write_byte_data(self, addr, reg, val):
            pass

        def read_byte_data(self, addr, reg):
            if addr == 0x68:
                return self.regs[reg]
            raise OSError(f"Нет устройства по адресу {addr:#x}")


# --- загрузка HardwareIMU из исходника через AST ---------------------------

def _load_hardware_imu():
    src_path = os.path.join(os.path.dirname(__file__), '..', 'robot_odom', 'imu_driver.py')
    src = open(os.path.abspath(src_path), encoding='utf-8').read()
    tree = ast.parse(src)
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == 'HardwareIMU')
    ns = {
        'np': np,
        'math': math,
        'time': __import__('time'),
        'smbus2': FakeSMBus2Module,
        'MPU_ACCEL_SCALE': 16384.0,
        'MPU_GYRO_SCALE': 131.0,
        'GRAVITY': 9.81,
    }
    exec(compile(ast.Module(body=[cls], type_ignores=[]), '<test>', 'exec'), ns)
    return ns['HardwareIMU']


# --- тесты -----------------------------------------------------------------

@pytest.fixture(scope='module')
def imu():
    HardwareIMU = _load_hardware_imu()
    dev = HardwareIMU(bus_num=1)
    assert dev.mag_type is None  # магнитометра на фейковой шине нет
    dev.calibrate(samples=30, interval=0.001)
    return dev


def test_gyro_bias_detected(imu):
    """Смещение гироскопа 150 LSB по Z должно быть точно оценено."""
    expected = 150.0 / 131.0 * (math.pi / 180.0)
    assert np.allclose(imu.gyro_bias, [0.0, 0.0, expected], atol=1e-4)


def test_acc_bias_when_flat(imu):
    """Робот лежит плашмя z-вверх: смещение акселерометра ≈ 0."""
    assert np.allclose(imu.acc_bias, 0.0, atol=1e-4)


def test_get_data_clean(imu):
    """После калибровки: acc = g, gyro = 0 (bias вычтен)."""
    acc, gyro, mag = imu.get_data()
    assert np.allclose(acc, [0.0, 0.0, 9.81], atol=1e-3)
    assert np.allclose(gyro, [0.0, 0.0, 0.0], atol=1e-5)
    assert np.allclose(mag, 0.0)


def test_calibration_flag(imu):
    assert imu.calibrated is True
