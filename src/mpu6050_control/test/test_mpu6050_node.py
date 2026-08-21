#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Тесты узла MPU6050 без реального железа и без запущенного ROS.

ЗАЧЕМ
-----
Узел устроен так, что при отсутствии датчика он остаётся жив и повторяет
попытки подключения. У такой логики есть неприятное свойство: ветка
УСПЕШНОГО подключения и ветка ОТКАЗА выполняются в разных ситуациях, и
ошибка в одной из них может годами не проявляться.

Именно так и случилось: атрибут _reconnect_timer создавался уже ПОСЛЕ
первого вызова _try_connect(), а этот метод в конце его читает. Пока
датчик молчал, код до чтения не доходил, и всё выглядело исправным.
Узел падал ровно в тот момент, когда железо чинили и датчик отвечал.

Эти тесты проходят обе ветки на подставных объектах, поэтому подобная
ошибка ловится на этапе сборки, а не на роботе в поле.

Запуск:
    colcon test --packages-select mpu6050_control
    python3 -m pytest src/mpu6050_control/test/test_mpu6050_node.py -v
"""

import sys
import types

import pytest


# --------------------------------------------------------------------------
# Подставные объекты вместо rclpy и драйвера датчика
# --------------------------------------------------------------------------

class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(('info', msg))

    def warning(self, msg):
        self.messages.append(('warning', msg))

    def error(self, msg):
        self.messages.append(('error', msg))

    def debug(self, msg):
        self.messages.append(('debug', msg))


class FakeTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeNode:
    """Минимальная замена rclpy.node.Node."""

    def __init__(self, name):
        self._name = name
        self._params = {}
        self._logger = FakeLogger()
        self.timers = []
        self.publishers = []

    def declare_parameter(self, name, default=None, descriptor=None):
        self._params.setdefault(name, default)
        return FakeParameter(self._params[name])

    def get_parameter(self, name):
        return FakeParameter(self._params.get(name))

    def set_parameter_for_test(self, name, value):
        self._params[name] = value

    def create_publisher(self, msg_type, topic, qos):
        self.publishers.append(topic)
        self.published = []
        return types.SimpleNamespace(publish=self.published.append)

    def create_timer(self, period, callback):
        timer = FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return self._logger

    def get_clock(self):
        stamp = types.SimpleNamespace(sec=0, nanosec=0)
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))

    def destroy_node(self):
        pass


class WorkingIMU:
    """Датчик, который подключается и отдаёт данные."""

    def __init__(self, *args, **kwargs):
        self.mpu_addr = 0x68
        self.calibrated = False

    def calibrate(self, samples=100, interval=0.01, logger=None):
        self.calibrated = True

    def get_data(self):
        return [0.0, 0.0, 9.81], [0.0, 0.0, 0.0]

    def close(self):
        pass


class DeadIMU:
    """Датчик, который не отвечает: имитация обрыва или отказа."""

    def __init__(self, *args, **kwargs):
        raise RuntimeError('MPU6050 не отвечает: [Errno 5] Input/output error')


class FlakyIMU:
    """Подключается, но потом перестаёт отдавать данные."""

    def __init__(self, *args, **kwargs):
        self.mpu_addr = 0x68
        self.closed = False

    def calibrate(self, samples=100, interval=0.01, logger=None):
        pass

    def get_data(self):
        raise OSError(121, 'Remote I/O error')

    def close(self):
        self.closed = True


# --------------------------------------------------------------------------
# Загрузка тестируемого модуля с подменёнными зависимостями
# --------------------------------------------------------------------------

def _stub_if_missing(module_name, **attrs):
    """Создаёт заглушку модуля, только если настоящего нет.

    На роботе установлен полноценный ROS, и заглушки не подменяют ничего
    реального. На машине разработчика без ROS они позволяют прогнать
    тесты логики узла, не поднимая всё окружение.
    """
    try:
        __import__(module_name)
        return
    except ImportError:
        pass

    parts = module_name.split('.')
    for i in range(1, len(parts) + 1):
        name = '.'.join(parts[:i])
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)
        if i > 1:
            setattr(sys.modules['.'.join(parts[:i - 1])],
                    parts[i - 1], sys.modules[name])

    for key, value in attrs.items():
        setattr(sys.modules[module_name], key, value)


class _Msg:
    """Универсальная заглушка сообщения ROS с произвольными полями."""

    def __init__(self, *args, **kwargs):
        self.header = types.SimpleNamespace(
            stamp=None, frame_id='')
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.linear_acceleration = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration_covariance = [0.0] * 9


def install_stubs():
    """Ставит заглушки для зависимостей, которых может не быть вне робота."""
    _stub_if_missing('sensor_msgs')
    _stub_if_missing('sensor_msgs.msg', Imu=_Msg)
    _stub_if_missing('geometry_msgs')
    _stub_if_missing('geometry_msgs.msg', Vector3Stamped=_Msg)
    _stub_if_missing('std_msgs')
    _stub_if_missing('std_msgs.msg', Header=_Msg)
    _stub_if_missing('rcl_interfaces')
    _stub_if_missing('rcl_interfaces.msg', ParameterDescriptor=_Msg)
    # Драйвер тянет smbus2; настоящая работа с шиной в тестах не нужна,
    # потому что класс датчика всё равно подменяется.
    _stub_if_missing('smbus2', SMBus=lambda bus: None)


def load_node_class(imu_class):
    """Импортирует MPU6050Node, подменив rclpy и драйвер датчика."""
    install_stubs()

    for name in list(sys.modules):
        if name.startswith('mpu6050_control'):
            del sys.modules[name]

    fake_rclpy = types.ModuleType('rclpy')
    fake_rclpy.init = lambda *a, **k: None
    fake_rclpy.shutdown = lambda *a, **k: None
    fake_rclpy.spin = lambda *a, **k: None

    fake_node_mod = types.ModuleType('rclpy.node')
    fake_node_mod.Node = FakeNode
    fake_rclpy.node = fake_node_mod

    sys.modules['rclpy'] = fake_rclpy
    sys.modules['rclpy.node'] = fake_node_mod

    import mpu6050_control.mpu6050_node as node_mod
    import importlib
    importlib.reload(node_mod)

    # Подменяем драйвер уже после импорта модуля
    node_mod.HardwareIMU = imu_class
    return node_mod


# --------------------------------------------------------------------------
# Тесты
# --------------------------------------------------------------------------

def test_starts_when_sensor_works():
    """Датчик отвечает сразу — узел обязан подняться без исключений.

    Это тот случай, который раньше падал с AttributeError: ветка успеха
    читала _reconnect_timer, которого ещё не существовало.
    """
    mod = load_node_class(WorkingIMU)
    node = mod.MPU6050Node()

    assert node.imu is not None, 'датчик должен быть подключён'
    assert node._reconnect_timer is None, \
        'при успешном подключении таймер переподключения не нужен'
    assert '/imu/data' in node.publishers


def test_survives_missing_sensor():
    """Датчика нет — узел обязан остаться жив и завести таймер повторов."""
    mod = load_node_class(DeadIMU)
    node = mod.MPU6050Node()

    assert node.imu is None, 'подключения быть не должно'
    assert node._reconnect_timer is not None, \
        'без датчика обязан работать таймер переподключения'

    errors = [m for lvl, m in node.get_logger().messages if lvl == 'error']
    assert errors, 'причина отказа должна попасть в лог'


def test_reconnects_after_sensor_appears():
    """Датчик появился на второй попытке — узел должен это подхватить."""
    mod = load_node_class(DeadIMU)
    node = mod.MPU6050Node()
    assert node.imu is None

    timer = node._reconnect_timer
    assert timer is not None

    # Железо починили
    mod.HardwareIMU = WorkingIMU
    timer.callback()

    assert node.imu is not None, 'узел обязан подключиться при повторе'
    assert timer.cancelled, 'таймер повторов должен быть остановлен'
    assert node._reconnect_timer is None


def test_reconnects_after_losing_sensor():
    """Датчик отвалился на ходу — узел должен переподключиться."""
    mod = load_node_class(FlakyIMU)
    node = mod.MPU6050Node()
    assert node.imu is not None

    max_errors = node._max_read_errors
    imu = node.imu

    # Копим ошибки чтения до порога
    for _ in range(max_errors):
        node.timer_callback()

    assert node.imu is None, 'после серии ошибок датчик считается потерянным'
    assert imu.closed, 'шина должна быть закрыта перед переподключением'
    assert node._reconnect_timer is not None, \
        'должен запуститься таймер переподключения'


def test_publishes_after_successful_start():
    """После штатного запуска узел обязан публиковать данные.

    Отдельный тест именно на ПЕРВУЮ публикацию. Раньше её никто не
    проверял, и это дорого обошлось: при правке конструктора блок
    инициализации ковариаций уехал в другой метод. Узел стартовал без
    единой жалобы, а падал уже в рабочем цикле, на первом же сообщении.

    Мораль: недостаточно проверить, что узел ЗАПУСТИЛСЯ. Надо проверять,
    что он РАБОТАЕТ.
    """
    mod = load_node_class(WorkingIMU)
    node = mod.MPU6050Node()

    node.timer_callback()

    published = node.published
    assert len(published) == 1, 'должно уйти ровно одно сообщение'

    msg = published[0]
    assert msg.header.frame_id == 'imu_link'
    # Ориентация не заполняется: её считает imu_filter_madgwick
    assert msg.orientation_covariance[0] == -1.0
    # Ковариации обязаны быть заполнены — ровно этого и не хватало
    assert len(msg.angular_velocity_covariance) == 9
    assert len(msg.linear_acceleration_covariance) == 9
    assert msg.linear_acceleration.z == pytest.approx(9.81)


def test_covariances_ready_before_first_publish():
    """Поля ковариаций существуют сразу после конструктора.

    Проверяем не поведение, а порядок инициализации: любое поле, которое
    читает рабочий цикл, обязано быть готово до создания таймера.
    """
    mod = load_node_class(WorkingIMU)
    node = mod.MPU6050Node()

    assert hasattr(node, 'angular_cov'), \
        'ковариации должны считаться в конструкторе, а не по ходу дела'
    assert hasattr(node, 'linear_cov')
    assert len(node.angular_cov) == 9
    assert len(node.linear_cov) == 9


def test_single_read_error_does_not_trigger_reconnect():
    """Одиночный сбой шины — обычное дело, переподключаться не нужно."""
    mod = load_node_class(WorkingIMU)
    node = mod.MPU6050Node()

    calls = {'n': 0}
    original = node.imu.get_data

    def flaky():
        calls['n'] += 1
        if calls['n'] == 1:
            raise OSError(121, 'Remote I/O error')
        return original()

    node.imu.get_data = flaky

    node.timer_callback()               # сбой
    assert node.imu is not None, 'один сбой не повод отключаться'

    node.timer_callback()               # успех
    assert node._read_errors == 0, \
        'успешное чтение обязано обнулять счётчик ошибок'


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
