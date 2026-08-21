#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
i2c_check — проверка датчиков на шине I2C без запуска ROS-узлов.

ЗАЧЕМ ОТДЕЛЬНАЯ УТИЛИТА
-----------------------
Когда датчик не отвечает, ROS только мешает: узлы падают, логи мешаются,
и непонятно, виновата программа или железо. Эта утилита обращается к шине
напрямую и отвечает на один вопрос — что физически подключено и живо.
Её можно запускать при полностью остановленном роботе.

    ros2 run robot_navigation i2c_check

Или вообще без ROS:

    python3 install/robot_navigation/lib/python3.12/site-packages/\\
robot_navigation/i2c_check.py

ЧТО ПРОВЕРЯЕТСЯ
---------------
  1. Доступна ли шина вообще (включён ли I2C, есть ли права).
  2. Какие адреса отвечают (аналог i2cdetect -y 1).
  3. Опознаются ли известные датчики робота.
  4. У MPU6050 — регистр WHO_AM_I, состояние сна, диапазон акселерометра
     и живое чтение: гравитация и гироскоп в покое.
  5. У магнитометра — модуль поля и правдоподобность величины.

ВАЖНО ПРО ОСТАНОВКУ РОБОТА
--------------------------
Запускайте утилиту, когда стек ОСТАНОВЛЕН. Если параллельно работает
mpu6050_control, оба процесса будут дёргать датчик одновременно, и
показания станут рваными.
"""

import argparse
import math
import os
import subprocess
import sys
import time


GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BOLD = '\033[1m'
RESET = '\033[0m'


# Адреса, которые встречаются на этом роботе и в похожих сборках.
KNOWN_DEVICES = {
    0x0D: 'магнитометр QMC5883L',
    0x1E: 'магнитометр HMC5883L',
    0x68: 'MPU6050 / MPU9250 (нога AD0 на землю)',
    0x69: 'MPU6050 / MPU9250 (нога AD0 на питание)',
    0x76: 'барометр BMP280',
    0x77: 'барометр BMP180 / BMP280',
    0x40: 'датчик тока INA219 или влажности HTU21',
    0x3C: 'дисплей OLED SSD1306',
}

# WHO_AM_I у MPU6050 и его распространённых клонов.
WHO_AM_I_VALUES = {
    0x68: 'MPU6050 (оригинал)',
    0x69: 'MPU6050 (клон)',
    0x70: 'MPU6500 или MPU9250',
    0x71: 'MPU9250',
    0x73: 'MPU9255',
    0x98: 'клон неизвестного производителя',
}

ACCEL_RANGES = {
    0x00: ('±2g', 16384.0),
    0x08: ('±4g', 8192.0),
    0x10: ('±8g', 4096.0),
    0x18: ('±16g', 2048.0),
}

GRAVITY = 9.80665


def ok(text, detail=''):
    print(f'{GREEN}[ OK ]{RESET} {text}')
    for row in filter(None, detail.split('\n')):
        print(f'       {row}')


def warn(text, detail=''):
    print(f'{YELLOW}[ ?? ]{RESET} {text}')
    for row in filter(None, detail.split('\n')):
        print(f'       {row}')


def fail(text, detail=''):
    print(f'{RED}[FAIL]{RESET} {text}')
    for row in filter(None, detail.split('\n')):
        print(f'       {row}')


def read_reg(bus, addr, reg, attempts=6, delay=0.05):
    """Обёртка: возвращает (значение или None, количество сбоев, ошибка)."""
    errors = 0
    last = None
    for _ in range(attempts):
        try:
            return bus.read_byte_data(addr, reg), errors, None
        except Exception as exc:
            last = exc
            errors += 1
            time.sleep(delay)
    return None, errors, last


def find_competing_process():
    """Ищет запущенные узлы, которые тоже работают с этими датчиками.

    Самая коварная причина ложного диагноза: стек робота не остановлен.
    Узел mpu6050_control раз в 5 секунд пытается подключиться и при этом
    выполняет ПОЛНЫЙ СБРОС датчика. Во время сброса (около 150 мс) чип не
    отвечает вообще. Если проверка попадёт в это окно, она сообщит о
    неисправности совершенно исправного датчика.
    """
    names = ('mpu6050_control', 'compass_control', 'imu_check')
    found = []
    try:
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            # Свой собственный процесс не считаем
            if int(entry) == os.getpid():
                continue
            try:
                with open(f'/proc/{entry}/cmdline', 'rb') as f:
                    cmdline = f.read().replace(b'\x00', b' ').decode(
                        'utf-8', 'replace')
            except Exception:
                continue
            for name in names:
                if name in cmdline and 'i2c_check' not in cmdline:
                    found.append((entry, name))
                    break
    except Exception:
        pass
    return found


def check_pin_functions():
    """Показывает, в каком режиме сейчас работают выводы GPIO 2 и 3.

    На Raspberry Pi один и тот же вывод могут забрать разные интерфейсы.
    GPIO 2 и 3 — это SDA1 и SCL1, то есть шина I2C-1. Если какой-то оверлей
    в config.txt переключил их на другую функцию, шина работать не будет,
    а сообщения об ошибках будут указывать куда угодно, только не на
    настоящую причину.

    Проверка не обязательная: если устройства на шине отвечают, значит с
    выводами всё в порядке. Но когда отвечают не все, знать это полезно.
    """
    for tool in (['pinctrl', 'get', '2,3'], ['raspi-gpio', 'get', '2,3']):
        try:
            res = subprocess.run(tool, capture_output=True, text=True,
                                 timeout=5)
        except (FileNotFoundError, subprocess.SubprocessError):
            continue

        out = (res.stdout or '').strip()
        if not out:
            continue

        # В режиме I2C инструменты пишут func SDA1 / SCL1 либо alt0 / a0
        low = out.lower()
        i2c_mode = ('sda1' in low and 'scl1' in low) or \
                   ('a0' in low or 'alt0' in low)

        if i2c_mode:
            ok('Выводы GPIO 2 и 3 работают как SDA1 / SCL1', out)
        else:
            warn('Выводы GPIO 2 и 3 НЕ в режиме I2C', out
                 + '\nКакой-то оверлей в /boot/firmware/config.txt забрал их '
                   'себе.\nЧаще всего это UART или SPI на тех же выводах.')
        return

    # Инструментов нет — не беда, это лишь вспомогательная проверка
    warn('Не найдены pinctrl и raspi-gpio — режим выводов не проверен',
         'Это не ошибка: проверка вспомогательная.')


def read_word_2c(bus, addr, reg):
    """Читает 16-битное знаковое значение (старший байт первым)."""
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) + low
    return val - 65536 if val >= 0x8000 else val


def scan(bus):
    """Возвращает список отвечающих адресов. Аналог i2cdetect -y 1."""
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except Exception:
            pass
    return found


def check_mpu(bus, addr):
    """Подробная проверка MPU6050 по найденному адресу."""
    print()
    print(f'--- MPU6050 по адресу 0x{addr:02X} ---')

    # WHO_AM_I — с повторами: одиночный сбой шины ещё ничего не значит
    who, errs, exc = read_reg(bus, addr, 0x75)

    if who is None:
        fail(f'Не читается WHO_AM_I после 6 попыток: {exc}',
             'Устройство ОТКЛИКАЕТСЯ на свой адрес (иначе его не было бы\n'
             'в списке выше), но не отдаёт содержимое регистра.\n'
             'Это НЕ похоже на сгоревший чип. Вероятные причины по порядку:\n'
             '\n'
             '  1. Стек робота не остановлен, и узел mpu6050_control\n'
             '     параллельно сбрасывает датчик. Остановите launch и\n'
             '     повторите проверку.\n'
             '  2. Слишком высокая частота шины при трёх устройствах.\n'
             '     Снизьте её до 100 кГц (см. README).\n'
             '  3. Плохой контакт или длинные провода SDA / SCL.')
        return False

    if errs:
        warn(f'WHO_AM_I прочитан только с {errs + 1}-й попытки',
             'Шина работает неустойчиво. Ниже смотрите долю сбоев чтения.')

    if who in WHO_AM_I_VALUES:
        ok(f'WHO_AM_I = 0x{who:02X} — {WHO_AM_I_VALUES[who]}')
    else:
        warn(f'WHO_AM_I = 0x{who:02X} — неизвестное значение',
             'Возможно, это не MPU6050, а другой датчик по тому же адресу.')

    # Питание и сон
    try:
        pwr = bus.read_byte_data(addr, 0x6B)
        if pwr & 0x40:
            warn('Датчик в режиме сна (бит SLEEP взведён)',
                 'Узел разбудит его при запуске. Само по себе не ошибка.')
        else:
            ok('Датчик активен (не спит)')
    except Exception as exc:
        fail(f'Не читается PWR_MGMT_1: {exc}')
        return False

    # Будим, чтобы прочитать реальные данные
    try:
        bus.write_byte_data(addr, 0x6B, 0x00)
        time.sleep(0.1)
    except Exception as exc:
        fail(f'Не удалось разбудить датчик: {exc}')
        return False

    # Диапазон акселерометра
    scale = 16384.0
    try:
        cfg = bus.read_byte_data(addr, 0x1C) & 0x18
        name, scale = ACCEL_RANGES.get(cfg, ('неизвестно', 16384.0))
        if cfg == 0x00:
            ok(f'Диапазон акселерометра: {name}')
        else:
            warn(f'Диапазон акселерометра: {name}',
                 'Драйвер рассчитан на ±2g. Показания будут занижены.')
    except Exception as exc:
        warn(f'Не читается ACCEL_CONFIG: {exc}')

    # Живое чтение
    print()
    print('Читаю данные 2 секунды, робот должен стоять неподвижно...')
    acc_samples, gyro_samples, errors = [], [], 0

    for _ in range(100):
        try:
            ax = read_word_2c(bus, addr, 0x3B) / scale * GRAVITY
            ay = read_word_2c(bus, addr, 0x3D) / scale * GRAVITY
            az = read_word_2c(bus, addr, 0x3F) / scale * GRAVITY
            gx = read_word_2c(bus, addr, 0x43) / 131.0
            gy = read_word_2c(bus, addr, 0x45) / 131.0
            gz = read_word_2c(bus, addr, 0x47) / 131.0
            acc_samples.append((ax, ay, az))
            gyro_samples.append((gx, gy, gz))
        except Exception:
            errors += 1
        time.sleep(0.02)

    if not acc_samples:
        fail('Ни одного успешного чтения',
             'Датчик отвечает на адрес, но данные не отдаёт. '
             'Похоже на неисправность или плохой контакт.')
        return False

    if errors:
        share = 100.0 * errors / (errors + len(acc_samples))
        warn(f'Сбоев чтения: {errors} из {errors + len(acc_samples)} '
             f'({share:.0f}%)',
             'Шина нестабильна. Обычно это длинные или неэкранированные '
             'провода.')
    else:
        ok(f'Чтение стабильно: {len(acc_samples)} измерений без сбоев')

    n = len(acc_samples)
    ax = sum(s[0] for s in acc_samples) / n
    ay = sum(s[1] for s in acc_samples) / n
    az = sum(s[2] for s in acc_samples) / n
    magnitude = math.sqrt(ax * ax + ay * ay + az * az)

    print()
    ok('Акселерометр (м/с²)',
       f'X={ax:+.2f}  Y={ay:+.2f}  Z={az:+.2f}\n'
       f'модуль = {magnitude:.2f} (должен быть около {GRAVITY:.2f})')

    if abs(magnitude - GRAVITY) > 1.5:
        fail(f'Модуль ускорения {magnitude:.2f} вместо {GRAVITY:.2f}',
             'Либо неверный диапазон акселерометра, либо датчик врёт.')
    else:
        ok('Гравитация измеряется правильно')

    # Ориентация осей: на ровной поверхности вся гравитация должна быть в Z
    if abs(az) > 8.0 and abs(ax) < 3.0 and abs(ay) < 3.0:
        ok('Датчик стоит горизонтально, ось Z смотрит вверх')
    elif abs(az) > 8.0:
        warn('Ось Z направлена вниз — датчик перевёрнут')
    else:
        warn('Гравитация распределена между осями',
             'Либо робот стоит под наклоном, либо плата IMU повёрнута.\n'
             'Поправьте imu_mount_roll_deg / imu_mount_pitch_deg.')

    gx = sum(s[0] for s in gyro_samples) / n
    gy = sum(s[1] for s in gyro_samples) / n
    gz = sum(s[2] for s in gyro_samples) / n
    drift = max(abs(gx), abs(gy), abs(gz))

    print()
    ok('Гироскоп в покое (град/с)', f'X={gx:+.2f}  Y={gy:+.2f}  Z={gz:+.2f}')

    if drift > 5.0:
        warn(f'Смещение нуля {drift:.1f} град/с — великовато',
             'Узел вычитает его при калибровке, но убедитесь, что робот\n'
             'действительно стоял неподвижно во время проверки.')
    else:
        ok('Смещение нуля в норме, калибровка его уберёт')

    return True


def check_mag(bus, addr):
    """Проверка магнитометра QMC5883L или HMC5883L."""
    print()
    print(f'--- Магнитометр по адресу 0x{addr:02X} ---')

    try:
        if addr == 0x0D:
            # QMC5883L: непрерывный режим, 200 Гц, ±8 Гаусс
            bus.write_byte_data(addr, 0x0B, 0x01)
            bus.write_byte_data(addr, 0x09, 0x1D)
            time.sleep(0.1)
            data = [bus.read_byte_data(addr, 0x00 + i) for i in range(6)]
            raw_x = data[0] | (data[1] << 8)
            raw_y = data[2] | (data[3] << 8)
            raw_z = data[4] | (data[5] << 8)
            lsb_to_t = 3e-8
        else:
            bus.write_byte_data(addr, 0x02, 0x00)
            time.sleep(0.1)
            raw_x = (bus.read_byte_data(addr, 0x03) << 8) | \
                bus.read_byte_data(addr, 0x04)
            raw_z = (bus.read_byte_data(addr, 0x05) << 8) | \
                bus.read_byte_data(addr, 0x06)
            raw_y = (bus.read_byte_data(addr, 0x07) << 8) | \
                bus.read_byte_data(addr, 0x08)
            lsb_to_t = 7.29927e-8
    except Exception as exc:
        fail(f'Не удалось прочитать магнитометр: {exc}')
        return False

    def signed(v):
        return v - 65536 if v >= 0x8000 else v

    mx = signed(raw_x) * lsb_to_t
    my = signed(raw_y) * lsb_to_t
    mz = signed(raw_z) * lsb_to_t
    total = math.sqrt(mx * mx + my * my + mz * mz) * 1e6

    ok('Магнитометр отвечает',
       f'X={mx * 1e6:+.1f}  Y={my * 1e6:+.1f}  Z={mz * 1e6:+.1f} мкТл\n'
       f'модуль = {total:.1f} мкТл')

    # Полное поле Земли: 25-65 мкТл. Заметно больше — рядом железо или магнит.
    if total < 15.0:
        warn(f'Поле {total:.1f} мкТл — слабовато',
             'Обычно поле Земли 25-65 мкТл. Возможно, датчик экранирован.')
    elif total > 120.0:
        warn(f'Поле {total:.1f} мкТл — намного больше земного',
             'Рядом с датчиком сильный источник магнитного поля:\n'
             'мотор, магнит или силовой провод под нагрузкой.\n'
             'Это и даёт большое смещение нуля при калибровке.')
    else:
        ok(f'Величина поля правдоподобна ({total:.1f} мкТл)')

    return True


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Проверка датчиков на шине I2C')
    parser.add_argument('--bus', type=int, default=1,
                        help='номер шины I2C (по умолчанию 1)')
    parser.add_argument('--force', action='store_true',
                        help='проверять, даже если стек робота запущен')
    # ROS передаёт свои аргументы даже при запуске через ros2 run — игнорируем их
    args, _ = parser.parse_known_args(
        argv if argv is not None else sys.argv[1:])

    print()
    print(f'{BOLD}ПРОВЕРКА ШИНЫ I2C-{args.bus}{RESET}')
    print('=' * 70)

    try:
        import smbus2
    except ImportError:
        fail('Не установлена библиотека smbus2',
             'Установите: sudo apt install python3-smbus2\n'
             '        или: pip3 install smbus2')
        return 1

    try:
        bus = smbus2.SMBus(args.bus)
    except Exception as exc:
        fail(f'Не удалось открыть шину I2C-{args.bus}: {exc}',
             'Проверьте:\n'
             '  1) I2C включён: sudo raspi-config -> Interface Options -> I2C\n'
             '  2) права: sudo usermod -aG i2c $USER, затем перелогиниться\n'
             '  3) файл устройства существует: ls -l /dev/i2c-*')
        return 1

    ok(f'Шина I2C-{args.bus} открыта')

    # Конкурирующий доступ — самая частая причина ложного диагноза
    competitors = find_competing_process()
    if competitors:
        print()
        procs = ', '.join(f'{name} (pid {pid})' for pid, name in competitors)
        fail('СТЕК РОБОТА НЕ ОСТАНОВЛЕН',
             f'Одновременно с проверкой работают: {procs}\n'
             '\n'
             'Узел mpu6050_control раз в 5 секунд выполняет ПОЛНЫЙ СБРОС\n'
             'датчика, и во время сброса чип не отвечает. Проверка почти\n'
             'наверняка объявит исправный датчик неисправным.\n'
             '\n'
             'Остановите launch (Ctrl+C) и запустите проверку заново.')
        print()
        if not args.force:
            print('Проверка прервана. Чтобы всё равно продолжить: --force')
            return 1
        warn('Продолжаю несмотря на предупреждение (--force)')

    # --- что вообще есть на шине ---
    print()
    print('--- Устройства на шине ---')
    found = scan(bus)

    if not found:
        fail('Не отвечает ни одно устройство',
             'Проблема в шине целиком, а не в отдельном датчике:\n'
             '  * не подключены провода SDA / SCL\n'
             '  * нет питания на датчиках\n'
             '  * отсутствует подтяжка линий к питанию\n'
             '  * I2C выключен в настройках системы')
        return 1

    for addr in found:
        name = KNOWN_DEVICES.get(addr, 'неизвестное устройство')
        print(f'       0x{addr:02X}  {name}')

    # --- режим выводов ---
    print()
    print('--- Режим выводов SDA / SCL ---')
    check_pin_functions()

    # --- MPU6050 ---
    mpu_addr = next((a for a in (0x68, 0x69) if a in found), None)
    if mpu_addr is None:
        print()
        fail('MPU6050 не найден (нет ни 0x68, ни 0x69)',
             'Шина рабочая — другие устройства отвечают. Значит, дело\n'
             'именно в этом модуле:\n'
             '  * проверьте питание VCC и землю ИМЕННО его платы\n'
             '  * проверьте пайку и контакты SDA / SCL\n'
             '  * попробуйте отключить остальные датчики и проверить его отдельно\n'
             '  * если модуль совмещённый (GY-87 и подобные), MPU мог выйти\n'
             '    из строя отдельно от магнитометра')
    else:
        check_mpu(bus, mpu_addr)

    # --- магнитометр ---
    mag_addr = next((a for a in (0x0D, 0x1E) if a in found), None)
    if mag_addr is None:
        print()
        warn('Магнитометр не найден (нет ни 0x0D, ни 0x1E)',
             'Без него не будет абсолютного курса.')
    else:
        check_mag(bus, mag_addr)

    print()
    print('=' * 70)
    if mpu_addr is not None and mag_addr is not None:
        print(f'  {GREEN}Оба датчика на месте.{RESET}')
    else:
        print(f'  {RED}Не все датчики найдены — см. подробности выше.{RESET}')
    print('=' * 70)
    print()

    try:
        bus.close()
    except Exception:
        pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
