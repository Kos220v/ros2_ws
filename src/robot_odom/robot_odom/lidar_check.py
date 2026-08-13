#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Диагностика YDLidar: ошибка драйвера
    "Fail to get baseplate device information!"

Что это значит: SDK лидара не смог прочитать информацию с платы датчика
(базы) через serial-порт — т.е. связи с лидаром НЕТ. Причины (по частоте):

  1. Неверный/отсутствующий порт в params драйвера (port: /dev/ttyUSB0),
     а у лидара другой (ttyUSB1, ttyACM0) или его вообще нет (USB не
     поднялся / не подключён / не запитался).
  2. Нет прав на порт: пользователь не в группе dialout (или udev-правило
     не даёт MODE="0666").
  3. Порт занят другим процессом (GPS на том же USB-адаптере, второй
     экземпляр драйвера, ModemManager).
  4. Питание лидара: не горит LED, мотор не крутится (хаб/USB-порт без
     достаточного тока, разъём).
  5. Несовпадение модели/скорости в params (baudrate 115200 для X2/X4;
     128000 для некоторых G-серий; lidar_type: 0 для serial).

Запуск:
    ros2 run robot_odom lidar_check
    ros2 run robot_odom lidar_check --probe   # попытаться «постучаться»
                                              # в каждый порт на 115200
"""

import argparse
import glob
import os
import subprocess
import sys

BAUD_CANDIDATES = [115200, 128000, 230400, 921600]

LIDAR_PATTERNS = [
    '/dev/ttyUSB*',
    '/dev/ttyACM*',
    '/dev/ydlidar',
    '/dev/serial/by-id/*',
]


def find_candidates():
    found = []
    for pattern in LIDAR_PATTERNS:
        found.extend(sorted(glob.glob(pattern)))
    # дедупликация с сохранением порядка
    return list(dict.fromkeys(found))


def check_port(port):
    """Возвращает (ok, проблемы, инфо)."""
    problems = []
    info = []
    if not os.path.exists(port):
        return False, [f"{port} НЕ существует"], []
    st = os.stat(port)
    import stat as st_mod
    mode = st_mod.filemode(st.st_mode)
    info.append(f"{port}: {mode}")
    # права: группа dialout / uucp / tty / other-w
    if not (st.st_mode & 0o002):
        problems.append(
            f"{port}: нет доступа для остальных (нет o+w). "
            "Добавьте себя в группу dialout: sudo usermod -aG dialout $USER "
            "(перелогиниться) или установите udev-правило MODE=\"0666\"."
        )
    # занят ли порт другим процессом
    try:
        out = subprocess.run(
            ['fuser', '-v', port], capture_output=True, text=True, timeout=3)
        if out.returncode == 0:
            problems.append(
                f"{port}: ЗАНЯТ другим процессом:\n{out.stderr.strip()}\n"
                "Убейте процесс (или найдите, кто держит порт): "
                "sudo lsof " + port
            )
    except FileNotFoundError:
        pass
    except Exception:
        pass
    return True, problems, info


def probe_port(port, baud, timeout=0.6):
    """Пытается открыть порт и прочитать что-нибудь (сырой тест связи)."""
    try:
        import serial
    except ImportError:
        return 'no-pyserial'
    try:
        with serial.Serial(port=port, baudrate=baud, timeout=timeout) as s:
            s.reset_input_buffer()
            data = s.read(32)
            if data:
                return f'OK: получено {len(data)} байт (первые: {data[:8].hex()})'
            return ('тишина (порт открылся, но данных нет — это нормально, '
                    'лидар отвечает только на команды)')
    except Exception as e:
        return f'ОШИБКА: {e}'


def udev_hint():
    print()
    print("Рекомендуемое udev-правило для лидара "
          "(создать /etc/udev/rules.d/99-ydlidar.rules):")
    print("    KERNEL==\"ttyUSB*\", MODE=\"0666\"")
    print("    KERNEL==\"ttyACM*\", MODE=\"0666\"")
    print("    KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"10c4\", "
          "ATTRS{idProduct}==\"ea60\", SYMLINK+=\"ydlidar\", MODE=\"0666\"")
    print("  (idVendor/idProduct посмотрите: lsusb | grep -i -E 'cp210|silicon|10c4')")
    print("  Затем: sudo udevadm control --reload-rules && "
          "sudo udevadm trigger")
    print()


def main():
    ap = argparse.ArgumentParser(description='Диагностика YDLidar')
    ap.add_argument('--probe', action='store_true',
                    help='открыть каждый найденный порт и попытаться прочитать')
    args = ap.parse_args()

    print("=== Диагностика YDLidar: 'Fail to get baseplate device information!' ===\n")

    # 1. USB-устройства
    print("--- USB (lsusb) ---")
    try:
        out = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5)
        for line in out.stdout.splitlines():
            if any(k in line.lower() for k in ('cp210', 'silicon', 'ftdi', 'ch340',
                                               '10c4', '1a86', '0403')):
                print("  [похоже на USB-UART/лидар]", line)
            else:
                print("  ", line)
    except Exception as e:
        print(f"  lsusb недоступен: {e}")

    # 2. Кандидаты-порты
    cands = find_candidates()
    print(f"\n--- Порты-кандидаты ({len(cands)}) ---")
    if not cands:
        print("  НЕТ /dev/ttyUSB* и /dev/ttyACM*!")
        print("  Проверьте: 1) лидар подключён и запитался (LED горит, мотор "
              "крутится);")
        print("  2) USB виден: dmesg | tail -30  (после подключения);")
        print("  3) кабель/разъём/хаб.")
    problems_total = 0
    for p in cands:
        ok, problems, info = check_port(p)
        for i in info:
            print(" ", i)
        if not ok:
            print(f"  ? {p} НЕ существует")
        for pr in problems:
            print("  ?", pr)
            problems_total += 1

    # 3. Проверка группы dialout
    print("\n--- Права доступа ---")
    try:
        groups = subprocess.run(['groups'], capture_output=True, text=True).stdout
        print(f"  Ваши группы: {groups.strip()}")
        if 'dialout' not in groups and 'uucp' not in groups:
            print("  ? Нет группы dialout/uucp — вероятно, нет прав на порт.")
            print("    Решение: sudo usermod -aG dialout $USER  (и перелогиниться)")
        else:
            print("  ? Группа dialout/uucp есть.")
    except Exception as e:
        print(f"  Не удалось проверить группы: {e}")

    # 4. Кто держит порты (lsof)
    print("\n--- Кто держит порты ---")
    try:
        out = subprocess.run(['lsof', '+D', '/dev'], capture_output=True,
                             text=True, timeout=10)
        lines = [l for l in out.stdout.splitlines()
                 if 'ttyUSB' in l or 'ttyACM' in l]
        if lines:
            print("  Найденные процессы на serial-портах:")
            for l in lines:
                print("  ", l)
            print("  Если порт лидара держит ДРУГОЙ процесс (GPS/второй "
                  "драйвер) — освободите его.")
        else:
            print("  Свободны (lsof не показал держателей ttyUSB/ttyACM).")
    except Exception as e:
        print(f"  lsof недоступен: {e}")

    # 5. Опционально: probe
    if args.probe:
        print("\n--- Probe портов ---")
        for p in cands:
            if not os.path.exists(p):
                continue
            for baud in BAUD_CANDIDATES:
                r = probe_port(p, baud)
                print(f"  {p} @ {baud}: {r}")
                if r.startswith('OK'):
                    break

    # 6. Где конфиг драйвера
    print("\n--- Где искать/править конфиг драйвера ---")
    print("  Параметры лидара (port, baudrate, lidar_type, frame_id) — в")
    print("  params-файле установленного пакета ydlidar_ros2_driver:")
    print("    ros2 pkg prefix ydlidar_ros2_driver")
    print("  и посмотрите:  <prefix>/share/ydlidar_ros2_driver/params/ydlidar.yaml")
    print("  Проверьте: port (должен совпадать с реальным), baudrate "
          "(X2/X4 = 115200), lidar_type (0 = serial).")
    print("  Быстрый тест лидара отдельно от стека:")
    print("    ros2 launch ydlidar_ros2_driver ydlidar_launch.py")
    print("  (если в одиночку работает — конфликт/порт в общем стеке).")

    udev_hint()

    print("=== Итог ===")
    if problems_total:
        print(f"  Найдено проблем: {problems_total}. Исправьте и повторите.")
    elif cands:
        print("  Порты на месте, права в порядке, порты не заняты.")
        print("  Если ошибка остаётся — проверьте ПИТАНИЕ лидара и"
              " baudrate/модель в params драйвера.")
    else:
        print("  Порт лидара не найден — начинайте с USB/питания.")
    return 0 if problems_total == 0 else 2


if __name__ == '__main__':
    sys.exit(main())