#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mag_calibrator — калибровка магнитометра вращением робота.

ЗАЧЕМ
-----
Идеальный магнитометр при вращении робота описывает концом вектора ШАР
с центром в нуле. Реальный, установленный на робота с моторами, батареей
и стальной рамой, описывает ЭЛЛИПСОИД, СМЕЩЁННЫЙ от нуля:

  * смещение центра (hard iron) — постоянное поле самого робота;
  * вытянутость (soft iron)     — искажение поля железом рядом с датчиком.

Утилита собирает измерения, пока вы вращаете робота, находит центр и размеры
этой фигуры и печатает готовые параметры, которые нужно вставить в конфиг.

ПОРЯДОК РАБОТЫ
--------------
1. Вынесите робота на открытое место, ПОДАЛЬШЕ от металла: не в гараже,
   не рядом с машиной, не на железном столе и не на арматуре в бетоне.

2. Запустите слой железа (питание робота должно быть ВКЛЮЧЕНО целиком —
   калибруется робот в сборе, а не датчик сам по себе):

       ros2 launch project_start start.launch.py

3. В другом терминале запустите калибровку:

       ros2 run robot_navigation mag_calibrator

4. Медленно вращайте робота вокруг вертикальной оси — 2-3 полных оборота
   примерно за минуту. Можно крутить на месте гусеницами с пульта.
   Утилита показывает, какая часть круга уже пройдена.

5. По окончании она напечатает готовый блок параметров. Скопируйте его
   в файл, путь к которому будет указан, и перезапустите робота.

ЧЕГО ДЕЛАТЬ НЕ НАДО
-------------------
Не наклоняйте и не переворачивайте робота: калибровка рассчитана на
вращение вокруг вертикальной оси, потому что курс считается по
горизонтальной составляющей поля. Наклоны только испортят выборку.
"""

import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import MagneticField


# Круг делим на 36 секторов по 10°: так видно, равномерно ли отработано
# вращение. Пропущенные сектора смещают оценку центра.
N_SECTORS = 36

GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BOLD = '\033[1m'
RESET = '\033[0m'


class MagCalibrator(Node):

    def __init__(self):
        super().__init__('mag_calibrator')

        self.declare_parameter('input_topic', '/imu/mag_raw')
        self.declare_parameter('duration', 60.0)
        self.declare_parameter(
            'output_file',
            os.path.expanduser('~/mag_calibration.yaml'))
        # Минимальная доля круга, при которой результату можно верить.
        self.declare_parameter('min_coverage', 0.8)

        self._duration = float(self.get_parameter('duration').value)
        self._min_coverage = float(self.get_parameter('min_coverage').value)
        self._out_path = self.get_parameter('output_file').value

        self._samples = []
        self._sectors = set()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            MagneticField, self.get_parameter('input_topic').value,
            self._on_mag, qos)

        print()
        print(f'{BOLD}КАЛИБРОВКА МАГНИТОМЕТРА{RESET}')
        print('=' * 66)
        print(f'Медленно вращайте робота вокруг вертикальной оси.')
        print(f'Нужно 2-3 полных оборота за {self._duration:.0f} секунд.')
        print(f'Наклонять робота НЕ надо.')
        print('=' * 66)
        print()

        self._elapsed = 0.0
        self.create_timer(1.0, self._tick)

    def _on_mag(self, msg: MagneticField):
        x = msg.magnetic_field.x
        y = msg.magnetic_field.y
        z = msg.magnetic_field.z

        # Нулевой вектор = обрыв связи с датчиком, такие точки не нужны.
        if abs(x) < 1e-12 and abs(y) < 1e-12:
            return

        self._samples.append((x, y, z))

        # Сектор считаем по СЫРОМУ углу. Он смещён из-за hard iron, но для
        # оценки полноты оборота этого достаточно.
        angle = math.atan2(y, x)
        sector = int((angle + math.pi) / (2 * math.pi) * N_SECTORS) % N_SECTORS
        self._sectors.add(sector)

    def _coverage(self):
        return len(self._sectors) / float(N_SECTORS)

    def _tick(self):
        self._elapsed += 1.0
        cov = self._coverage()

        # Наглядная полоска: какие сектора круга уже пройдены.
        bar = ''.join(
            '#' if i in self._sectors else '.' for i in range(N_SECTORS))
        remaining = max(0.0, self._duration - self._elapsed)

        sys.stdout.write(
            f'\r  [{bar}]  пройдено {cov * 100:5.1f}%  '
            f'точек {len(self._samples):5d}  осталось {remaining:4.0f} с')
        sys.stdout.flush()

        if self._elapsed >= self._duration:
            print()
            self._finish()

    # ------------------------------------------------------------------ расчёт
    def _finish(self):
        print()

        if len(self._samples) < 100:
            print(f'{RED}Слишком мало измерений ({len(self._samples)}).{RESET}')
            print('Проверьте, что compass_control запущен и публикует '
                  'в /imu/mag_raw.')
            rclpy.shutdown()
            return

        cov = self._coverage()

        xs = [s[0] for s in self._samples]
        ys = [s[1] for s in self._samples]
        zs = [s[2] for s in self._samples]

        # Центр эллипса = середина между крайними значениями по каждой оси.
        # Это и есть смещение нуля (hard iron).
        off_x = (max(xs) + min(xs)) / 2.0
        off_y = (max(ys) + min(ys)) / 2.0
        off_z = (max(zs) + min(zs)) / 2.0

        # Полуоси эллипса.
        rad_x = (max(xs) - min(xs)) / 2.0
        rad_y = (max(ys) - min(ys)) / 2.0

        if rad_x < 1e-9 or rad_y < 1e-9:
            print(f'{RED}Поле почти не менялось — робот не вращался.{RESET}')
            print('Повторите калибровку, вращая робота на месте.')
            rclpy.shutdown()
            return

        # Приводим полуоси к общему среднему: эллипс становится окружностью.
        avg = (rad_x + rad_y) / 2.0
        scale_x = avg / rad_x
        scale_y = avg / rad_y
        # Z в определении курса не участвует (вращение вокруг вертикали
        # его не меняет), поэтому масштаб по Z оставляем единичным.
        scale_z = 1.0

        self._print_report(cov, off_x, off_y, off_z,
                           rad_x, rad_y, scale_x, scale_y, scale_z)

        self._save(off_x, off_y, off_z, scale_x, scale_y, scale_z)

        rclpy.shutdown()

    def _print_report(self, cov, off_x, off_y, off_z,
                      rad_x, rad_y, scale_x, scale_y, scale_z):
        uT = 1e6   # тесла -> микротесла, только для удобства чтения

        print('=' * 66)
        print(f'{BOLD}РЕЗУЛЬТАТ{RESET}')
        print('=' * 66)
        print(f'Измерений: {len(self._samples)}, '
              f'пройдено круга: {cov * 100:.1f}%')
        print()

        # --- оценка качества ------------------------------------------------
        ok = True

        if cov < self._min_coverage:
            ok = False
            print(f'{RED}[ПЛОХО]{RESET} Круг пройден не полностью '
                  f'({cov * 100:.0f}%, нужно {self._min_coverage * 100:.0f}%).')
            print('        Непройденные сектора смещают оценку центра.')
            print('        Повторите калибровку, сделав полные обороты.')
        else:
            print(f'{GREEN}[ОК]{RESET}    Круг пройден полностью.')

        # Насколько смещение велико по сравнению с самим полем Земли.
        field = (rad_x + rad_y) / 2.0
        offset_mag = math.hypot(off_x, off_y)
        ratio = offset_mag / field if field > 0 else 0.0

        print(f'        Поле Земли (горизонт.): {field * uT:.2f} мкТл')
        print(f'        Смещение нуля:          {offset_mag * uT:.2f} мкТл '
              f'({ratio * 100:.0f}% от поля)')

        if ratio > 2.0:
            ok = False
            print(f'{RED}[ПЛОХО]{RESET} Смещение втрое больше самого поля.')
            print('        Магнитометр стоит вплотную к источнику магнитного')
            print('        поля — мотору, силовому проводу или магниту.')
            print('        Перенесите плату дальше и повторите.')
        elif ratio > 0.5:
            print(f'{YELLOW}[ТЕРПИМО]{RESET} Смещение заметное, но '
                  f'компенсируемое.')
        else:
            print(f'{GREEN}[ОК]{RESET}    Смещение небольшое.')

        # Вытянутость эллипса.
        distortion = max(rad_x, rad_y) / min(rad_x, rad_y)
        print(f'        Вытянутость эллипса:    {distortion:.2f}')
        if distortion > 1.5:
            print(f'{YELLOW}[ТЕРПИМО]{RESET} Поле сильно искажено железом '
                  f'рядом с датчиком.')
        else:
            print(f'{GREEN}[ОК]{RESET}    Форма близка к правильной.')

        print()
        print('=' * 66)
        print(f'{BOLD}ВСТАВЬТЕ ЭТО В config/mag_calibration.yaml{RESET}')
        print('=' * 66)
        print()
        print('mag_declination_node:')
        print('  ros__parameters:')
        print(f'    hard_iron_x: {off_x:.10e}')
        print(f'    hard_iron_y: {off_y:.10e}')
        print(f'    hard_iron_z: {off_z:.10e}')
        print(f'    soft_iron_scale_x: {scale_x:.6f}')
        print(f'    soft_iron_scale_y: {scale_y:.6f}')
        print(f'    soft_iron_scale_z: {scale_z:.6f}')
        print()
        print('=' * 66)

        if ok:
            print(f'{GREEN}Калибровка удалась.{RESET} Дальше:')
        else:
            print(f'{RED}К результату есть вопросы (см. выше).{RESET} '
                  f'Всё равно можно продолжить, но лучше повторить. Дальше:')

        print()
        print('  1. Впишите значения в config/mag_calibration.yaml')
        print('  2. Перезапустите робота')
        print('  3. Определите угол монтажа платы:')
        print('       ros2 run robot_navigation heading_check \\')
        print('           --ros-args -p true_azimuth_deg:=<азимут по компасу>')
        print()

    def _save(self, off_x, off_y, off_z, scale_x, scale_y, scale_z):
        """Дублирует результат в файл — чтобы не потерять при закрытии окна."""
        try:
            directory = os.path.dirname(os.path.abspath(self._out_path))
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self._out_path, 'w', encoding='utf-8') as f:
                f.write('# Результат калибровки магнитометра.\n')
                f.write('# Скопируйте в robot_navigation/config/'
                        'mag_calibration.yaml\n')
                f.write('mag_declination_node:\n')
                f.write('  ros__parameters:\n')
                f.write(f'    hard_iron_x: {off_x:.10e}\n')
                f.write(f'    hard_iron_y: {off_y:.10e}\n')
                f.write(f'    hard_iron_z: {off_z:.10e}\n')
                f.write(f'    soft_iron_scale_x: {scale_x:.6f}\n')
                f.write(f'    soft_iron_scale_y: {scale_y:.6f}\n')
                f.write(f'    soft_iron_scale_z: {scale_z:.6f}\n')
            print(f'Копия результата сохранена в {self._out_path}')
            print()
        except Exception as exc:
            print(f'{YELLOW}Не удалось сохранить файл: {exc}{RESET}')


def main(args=None):
    rclpy.init(args=args)
    node = MagCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nПрервано пользователем.')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
