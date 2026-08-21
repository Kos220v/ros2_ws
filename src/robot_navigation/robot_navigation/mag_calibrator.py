#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mag_calibrator — калибровка магнитометра вращением робота.

ЗАЧЕМ
-----
Идеальный магнитометр при вращении робота описывает концом вектора
ОКРУЖНОСТЬ с центром в нуле. Реальный, установленный на робота с моторами,
батареей и стальной рамой, описывает окружность, СМЕЩЁННУЮ от нуля и слегка
сплюснутую:

  * смещение центра (hard iron) — постоянное поле самого робота;
  * сплюснутость (soft iron)    — искажение поля железом рядом с датчиком.

Утилита находит центр и размеры этой окружности и печатает готовые
параметры для config/mag_calibration.yaml.

КАК НАХОДИТСЯ ЦЕНТР
-------------------
Методом наименьших квадратов (алгебраическая подгонка окружности Косы),
а не по крайним значениям min/max. Разница принципиальна:

  * min/max требует ПОЛНОГО оборота и чувствителен к единственному
    выбросу — одна помеха сдвигает оценку центра;
  * подгонка по всем точкам устойчива к выбросам и работает даже когда
    оборот пройден не полностью.

ПОЧЕМУ ОХВАТ СЧИТАЕТСЯ ВОКРУГ ЦЕНТРА, А НЕ ВОКРУГ НУЛЯ
------------------------------------------------------
Когда смещение больше самого поля Земли, окружность измерений вообще не
охватывает начало координат. Робот делает полный оборот, а сырой угол
atan2(y, x) меняется всего на десяток-другой градусов. Если считать охват
по сырому углу, утилита сообщит «пройдено 8%» при честно выполненных трёх
оборотах и собьёт с толку. Поэтому охват считается по углу ОТНОСИТЕЛЬНО
НАЙДЕННОГО ЦЕНТРА — эта величина отражает реальное вращение робота.

ПОРЯДОК РАБОТЫ
--------------
1. Вынесите робота на открытое место, ПОДАЛЬШЕ от металла: не в гараже,
   не рядом с машиной, не на железном столе и не над арматурой в бетоне.

2. Запустите слой железа (питание робота включено целиком — калибруется
   робот в сборе, а не датчик сам по себе):

       ros2 launch project_start start.launch.py

3. В другом терминале:

       ros2 run robot_navigation mag_calibrator

4. Медленно вращайте робота вокруг вертикальной оси — 2-3 полных оборота
   примерно за минуту. Можно крутить на месте гусеницами с пульта.

5. Скопируйте напечатанные значения в config/mag_calibration.yaml.

ЧЕГО ДЕЛАТЬ НЕ НАДО
-------------------
Не наклоняйте и не переворачивайте робота: расчёт рассчитан на вращение
вокруг вертикальной оси, потому что курс определяется горизонтальной
составляющей поля.
"""

import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import MagneticField


# Круг делим на 36 секторов по 10°: так видно, равномерно ли отработано
# вращение. Пропущенные сектора ухудшают оценку.
N_SECTORS = 36

GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BOLD = '\033[1m'
RESET = '\033[0m'

UT = 1e6   # тесла -> микротесла, только для удобства чтения


def fit_circle(xs, ys):
    """Подгоняет окружность по методу наименьших квадратов (метод Косы).

    Решает переопределённую систему
        x² + y² = 2·a·x + 2·b·y + c
    относительно (a, b, c). Центр окружности — (a, b),
    радиус — sqrt(c + a² + b²).

    Возвращает (cx, cy, r) либо None, если точки вырождены
    (лежат на прямой или совпадают).
    """
    x = np.asarray(xs, dtype=float)
    y = np.asarray(ys, dtype=float)

    a_mat = np.column_stack([2.0 * x, 2.0 * y, np.ones_like(x)])
    b_vec = x * x + y * y

    try:
        sol, *_ = np.linalg.lstsq(a_mat, b_vec, rcond=None)
    except np.linalg.LinAlgError:
        return None

    cx, cy, c = sol
    r_sq = c + cx * cx + cy * cy
    if not np.isfinite(r_sq) or r_sq <= 0.0:
        return None

    return float(cx), float(cy), float(math.sqrt(r_sq))


def coverage_sectors(xs, ys, cx, cy):
    """Множество пройденных секторов при взгляде ИЗ ЦЕНТРА окружности."""
    sectors = set()
    for x, y in zip(xs, ys):
        angle = math.atan2(y - cy, x - cx)
        sectors.add(
            int((angle + math.pi) / (2 * math.pi) * N_SECTORS) % N_SECTORS)
    return sectors


class MagCalibrator(Node):

    def __init__(self):
        super().__init__('mag_calibrator')

        self.declare_parameter('input_topic', '/imu/mag_raw')
        self.declare_parameter('duration', 60.0)
        self.declare_parameter(
            'output_file', os.path.expanduser('~/mag_calibration.yaml'))
        # Минимальная доля круга, при которой результату можно верить.
        self.declare_parameter('min_coverage', 0.8)

        self._duration = float(self.get_parameter('duration').value)
        self._min_coverage = float(self.get_parameter('min_coverage').value)
        self._out_path = self.get_parameter('output_file').value

        self._xs, self._ys, self._zs = [], [], []

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
        print('Медленно вращайте робота вокруг вертикальной оси.')
        print(f'Нужно 2-3 полных оборота за {self._duration:.0f} секунд.')
        print('Наклонять робота НЕ надо.')
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

        self._xs.append(x)
        self._ys.append(y)
        self._zs.append(z)

    def _tick(self):
        self._elapsed += 1.0
        remaining = max(0.0, self._duration - self._elapsed)
        n = len(self._xs)

        # Пересчитываем центр на каждом шаге: подгонка по 1500 точкам
        # занимает доли миллисекунды, зато полоска показывает честный
        # охват, а не угол вокруг нуля.
        bar = '.' * N_SECTORS
        cov = 0.0
        if n >= 50:
            fit = fit_circle(self._xs, self._ys)
            if fit is not None:
                cx, cy, _ = fit
                sectors = coverage_sectors(self._xs, self._ys, cx, cy)
                cov = len(sectors) / float(N_SECTORS)
                bar = ''.join(
                    '#' if i in sectors else '.' for i in range(N_SECTORS))

        sys.stdout.write(
            f'\r  [{bar}]  пройдено {cov * 100:5.1f}%  '
            f'точек {n:5d}  осталось {remaining:4.0f} с')
        sys.stdout.flush()

        if self._elapsed >= self._duration:
            print()
            self._finish()

    # ------------------------------------------------------------------ расчёт
    def _finish(self):
        print()

        if len(self._xs) < 100:
            print(f'{RED}Слишком мало измерений ({len(self._xs)}).{RESET}')
            print('Проверьте, что compass_control запущен и публикует '
                  'в /imu/mag_raw:')
            print('  ros2 topic hz /imu/mag_raw')
            rclpy.shutdown()
            return

        fit = fit_circle(self._xs, self._ys)
        if fit is None:
            print(f'{RED}Не удалось подобрать окружность.{RESET}')
            print('Похоже, робот не вращался или данные вырождены.')
            rclpy.shutdown()
            return

        off_x, off_y, radius = fit
        sectors = coverage_sectors(self._xs, self._ys, off_x, off_y)
        cov = len(sectors) / float(N_SECTORS)

        # --- масштаб осей (soft iron) ---------------------------------------
        # Считаем только при хорошем охвате: на неполном обороте крайние
        # значения по осям просто не достигнуты, и оценка будет ложной.
        cx_arr = np.asarray(self._xs) - off_x
        cy_arr = np.asarray(self._ys) - off_y
        rad_x = (cx_arr.max() - cx_arr.min()) / 2.0
        rad_y = (cy_arr.max() - cy_arr.min()) / 2.0

        if cov >= self._min_coverage and rad_x > 1e-9 and rad_y > 1e-9:
            avg = (rad_x + rad_y) / 2.0
            scale_x = avg / rad_x
            scale_y = avg / rad_y
            scale_known = True
        else:
            scale_x = scale_y = 1.0
            scale_known = False

        self._report(cov, off_x, off_y, radius,
                     rad_x, rad_y, scale_x, scale_y, scale_known)

        self._save(off_x, off_y, scale_x, scale_y)
        rclpy.shutdown()

    def _report(self, cov, off_x, off_y, radius,
                rad_x, rad_y, scale_x, scale_y, scale_known):
        print('=' * 66)
        print(f'{BOLD}РЕЗУЛЬТАТ{RESET}')
        print('=' * 66)
        print(f'Измерений: {len(self._xs)}, пройдено круга: {cov * 100:.1f}%')
        print()

        ok = True

        # --- полнота оборота -------------------------------------------------
        if cov < self._min_coverage:
            ok = False
            print(f'{RED}[ПЛОХО]{RESET}   Круг пройден не полностью '
                  f'({cov * 100:.0f}%, нужно {self._min_coverage * 100:.0f}%).')
            print('          Сделайте 2-3 полных оборота и повторите.')
        else:
            print(f'{GREEN}[ОК]{RESET}      Круг пройден полностью.')

        # --- величина смещения ------------------------------------------------
        offset_mag = math.hypot(off_x, off_y)
        ratio = offset_mag / radius if radius > 0 else 0.0

        print()
        print(f'          Поле Земли (горизонт.): {radius * UT:6.2f} мкТл')
        print(f'          Смещение нуля:          {offset_mag * UT:6.2f} мкТл '
              f'({ratio:.1f}x от поля)')

        if ratio > 3.0:
            ok = False
            print(f'{RED}[ПЛОХО]{RESET}   Смещение в {ratio:.0f} раз больше '
                  f'поля Земли.')
            print()
            print('          Это НАДО чинить физически, а не в программе.')
            print('          Формально смещение вычитается и курс начнёт')
            print('          считаться, но такая калибровка очень хрупкая:')
            # Наглядный расчёт: почему большое смещение опасно.
            drift = offset_mag * 0.05
            err = math.degrees(math.asin(min(1.0, drift / radius))) \
                if radius > 0 else 90.0
            print(f'          изменение поля робота всего на 5% '
                  f'(это {drift * UT:.1f} мкТл,')
            print(f'          обычное дело при росте тока моторов в повороте)')
            print(f'          сдвинет курс на {err:.0f}°.')
            print()
            print('          ЧТО ДЕЛАТЬ: перенести плату магнитометра дальше')
            print('          от моторов, аккумулятора и силовых проводов —')
            print('          лучше всего на мачту над корпусом, на 30 см и')
            print('          выше. Поле убывает как куб расстояния, поэтому')
            print('          даже 20 лишних сантиметров дают огромный выигрыш.')
        elif ratio > 1.0:
            print(f'{YELLOW}[ТЕРПИМО]{RESET} Смещение больше поля Земли, '
                  f'но компенсируемое.')
            print('          Курс поедет, однако при возможности отнесите')
            print('          магнитометр дальше от силовой части.')
        else:
            print(f'{GREEN}[ОК]{RESET}      Смещение небольшое.')

        # --- форма окружности --------------------------------------------------
        print()
        if scale_known:
            distortion = max(rad_x, rad_y) / min(rad_x, rad_y)
            print(f'          Вытянутость:            {distortion:.2f}')
            if distortion > 1.5:
                print(f'{YELLOW}[ТЕРПИМО]{RESET} Поле заметно искажено '
                      f'железом рядом с датчиком.')
            else:
                print(f'{GREEN}[ОК]{RESET}      Форма близка к правильной.')
        else:
            print(f'{YELLOW}[ПРОПУЩЕНО]{RESET} Масштаб осей не вычислялся: '
                  f'оборот неполный.')
            print('          Значения масштаба оставлены единичными.')

        # --- готовые параметры --------------------------------------------------
        print()
        print('=' * 66)
        print(f'{BOLD}ВСТАВЬТЕ ЭТО В config/mag_calibration.yaml{RESET}')
        print('=' * 66)
        print()
        self._print_params(off_x, off_y, scale_x, scale_y)
        print()
        print('=' * 66)

        if ok:
            print(f'{GREEN}Калибровка удалась.{RESET} Дальше:')
        else:
            print(f'{RED}К результату есть вопросы (см. выше).{RESET}')
            print('Сначала устраните замечания, потом продолжайте. Дальше:')

        print()
        print('  1. Впишите значения в config/mag_calibration.yaml')
        print('  2. Перезапустите робота')
        print('  3. Определите угол монтажа платы:')
        print('       ros2 run robot_navigation heading_check \\')
        print('           --ros-args -p true_azimuth_deg:=<азимут по компасу>')
        print()

    @staticmethod
    def _params_lines(off_x, off_y, scale_x, scale_y):
        return [
            'mag_declination_node:',
            '  ros__parameters:',
            f'    hard_iron_x: {off_x:.10e}',
            f'    hard_iron_y: {off_y:.10e}',
            # Смещение по Z принципиально не определяется вращением вокруг
            # вертикальной оси: при таком вращении Z не меняется, и отделить
            # поле робота от вертикальной составляющей поля Земли (а она на
            # широте 56° около 48 мкТл, втрое больше горизонтальной)
            # невозможно. Подставить сюда измеренное Z означало бы заодно
            # вычесть поле Земли и исказить наклон вектора.
            # На курс это не влияет, пока робот едет по ровному месту.
            '    hard_iron_z: 0.0',
            f'    soft_iron_scale_x: {scale_x:.6f}',
            f'    soft_iron_scale_y: {scale_y:.6f}',
            '    soft_iron_scale_z: 1.0',
        ]

    def _print_params(self, off_x, off_y, scale_x, scale_y):
        for line in self._params_lines(off_x, off_y, scale_x, scale_y):
            print(line)

    def _save(self, off_x, off_y, scale_x, scale_y):
        """Дублирует результат в файл — чтобы не потерять при закрытии окна."""
        try:
            directory = os.path.dirname(os.path.abspath(self._out_path))
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self._out_path, 'w', encoding='utf-8') as f:
                f.write('# Результат калибровки магнитометра.\n')
                f.write('# Скопируйте в robot_navigation/config/'
                        'mag_calibration.yaml\n')
                for line in self._params_lines(off_x, off_y, scale_x, scale_y):
                    f.write(line + '\n')
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
