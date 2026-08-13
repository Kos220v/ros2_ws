#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Диагностика IMU: проверка ориентации осей, магнитометра и углов уровня.

Помогает проверить правильность установки датчика на роботе:
  1. оси акселерометра (плашмя: AZ ~ +9.8, AX/AY ~ 0; наклоны меняют знаки);
  2. гироскоп после калибровки (в покое ~ 0);
  3. магнитометр: выравнивание осей и hard-iron смещение.

Запуск:
    ros2 run robot_odom imu_check                       # живой режим
    ros2 run robot_odom imu_check --heading 35          # проверка осей по компасу телефона

Режим --heading НЕ требует вращения робота:
  * робот стоит неподвижно (любой ориентацией);
  * положите телефон с компасом вдоль оси X робота, запишите азимут H (град);
  * скрипт измерит направление магнитометра, сравнит с H и скажет,
    как переставлены/инвертированы оси X/Y и что поправить в imu_driver.py.

Если робота можно развернуть на 360° — используйте обычный режим и сводку
по Ctrl+C (диапазоны и смещения по каждой оси).
"""

import argparse
import math
import sys
import time

import numpy as np

from robot_odom.imu_driver import HardwareIMU, GRAVITY

RAD2DEG = 180.0 / math.pi


def normalize_deg(a):
    """Приводит угол к диапазону (-180, 180]."""
    a = a % 360.0
    if a > 180.0:
        a -= 360.0
    return a


def acc_level(acc):
    """Углы уровня (roll/pitch) из акселерометра в покое.
    ENU, ось X вперёд, ось Z вверх:
      pitch > 0 — нос поднят (AX > 0),
      roll  > 0 — крен вправо (правый борт вниз).
    """
    ax, ay, az = acc
    pitch = math.atan2(ax, math.hypot(ay, az))
    roll = math.atan2(-ay, math.hypot(ax, az))
    return roll * RAD2DEG, pitch * RAD2DEG


def heading_report(imu, heading, samples=20, interval=0.05):
    """
    Проверка выравнивания осей X/Y магнитометра по одному неподвижному
    положению робота и азимуту его оси X, измеренному компасом телефона.

    heading — азимут (град, по компасу) направления, куда смотрит ось X робота.
    """
    print()
    print(f"Проверка осей магнитометра (робот должен стоять НЕПОДВИЖНО)...")
    mags = []
    for _ in range(samples):
        _, _, mag = imu.get_data()
        mags.append(mag)
        time.sleep(interval)
    mags = np.array(mags)
    m = mags.mean(axis=0)
    spread = mags.std(axis=0).max()

    theta = math.degrees(math.atan2(m[1], m[0])) % 360.0
    delta = normalize_deg(theta - heading)

    print(f"  Азимут оси X робота по телефону : {heading:7.1f}°")
    print(f"  Направление магнитометра        : {theta:7.1f}°")
    print(f"  Расхождение                     : {delta:+7.1f}°")
    print(f"  Разброс замеров (макс, LSB)     : {spread:.0f} "
          "(>50 — робот двигался, повторите)")
    print(f"  Средний вектор (сырые LSB)      : "
          f"X={m[0]:7.1f} Y={m[1]:7.1f} Z={m[2]:7.1f} |M|={np.linalg.norm(m):7.1f}")
    print()

    ad = abs(delta)
    if spread > 50:
        print("  ⚠  Робот двигался во время замера — повторите, удерживая его.")
        return

    if ad <= 15:
        print("  ✔ Оси X/Y магнитометра совпадают с осями робота (X вперёд, Y влево).")
        print("    Небольшое расхождение — это hard-iron смещение/деклинация;")
        print("    для точного курса сделайте калибровку магнитометра.")
    elif abs(ad - 90) <= 15:
        print("  ✘ Оси перепутаны (датчик повёрнут на ~90°).")
        print("    В imu_driver.get_data() замените:  X -> Y,  Y -> -X   (или проверьте знак)")
    elif abs(ad - 180) <= 15:
        print("  ✘ Оси развёрнуты на 180° (X смотрит назад).")
        print("    В imu_driver.get_data() инвертируйте обе оси:  X -> -X,  Y -> -Y")
    else:
        print(f"  ✘ Расхождение {delta:+.0f}° — произвольный поворот датчика в плоскости.")
        print("    Поверните показания магнитометра на величину коррекции:")
        print("        d = math.radians(%.1f)" % (-delta))
        print("        mx, my = mag[0], mag[1]")
        print("        mag[0] = mx * math.cos(d) - my * math.sin(d)")
        print("        mag[1] = mx * math.sin(d) + my * math.cos(d)")
    print()
    print("  Проверка Z: значение MZ должно быть стабильным и не менять знак")
    print("  при лёгком покачивании; знак Z влияет только на компенсацию наклона.")
    print()


def mag_heading_live(imu, refresh=0.1):
    """
    Живой режим: показывает направление магнитометра (азимут по осям робота)
    и текущие MX/MY/MZ при повороте робота.

    Полезен, чтобы понять, почему калибровка даёт странные размахи:
      * при повороте на 360° азимут должен плавно пройти все 360°;
      * MX и MY должны меняться в противофазе с сопоставимой амплитудой.
    """
    print("Поворачивайте робота на 360° по горизонтали, медленно.")
    print("Ожидаемо: ANG проходит 0..360°, MX и MY меняются в противофазе,")
    print("амплитуды X и Y сопоставимы. Ctrl+C — выход.")
    print()
    try:
        while True:
            _, _, mag = imu.get_data()
            m = mag[:2]
            if np.linalg.norm(m) > 1.0:
                ang = math.degrees(math.atan2(m[1], m[0])) % 360.0
            else:
                ang = float('nan')
            line = (f"MX={mag[0]:+7.1f} MY={mag[1]:+7.1f} MZ={mag[2]:+7.1f} "
                    f"| ANG(atan2)= {ang:6.1f}°")
            sys.stdout.write("\r" + line)
            sys.stdout.flush()
            time.sleep(refresh)
    except KeyboardInterrupt:
        print()
        print("Готово.")


def mag_log(imu, interval=0.2):
    """
    Логирование магнитометра построчно (без перезаписи строки).
    Удобно для записи замеров по точкам: поверните робота на 0°,90°,180°,
    270° и копируйте строки. Ctrl+C — выход.
    """
    print("Лог магнитометра (построчно, каждая строка — один замер).")
    print("Поворачивайте робота медленно; Ctrl+C — выход.")
    print("Формат: MX MY MZ ANG(atan2)")
    try:
        while True:
            _, _, mag = imu.get_data()
            m = mag[:2]
            ang = math.degrees(math.atan2(m[1], m[0])) % 360.0 \
                if np.linalg.norm(m) > 1.0 else float('nan')
            print(f"MX={mag[0]:+7.1f} MY={mag[1]:+7.1f} MZ={mag[2]:+7.1f} "
                  f"ANG={ang:6.1f}°", flush=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        print("Готово.")


def stationary_test(imu, duration=15.0, interval=0.25):
    """
    СТАЦИОНАРНЫЙ ТЕСТ: робот стоит неподвижно, проверяем, что драйвер
    корректно читает гироскоп и магнитометр.

    Отвечает на вопрос «почему yaw вращается/скачет, когда робот стоит»:
      * GYRO (град/с) — в покое должен быть ~0 по всем осям. Если по Z есть
        стабильное смещение (например, +40°/с) — калибровка bias была плохой
        (робот двигался/вибрировал при старте), и EKF интегрирует это
        смещение → yaw равномерно вращается;
      * MAG ANG (азимут atan2) — должен СТОЯТЬ на месте (не вращаться).
        Если ANG равномерно «едет» при неподвижном роботе — магнитометр
        читается неверно (байтовый порядок/регистры/сбой I2C) либо рядом
        вращающееся магнитное поле (моторы!);
      * |M| — стабильный модуль поля (без скачков и без аномально малых
        значений — I2C-сбои дают нули).
    """
    print()
    print(f"=== Стационарный тест датчиков ({duration:.0f} сек) ===")
    print("РОБОТ ДОЛЖЕН СТОЯТЬ НЕПОДВИЖНО. МОТОРЫ И ЛИДАР — ВЫКЛЮЧЕНЫ!")
    print("(вибрация и магнитные помехи моторов испортят выводы)")
    print()
    t0 = time.time()
    prev_ang = None
    ang_unwrapped = 0.0        # накопленный дрейф ANG (без учёта перехода 360->0)
    n = 0
    gyr_sum = np.zeros(3)
    gyr_sum2 = np.zeros(3)
    mag_norms = []
    try:
        while time.time() - t0 < duration:
            acc, gyro, mag = imu.get_data()
            n += 1
            gyr_sum += gyro
            gyr_sum2 += gyro * gyro
            mn = float(np.linalg.norm(mag))
            mag_norms.append(mn)
            if mn > 1.0:
                ang = math.degrees(math.atan2(mag[1], mag[0])) % 360.0
                if prev_ang is not None:
                    d = (ang - prev_ang + 180.0) % 360.0 - 180.0
                    ang_unwrapped += d
                prev_ang = ang
            roll, pitch = acc_level(acc)
            sys.stdout.write(
                "\r[%4.1fс] GYR X=%+6.2f Y=%+6.2f Z=%+6.2f °/с | "
                "MAG X=%+6.0f Y=%+6.0f Z=%+6.0f |M|=%6.0f | ANG=%6.1f° | "
                "level r=%+5.1f p=%+5.1f°" % (
                    time.time() - t0,
                    gyro[0] * RAD2DEG, gyro[1] * RAD2DEG, gyro[2] * RAD2DEG,
                    mag[0], mag[1], mag[2], mn, ang if mn > 1.0 else float('nan'),
                    roll, pitch))
            sys.stdout.flush()
            time.sleep(interval)
    except KeyboardInterrupt:
        pass
    print()
    print()

    gyr_mean = gyr_sum / max(n, 1)
    gyr_std = np.sqrt(np.maximum(gyr_sum2 / max(n, 1) - gyr_mean * gyr_mean, 0.0))
    gz_mean = gyr_mean[2] * RAD2DEG
    gz_std = gyr_std[2] * RAD2DEG
    gxy_max = max(abs(gyr_mean[0]), abs(gyr_mean[1])) * RAD2DEG
    mag_norms = np.array(mag_norms) if mag_norms else np.array([0.0])
    m_min, m_max = float(mag_norms.min()), float(mag_norms.max())
    m_spread = (m_max - m_min) / max(m_min, 1.0) * 100.0

    print("=== Итог стационарного теста ===")
    print(f"  Гироскоп Z: среднее {gz_mean:+.2f} °/с, разброс {gz_std:.2f} °/с "
          f"(X/Y среднее: {gxy_max:.2f} °/с)")
    print(f"  Магнитометр: |M| от {m_min:.0f} до {m_max:.0f} LSB "
          f"(разброс {m_spread:.0f}%)")
    print(f"  Магнитный курс ANG: дрейф за тест {ang_unwrapped:+.1f}° "
          f"(должен быть ~0)")
    print()

    problems = 0

    if abs(gz_mean) > 2.0:
        problems += 1
        print(f"  ✘ ГИРОСКОП: по Z смещение {gz_mean:+.2f} °/с — это и есть "
              "источник вращения yaw.")
        print("    Причина: калибровка bias была плохой (робот двигался или "
              "вибрировал при старте, лидар/моторы были включены).")
        print("    Решение: перезапустите ноду при НЕПОДВИЖНОМ роботе, "
              "лидар/моторы выключены; проверьте калибровку:")
        print("      ros2 run robot_odom imu_check --calibrate-gyro")
    elif gz_std > 2.0:
        problems += 1
        print(f"  ⚠ ГИРОСКОП: разброс по Z {gz_std:.2f} °/с — вибрация/помехи.")
        print("    Убедитесь, что лидар и моторы выключены во время теста.")
    else:
        print("  ✔ ГИРОСКОП: в покое ~0 — чтение корректно, калибровка bias ок.")

    if imu.mag_type is None:
        problems += 1
        print("  ✘ МАГНИТОМЕТР: не найден на шине (QMC5883L=0x0D, HMC5883L=0x1E).")
        print("    При включённом use_magnetometer курс остаётся чисто гироскопным")
        print("    и дрейфует. Проверьте: i2cdetect -y 1, провода, адрес.")
    elif m_min < 100.0:
        problems += 1
        print(f"  ✘ МАГНИТОМЕТР: |M| = {m_min:.0f} LSB — подозрительно мало "
              "(похоже на сбой I2C: чтение даёт нули).")
        print("    Проверьте провода/пайку/адрес (i2cdetect -y 1).")
    elif abs(ang_unwrapped) > 8.0:
        problems += 1
        print(f"  ✘ МАГНИТОМЕТР: магнитный курс вращается на "
              f"{ang_unwrapped:+.1f}° за тест при неподвижном роботе!")
        print("    Причины: 1) неверное чтение регистров/порядок байт в "
              "imu_driver; 2) вращающееся магнитное поле рядом (моторы/динамики);")
        print("    3) датчик физически вращается (жгут проводов?).")
        print("    Проверьте поточечно: ros2 run robot_odom imu_check --mag-log")
    else:
        print("  ✔ МАГНИТОМЕТР: курс стабилен при неподвижном роботе — "
              "чтение корректно.")

    if m_spread > 30.0:
        problems += 1
        print(f"  ⚠ МАГНИТОМЕТР: |M| скачет ({m_spread:.0f}%) — помехи от "
              "моторов/питания или сбои I2C.")
    elif imu.mag_type is not None:
        print(f"  ✔ МАГНИТОМЕТР: |M| стабилен (разброс {m_spread:.0f}%).")

    print()
    if problems == 0:
        print("  ВЫВОД: датчики читаются правильно. Проблема — в обработке/")
        print("  слиянии внутри odom_node (EKF/фильтры) — смотрите логи ноды.")
    else:
        print(f"  ВЫВОД: найдено проблем: {problems}. Исправьте их и повторите тест.")
    print()


def calibrate_gyro(imu, samples=200, interval=0.01):
    """
    Калибровка гироскопа с контролем неподвижности.

    ВАЖНО: лидар/моторы должны быть ВЫКЛЮЧЕНЫ (вибрация портит bias!).
    Собирает сэмплы в покое, проверяет разброс, печатает bias.
    """
    print("Калибровка гироскопа. УБЕДИТЕСЬ:")
    print("  * робот неподвижен, лежит плашмя;")
    print("  * ЛИДАР И МОТОРЫ ВЫКЛЮЧЕНЫ (вибрация портит bias).")
    print(f"Собираю {samples} сэмплов...")
    gyros = []
    for _ in range(samples):
        _, g, _ = imu.get_data()
        gyros.append(g)
        time.sleep(interval)
    gyros = np.array(gyros)
    bias = gyros.mean(axis=0)
    std = gyros.std(axis=0)
    print()
    print(f"Bias гироскопа (рад/с): {np.round(bias, 5)}")
    print(f"  = {np.round(bias * RAD2DEG, 3)} °/с")
    print(f"Разброс (рад/с): {np.round(std, 5)}")
    print(f"  = {np.round(std * RAD2DEG, 3)} °/с")
    print()
    if std.max() > 0.05:
        print("⚠ Высокий разброс — робот двигался или вибрация. Повторите")
        print("  при выключенных лидаре/моторах.")
    elif abs(bias[2]) > 0.01:
        print("⚠ Bias по Z > 0.01 рад/с (0.57°/с) — дрейф будет заметен.")
        print("  Повторите при выключенном лидаре/моторах.")
    else:
        print("✔ Калибровка хорошая — дрейф будет мал.")
    print()
    print("Нода сама калибрует гироскоп при старте (calibration_samples).")
    print("Убедитесь, что при запуске стека лидар стартует ПОЗЖЕ калибровки.")


def calibrate_mag(imu, duration=40.0, interval=0.02):
    """
    Полная калибровка магнитометра (hard-iron + soft-iron).

    Пользователь вращает робота на 360° в горизонтальной плоскости
    (и, по возможности, наклоняет вперёд/назад/вбок — для калибровки Z)
    в течение duration секунд. Скрипт собирает min/max по осям и печатает
    готовые параметры для odom_node.
    """
    print("=== Калибровка магнитометра ===")
    print(f"Вращайте робота на 360° в течение {duration:.0f} сек:")
    print("  * медленно, плавно, без рывков;")
    print("  * старайтесь покрыть полный круг по горизонтали;")
    print("  * по возможности наклоняйте вперёд/назад/в стороны —")
    print("    это откалибрует ось Z.")
    print("  Ctrl+C — завершить досрочно.")
    print()
    samples = []
    import time as _t
    t0 = _t.time()
    try:
        while _t.time() - t0 < duration:
            _, _, mag = imu.get_data()
            # Отбрасываем замеры со сбоем чтения: магнитометр не может дать
            # ровно 0 при таком поле — это ошибка I2C (read_word_2c -> 0).
            if np.any(mag == 0.0) or np.linalg.norm(mag) < 100.0:
                continue
            samples.append(mag)
            _t.sleep(interval)
    except KeyboardInterrupt:
        pass
    mags = np.array(samples)
    if len(mags) < 100:
        print("⚠ Слишком мало данных — повторите калибровку.")
        return
    # --- центр окружности в плоскости XY методом наименьших квадратов ---
    # Точнее, чем (min+max)/2: устойчив к неполному кругу и шуму.
    x = mags[:, 0]
    y = mags[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    b = x * x + y * y
    coef, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, cc = float(coef[0]), float(coef[1]), float(coef[2])
    R = math.sqrt(max(cc + cx * cx + cy * cy, 0.0))
    radii = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    mn = mags.min(axis=0)
    mx = mags.max(axis=0)
    offset_mm = (mn + mx) / 2.0
    span = mx - mn
    max_span = span.max()
    scale = np.ones(3) if max_span <= 0 else max_span / span
    # Используем МНК-центр для XY (точнее), min/max — для Z:
    offset = np.array([cx, cy, offset_mm[2]])
    # Предупреждение о некачественной калибровке: если размахи осей
    # отличаются сильно, датчик установлен перекошено или вращение было
    # неполным/по наклонной поверхности.
    ratio = max_span / max(span.min(), 1.0)
    r_std = float(radii.std())
    print()
    print("=== Результат калибровки ===")
    print(f"  Данных: {len(mags)} замеров")
    print(f"  Диапазоны: X {mn[0]:.0f}..{mx[0]:.0f}  "
          f"Y {mn[1]:.0f}..{mx[1]:.0f}  Z {mn[2]:.0f}..{mx[2]:.0f}")
    print(f"  Центр окружности XY (МНК): X {cx:+.0f}  Y {cy:+.0f}  R≈{R:.0f}")
    print(f"  Центр по min/max:          X {offset_mm[0]:+.0f}  "
          f"Y {offset_mm[1]:+.0f}  Z {offset_mm[2]:+.0f}")
    print(f"  Качество (разброс R): {r_std:.0f} LSB "
          f"({radii.min():.0f}..{radii.max():.0f}) — чем меньше, тем круглее")
    print(f"  Hard-iron (offset): X {offset[0]:+.0f}  "
          f"Y {offset[1]:+.0f}  Z {offset[2]:+.0f}")
    print(f"  Soft-iron (scale):  X {scale[0]:.3f}  "
          f"Y {scale[1]:.3f}  Z {scale[2]:.3f}")
    if ratio > 3.0:
        print()
        print(f"  ⚠ ВНИМАНИЕ: размах осей отличается в {ratio:.1f} раз —")
        print("    калибровка НЕПРИГОДНА. Возможные причины:")
        print("    1) робот вращался не по горизонтали (неровный пол/наклон);")
        print("    2) плата компаса в корпусе GPS стоит НЕ параллельно палубе")
        print("       робота (перекошена/вертикально) — ось чувствительности")
        print("       направлена не туда;")
        print("    3) вращение было неполным или рывками.")
        print("    НЕ применяйте эти scale-коэффициенты — они усилят шум")
        print("    в 9-33 раза. Проверьте монтаж компаса и повторите")
        print("    калибровку на ровном полу, 2-3 полных круга.")
        print("    Если размахи снова отличаются в разы — компас физически")
        print("    перекошен, оставьте use_magnetometer:=false (курс от гироскопа).")
        return
    print()
    print("  Пропишите в launch odom_node:")
    print(f"    'mag_hard_iron_x': {offset[0]:.1f},")
    print(f"    'mag_hard_iron_y': {offset[1]:.1f},")
    print(f"    'mag_hard_iron_z': {offset[2]:.1f},")
    print(f"    'mag_scale_x': {scale[0]:.3f},")
    print(f"    'mag_scale_y': {scale[1]:.3f},")
    print(f"    'mag_scale_z': {scale[2]:.3f},")
    print("    'use_magnetometer': True,")
    print()
    print("  После этого проверьте курс:")
    print("    ros2 run robot_odom imu_check --heading <азимут_телефона>")
    print("  Расхождение скомпенсируйте параметром mag_yaw_offset_deg")
    print("  (прибавляйте/убавляйте, пока --heading не покажет ~0°).")


def calibrate_mount(imu, samples=50, interval=0.05):
    """
    Калибровка монтажного наклона платы IMU.

    Робот должен стоять на ГОРИЗОНТАЛЬНОЙ поверхности (проверьте уровнем!).
    По среднему вектору ускорения вычисляется наклон платы относительно
    корпуса, печатаются параметры для odom_node.
    """
    print("Калибровка монтажного наклона IMU.")
    print("УБЕДИТЕСЬ, что робот стоит на ГОРИЗОНТАЛЬНОЙ поверхности")
    print("(проверьте пузырьковым уровнем) и НЕПОДВИЖЕН...")
    accs = []
    for _ in range(samples):
        a = imu.read_raw_acc()   # СЫРОЕ ускорение (до bias-коррекции) —
        accs.append(a)           # иначе уровень уже «выпрямлен» калибровкой
        time.sleep(interval)
    accs = np.array(accs)
    a = accs.mean(axis=0)
    g = np.linalg.norm(a)
    # Точные формулы для R = Rz(yaw)*Ry(pitch)*Rx(roll):
    #   ax = -g*sin(pitch), ay = g*sin(roll)*cos(pitch), az = g*cos(roll)*cos(pitch)
    pitch = math.degrees(math.atan2(-a[0], math.hypot(a[1], a[2])))
    roll = math.degrees(math.atan2(a[1], a[2]))
    print()
    print(f"ACC среднее: X={a[0]:+.2f} Y={a[1]:+.2f} Z={a[2]:+.2f}  |g|={g:.2f}")
    print()
    if abs(g - GRAVITY) > 0.5:
        print(f"⚠ |g|={g:.2f} ≠ 9.81 — робот двигался или поверхность неровная.")
        print("  Повторите калибровку, удерживая робота неподвижно на ровном полу.")
        return
    if abs(roll) < 0.3 and abs(pitch) < 0.3:
        print("✔ Плата IMU установлена ровно (наклон < 0.3°), компенсация не нужна.")
        return
    print("Наклон платы IMU обнаружен. Задайте в launch odom_node:")
    print()
    print(f"    'imu_mount_roll_deg': {roll:.1f},")
    print(f"    'imu_mount_pitch_deg': {pitch:.1f},")
    print()
    print("После этого ACC при ровном роботе станет X≈0 Y≈0 Z≈+9.81")
    print("и ось Z odom в RViz встанет вертикально.")
    print("Если наклон был из-за неровного пола — после выравнивания робота")
    print("углы станут ~0 и параметры можно убрать.")


def live_loop(imu, refresh):
    """Живой режим: показывает ACC/GYR/MAG и углы уровня в реальном времени."""
    mag_min = np.full(3, np.inf)
    mag_max = np.full(3, -np.inf)
    mag_norm_min = np.inf
    mag_norm_max = -np.inf

    print("Что проверяем (робот в руках или на столе):")
    print("  1. Плашмя z-вверх:  AZ ~ +9.8, AX ~ 0, AY ~ 0, |ACC| ~ 9.8")
    print("  2. Поднять нос:     AX > 0    |   опустить нос:     AX < 0")
    print("  3. Крен вправо:     AY < 0    |   крен влево:       AY > 0")
    print("  4. Гироскоп в покое должен быть ~ 0 (после калибровки)")
    print("  5. Магнитометр: вращайте робот на 360° в горизонтальной плоскости:")
    print("     * MX максимален, когда ось X датчика смотрит на СЕВЕР;")
    print("     * MY максимален, когда ось X смотрит на ВОСТОК;")
    print("     * |MAG| должен меняться умеренно (эллипс), без резких скачков.")
    print("  Ctrl+C — выход; в конце печатается сводка по магнитометру.")
    print()

    frames = 0
    try:
        while True:
            acc, gyro, mag = imu.get_data()
            frames += 1

            mag_min = np.minimum(mag_min, mag)
            mag_max = np.maximum(mag_max, mag)
            m_norm = np.linalg.norm(mag)
            mag_norm_min = min(mag_norm_min, m_norm)
            mag_norm_max = max(mag_norm_max, m_norm)

            roll, pitch = acc_level(acc)
            gyr_dps = gyro * RAD2DEG

            line = (
                f"ACC X={acc[0]:+7.2f} Y={acc[1]:+7.2f} Z={acc[2]:+7.2f} "
                f"|A|={np.linalg.norm(acc):5.2f}  "
                f"LEVEL roll={roll:+6.1f} pitch={pitch:+6.1f}  |  "
                f"GYR X={gyr_dps[0]:+6.2f} Y={gyr_dps[1]:+6.2f} "
                f"Z={gyr_dps[2]:+6.2f} deg/s  |  "
                f"MAG X={mag[0]:+6.0f} Y={mag[1]:+6.0f} Z={mag[2]:+6.0f} "
                f"|M|={m_norm:6.0f}"
            )
            sys.stdout.write("\r" + line)
            sys.stdout.flush()

            if frames % 25 == 0:
                sys.stdout.write(
                    "\n  [min/max] MAG X: %.0f..%.0f  Y: %.0f..%.0f  Z: %.0f..%.0f"
                    "  |M|: %.0f..%.0f\n" % (
                        mag_min[0], mag_max[0], mag_min[1], mag_max[1],
                        mag_min[2], mag_max[2], mag_norm_min, mag_norm_max))

            time.sleep(refresh)

    except KeyboardInterrupt:
        print()
        print()
        if imu.mag_type is not None and np.isfinite(mag_min).all():
            print("=== Сводка магнитометра (hard/soft-iron диагностика) ===")
            offset = (mag_min + mag_max) / 2.0
            span = mag_max - mag_min
            print(f"  Диапазоны: X {mag_min[0]:.0f}..{mag_max[0]:.0f}  "
                  f"Y {mag_min[1]:.0f}..{mag_max[1]:.0f}  "
                  f"Z {mag_min[2]:.0f}..{mag_max[2]:.0f}")
            print(f"  Смещение (hard-iron): X {offset[0]:+.0f}  "
                  f"Y {offset[1]:+.0f}  Z {offset[2]:+.0f}")
            print(f"  Размах (soft-iron):   X {span[0]:.0f}  "
                  f"Y {span[1]:.0f}  Z {span[2]:.0f}")
            print(f"  |M|: {mag_norm_min:.0f}..{mag_norm_max:.0f} "
                  f"(отношение max/min = {mag_norm_max / mag_norm_min:.2f})")
            print()
            print("  Оценка:")
            print("  * Смещение X/Y в сотни LSB — постоянный магнитный фон")
            print("    (hard-iron, моторы/динамики рядом). Компенсируйте вычитанием")
            print("    offset в imu_driver.get_data() перед публикацией.")
            print("  * Отношение |M| max/min > 1.3 — soft-iron (искажение полем")
            print("    железа). Для точного курса нужна полная калибровка.")
            print()
            print("  Сверка осей магнитометра с роботом:")
            print("  * ось X на север  -> MX ~ максимум (наиболее положителен)")
            print("  * ось X на восток -> MY ~ максимум")
            print("  Если знаки зеркальны/перепутаны — поправьте оси в imu_driver.get_data().")
        else:
            print("Магнитометр не найден — сводка недоступна.")
        print()


def mag_scan():
    """Скан WHO_AM_I / ID регистров магнитометров по известным адресам.

    Помогает определить, какой компас реально стоит в GPS-модуле
    (GEP-M10 и т.п.): QMC5883L/HMC5883L/IST8310/QMC7983/AK8963.
    """
    import smbus2
    print("=== Скан магнитометров (WHO_AM_I / ID) ===")
    print("Адрес  Регистр  Ожидание   Прочитано  ->  Чип")
    cands = [
        (0x0D, 0x00, 0xFF, 'QMC5883L'),
        (0x0E, 0x00, 0x10, 'IST8310'),
        (0x2C, 0x00, 0x8B, 'QMC7983'),
        (0x0C, 0x00, 0x48, 'AK8963'),
        (0x1E, 0x0A, 0x48, 'HMC5883L'),
        (0x1C, 0x0F, 0x3D, 'LIS3MDL'),
        (0x30, 0x2F, 0x30, 'MMC5983MA'),
    ]
    try:
        bus = smbus2.SMBus(1)
    except Exception as e:
        print(f"  Не удалось открыть I2C-1: {e}")
        return
    found = None
    for addr, reg, exp, name in cands:
        try:
            val = bus.read_byte_data(addr, reg)
            mark = 'OK' if val == exp else '  '
            print(f"  0x{addr:02X}   0x{reg:02X}    0x{exp:02X}      0x{val:02X}    {mark} {name}")
            if val == exp:
                found = (name, addr)
        except Exception:
            print(f"  0x{addr:02X}   0x{reg:02X}    0x{exp:02X}      --        (нет ответа)")
    print()
    if found:
        print(f"  ИТОГ: найден {found[0]} на адресе 0x{found[1]:02X}")
        print("  Если это не QMC5883L/HMC5883L — драйвер уже умеет его читать")
        print("  (IST8310/QMC7983/AK8963), пересоберите robot_odom.")
    else:
        print("  ИТОГ: знакомый магнитометр не найден.")
        print("  Проверьте i2cdetect -y 1 и маркировку GPS-модуля.")


def main():
    parser = argparse.ArgumentParser(
        description="Проверка ориентации IMU и магнитометра")
    parser.add_argument("--samples", type=int, default=50,
                        help="сэмплов автокалибровки (по умолчанию 50)")
    parser.add_argument("--refresh", type=float, default=0.2,
                        help="период обновления вывода, с (по умолчанию 0.2)")
    parser.add_argument("--heading", type=float, default=None, metavar="ГРАД",
                        help="азимут оси X робота по компасу телефона, град. "
                             "Проверка осей магнитометра БЕЗ вращения робота.")
    parser.add_argument("--mag-offset", type=float, default=0.0, metavar="ГРАД",
                        help="та же поправка осей магнитометра, что и параметр "
                             "mag_yaw_offset_deg ноды (для проверки фикса).")
    parser.add_argument("--calibrate-mount", action="store_true",
                        help="калибровка монтажного наклона платы IMU: робот "
                             "стоит ровно, скрипт выдаёт imu_mount_roll_deg/pitch_deg")
    parser.add_argument("--calibrate-mag", type=float, nargs="?", const=40.0,
                        metavar="СЕК",
                        help="полная калибровка магнитометра (hard/soft-iron): "
                             "вращать робота на 360° указанное число секунд "
                             "(по умолчанию 40); Ctrl+C — досрочно")
    parser.add_argument("--calibrate-gyro", action="store_true",
                        help="калибровка гироскопа в покое (лидар/моторы ВЫКЛ)")
    parser.add_argument("--mag-heading-live", action="store_true",
                        help="живой режим: направление магнитометра (азимут) "
                             "при повороте робота — диагностика размахов")
    parser.add_argument("--mag-log", action="store_true",
                        help="лог магнитометра построчно (по точкам: 0/90/180/270°)")
    parser.add_argument("--stationary", type=float, nargs="?", const=15.0,
                        metavar="СЕК",
                        help="СТАЦИОНАРНЫЙ тест: робот стоит неподвижно, "
                             "проверяем чтение гироскопа и магнитометра "
                             "(по умолчанию 15 сек); моторы/лидар ВЫКЛ")
    parser.add_argument("--mag-scan", action="store_true",
                        help="скан WHO_AM_I/ID магнитометров (определить чип "
                             "компаса в GPS-модуле)")
    parser.add_argument("--mag-z-invert", action="store_true",
                        help="как параметр mag_z_invert ноды: инвертировать "
                             "ось Z магнитометра (QMC выдаёт MZ «вниз»)")
    parser.add_argument("--hard-iron", type=str, default=None, metavar="X Y Z",
                        help="компенсация hard-iron в LSB (например: '2752 3375 1487')")
    parser.add_argument("--scale", type=str, default=None, metavar="X Y Z",
                        help="масштаб осей soft-iron (например: '1 1 1')")
    args = parser.parse_args()

    print("Инициализация IMU и автокалибровка... "
          "(держите робота неподвижно плашмя)")
    hi = [float(v) for v in args.hard_iron.split()] if args.hard_iron else [0, 0, 0]
    sc = [float(v) for v in args.scale.split()] if args.scale else [1, 1, 1]
    if args.mag_scan:
        mag_scan()
        return 0

    imu = HardwareIMU(bus_num=1,
                      mag_yaw_offset_deg=args.mag_offset,
                      mag_z_invert=args.mag_z_invert,
                      mag_hard_iron_x=hi[0], mag_hard_iron_y=hi[1], mag_hard_iron_z=hi[2],
                      mag_scale_x=sc[0], mag_scale_y=sc[1], mag_scale_z=sc[2])
    imu.calibrate(samples=args.samples)
    print("Магнитометр:", imu.mag_type or "не найден")
    print()

    if args.stationary:
        stationary_test(imu, duration=args.stationary)
        return 0

    if args.mag_log:
        mag_log(imu, interval=max(args.refresh, 0.1))
        return 0

    if args.mag_heading_live:
        mag_heading_live(imu, refresh=args.refresh)
        return 0

    if args.calibrate_mag:
        calibrate_mag(imu, duration=args.calibrate_mag)
        return 0

    if args.calibrate_mount:
        calibrate_mount(imu, samples=args.samples)
        return 0

    if args.calibrate_gyro:
        calibrate_gyro(imu, samples=args.samples)
        return 0

    if args.heading is not None:
        if imu.mag_type is None:
            print("Магнитометр не найден — проверка курса невозможна.")
            return 1
        heading_report(imu, args.heading)
        return 0

    live_loop(imu, args.refresh)
    return 0


if __name__ == "__main__":
    sys.exit(main())
