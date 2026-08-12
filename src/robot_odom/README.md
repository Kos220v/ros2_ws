# robot_odom — ROS 2 пакет одометрии и IMU

Нода `odom_node` для Raspberry Pi:

- **MPU6050** (акселерометр + гироскоп) по I2C-1 (адрес `0x68`);
- **QMC5883L** (`0x0D`) или **HMC5883L** (`0x1E`) — магнитометр, опционально;
- фильтрация ориентации — `ahrs.filters.EKF` (ENU, REP-103);
- **автокалибровка при старте**: смещение гироскопа (убирает дрейф yaw) и,
  если робот лежит плашмя, смещение акселерометра.

## Структура

```
robot_odom/
├── package.xml              # манифест (format 3)
├── resource/robot_odom      # маркер пакета для ament
├── robot_odom/
│   ├── imu_driver.py        # драйвер IMU (без rclpy)
│   ├── imu_check.py         # диагностика ориентации и магнитометра
│   └── odom_node.py         # нода
├── setup.py                 # entry points: odom_node, imu_check
├── setup.cfg
└── test/
    └── test_imu.py          # pytest: автокалибровка на эмулированной I2C-шине
```

## Темы и TF

| Что | Тип | Описание |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | ориентация, угловая скорость, ускорение |
| `/odom` | `nav_msgs/Odometry` | позиция из `joint_states` + курс от EKF |
| `/joint_states` | `sensor_msgs/JointState` (подписка) | позиции суставов X/Y |
| TF | `odom → base_link` | если `publish_tf := true` |

## Установка зависимостей

```bash
# На Raspberry Pi (Ubuntu + ROS 2):
sudo apt install python3-smbus i2c-tools
pip install smbus2 ahrs        # numpy обычно уже есть (python3-numpy)
sudo usermod -aG i2c $USER     # доступ к /dev/i2c-1 (перелогиниться!)
```

Проверка железа:

```bash
i2cdetect -y 1
# 0x68 — MPU6050 (обязательно)
# 0x0D — QMC5883L или 0x1E — HMC5883L (опционально)
```

## Сборка и запуск

```bash
mkdir -p ~/ros2_ws/src
cp -r robot_odom ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select robot_odom
source install/setup.bash

# Запуск ноды:
ros2 run robot_odom odom_node
```

Параметры можно переопределить прямо в командной строке:

```bash
ros2 run robot_odom odom_node --ros-args \
    -p robot_x_joint:=robot_x \
    -p robot_y_joint:=robot_y \
    -p imu_rate:=50.0
```

Или напрямую без сборки (файл лежит в корне репозитория):

```bash
python3 odom_node.py --ros-args -p robot_x_joint:=robot_x -p robot_y_joint:=robot_y
```

> ⚠️ **Перед запуском положите робота плашмя и не двигайте его** — идёт
> автокалибровка (`calibration_samples` сэмплов, ~1 с при 100 сэмплах).

## Параметры

| Параметр | По умолчанию | Описание |
|---|---|---|
| `robot_x_joint` | `"robot_x"` | имя сустава по оси X |
| `robot_y_joint` | `"robot_y"` | имя сустава по оси Y |
| `odom_frame` | `"odom"` | кадр одометрии |
| `base_frame` | `"base_link"` | кадр робота |
| `publish_tf` | `true` | публиковать TF |
| `imu_rate` | `50.0` | частота опроса IMU, Гц |
| `ekf_frame` | `"ENU"` | конвенция кадра EKF (REP-103) |
| `calibration_samples` | `100` | сэмплов автокалибровки |

## Тесты

```bash
# в рабочем пространстве (colcon):
colcon test --packages-select robot_odom

# или локально, без ROS 2 (тест не зависит от rclpy):
cd ~/ros2_ws/src/robot_odom
pytest test/test_imu.py
```

## Проверка ориентации IMU и магнитометра

Запустите диагностику (требует доступа к I2C; робота держите в руках или на столе):

```bash
ros2 run robot_odom imu_check            # после colcon build
# или прямо из src:
cd ~/ros2_ws/src/robot_odom && python3 -m robot_odom.imu_check
```

Она покажет в реальном времени ACC/GYR/MAG, углы уровня (roll/pitch) и по Ctrl+C
— сводку по магнитометру. Дальше проверяйте по шагам:

### 1. Оси акселерометра

| Положение | Ожидание |
|---|---|
| Плашмя, z-вверх | `AZ ~ +9.8`, `AX ≈ AY ≈ 0`, `|ACC| ~ 9.8` |
| Нос вверх | `AX > 0` |
| Нос вниз | `AX < 0` |
| Крен вправо | `AY < 0` |
| Крен влево | `AY > 0` |
| Вверх ногами | `AZ ~ -9.8` |

Если знаки зеркальны — датчик установлен «вверх ногами» (инвертируйте оси);
если «нос» и «крен» перепутаны местами — датчик повёрнут на 90° (поменяйте X↔Y).

### 2. Гироскоп

После калибровки в покое все значения должны быть ~0. Если дрейф заметен
(>0.1 °/с) — повторите калибровку, удерживая робота неподвижно.

### 3. Магнитометр (основная проверка)

Вращайте робота на 360° в горизонтальной плоскости (лежит на столе):

* **Оси**: направьте ось X датчика на СЕВЕР — `MX` должен быть максимальным
  (наиболее положительным); на ВОСТОК — максимален `MY`.
  Если максимумы не совпадают с этими направлениями — оси магнитометра
  перепутаны/инвертированы относительно MPU6050 (частая беда QMC5883L на
  платах GY-87/GY-271) — поправьте в `imu_driver.py` → `get_data()`.
* **Hard-iron**: после Ctrl+C посмотрите сводку — смещение (offset) по X/Y
  в сотни LSB означает постоянный магнитный фон (моторы, динамики).
  Компенсируется вычитанием offset перед публикацией.
* **Soft-iron**: отношение `|M| max/min > 1.3` — поле искажается железом;
  для точного курса нужна полная калибровка (окружность по X/Y).

### 4. Курс EKF (итоговая проверка)

```bash
ros2 run robot_odom odom_node &
ros2 topic echo /imu/data --once          # посмотрите orientation
```

Поверните робота на 90° вправо (по часовой, если смотреть сверху) — yaw из
кватерниона должен увеличиться примерно на 90°. Если направление знака
обратное — инвертируйте ось Z гироскопа или магнитометра.

## Устранение проблем

**Нода не запускается, в терминале только `[XMLPARSER Error] realpath failed ... loadDefaultXMLFile`**

Это безобидное сообщение от Fast DDS (стандартный RMW в ROS 2): он ищет
дефолтный XML-файл профилей и не находит. Оно печатается ДО вашего кода, во
время `rclpy.init()`, и на работу ноды не влияет.

Если после него нет НИЧЕГО (ни трейсбека, ни сообщений калибровки) — процесс
умирает на уровне DDS, это частая беда Fast DDS на Raspberry Pi/ARM. Лечится
переключением на CycloneDDS:

```bash
echo $ROS_DISTRO            # humble / jazzy / ...
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
ros2 daemon stop
```

Проверка, что дело именно в среде, а не в пакете:

```bash
ros2 run demo_nodes_py talker     # та же XMLPARSER-строка? значит, дело в DDS
```

**Нода падает сразу после `XMLPARSER Error` с трейсбеком** — смотрите текст
ошибки:

* `ModuleNotFoundError: No module named 'ahrs' / 'smbus2'` → `pip install ahrs smbus2`
* `RuntimeError: Не удалось открыть шину I2C...` → включайте I2C и права, как
  написано в сообщении (см. ниже)
* `SyntaxError` / `UnicodeDecodeError` при импорте → файл перекодирован в
  CP1251, лечите `iconv` (см. ниже)

Чтобы увидеть настоящий трейсбек в обход `ros2 run`:

```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/robot_odom/robot_odom/odom_node.py
```

**`RuntimeError: Не удалось открыть шину I2C-1`**

```bash
sudo raspi-config            # Interface Options -> I2C -> Enable
sudo usermod -aG i2c $USER   # затем перелогиниться
i2cdetect -y 1               # должен появиться адрес 0x68 (MPU6050)
```

**`colcon build`: `'utf-8' codec can't decode byte 0xed ... invalid continuation byte`**

Файлы пакета в UTF-8, а `package.xml` перекодировался в Windows-1251 при
копировании (0xED — это «н» в CP1251). `package.xml` и `setup.py` теперь чисто
ASCII, но проверьте кодировку копии на своём ПК:

```bash
# Проверка: команда должна молча завершиться без ошибок
python3 -c "open('package.xml', encoding='utf-8').read()"
python3 -c "open('robot_odom/odom_node.py', encoding='utf-8').read()"

# или с утилитой file:
file package.xml robot_odom/odom_node.py
# ожидаемо: "UTF-8 Unicode text" или "ASCII text"; если пишет
# "Non-ISO extended-ASCII text" — файл перекодирован, лечим так:
iconv -f CP1251 -t UTF-8 robot_odom/odom_node.py > /tmp/fix.py && mv /tmp/fix.py robot_odom/odom_node.py
iconv -f CP1251 -t UTF-8 test/test_imu.py > /tmp/fix.py && mv /tmp/fix.py test/test_imu.py
```

Причина обычно в способе переноса файлов (FTP в режиме ASCII, редактор с
локалью `ru_RU.CP1251`, копирование текста через терминал). Лучше всего
переносить архивом (`tar czf`/`zip`) или `scp`/`rsync` — они сохраняют байты как есть.

## Известные ограничения

- **Оси магнитометра.** На платах GY-87/GY-271 QMC5883L часто повёрнут на 90°
  относительно MPU6050 — проверьте выравнивание осей в `get_data()`, иначе курс
  будет смещён. Капсулирование (hard/soft-iron) магнитометра не делается —
  при необходимости добавьте сами.
- **Калибровка только при старте.** Для полной точности добавьте калибровку
  температурного дрейфа гироскопа в рантайме.
- **`time.time()` в `_on_joint_states`.** Если используете симуляционное время
  (Gazebo), замените на `self.get_clock().now().nanoseconds / 1e9`.
