# Подключение STM32 IMU к Raspberry Pi через USB-UART

Эта инструкция предназначена для платы STM32F303 с новым IMU-мостом
`imu_stm32_bridge`, подключённой к Raspberry Pi через внешний USB-UART
преобразователь.

## 1. Электрическое подключение

USB-UART должен работать в режиме **3,3 В TTL**. Это не RS-232 и не
RS-485.

| USB-UART | STM32F303 | Назначение |
|---|---|---|
| `TXD` | `RX` платы | данные от Raspberry Pi к STM32 |
| `RXD` | `TX` платы | данные от STM32 к Raspberry Pi |
| `GND` | `GND` | общий провод |

Правила подключения:

- `TX` и `RX` подключаются крест-накрест.
- Обязательно соединить `GND`.
- Не подавать 5 В на входы STM32.
- Не подключать одновременно несколько источников питания к плате IMU.
- Если на USB-UART есть переключатель `3.3V/5V`, установить `3.3V`.
- Перед подключением проверить распиновку конкретного адаптера: маркировка
  `VCC` не всегда означает безопасное напряжение логических уровней.

Питание от USB-UART можно использовать только если это предусмотрено схемой
платы IMU. Для первого запуска безопаснее питать STM32 отдельно, а от
USB-UART подключить только `TXD`, `RXD` и `GND`.

## 2. Узнать, какой порт получил адаптер

Подключить USB-UART к Raspberry Pi и выполнить:

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
dmesg --follow
```

После подключения в `dmesg` появится один из типичных драйверов:

- `ch341` или `ch343` — обычно порт `/dev/ttyUSB0`, VID:PID `1a86:7523`;
- `cp210x` — обычно `/dev/ttyUSB0`, VID:PID `10c4:ea60`;
- `ftdi_sio` — обычно `/dev/ttyUSB0`, VID:PID `0403:6001`;
- `cdc_acm` — обычно `/dev/ttyACM0`.

Номер не следует считать постоянным: после переподключения, добавления
лидара или перезагрузки Raspberry Pi устройство может стать `ttyUSB1`.
Именно поэтому для робота используется постоянная ссылка `/dev/imu_stm32`.

Точные идентификаторы подключённого устройства можно получить так:

```bash
udevadm info --query=property --name=/dev/ttyUSB0 \
  | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_VENDOR|ID_MODEL'
```

Если устройство определилось как `/dev/ttyACM0`, подставить этот путь вместо
`/dev/ttyUSB0` в командах выше.

## 3. Дать пользователю доступ к UART

Узнать текущего пользователя и добавить его в группу `dialout`:

```bash
id -un
sudo usermod -aG dialout "$USER"
```

После этого необходимо полностью выйти из графической сессии и войти снова,
либо перезагрузить Raspberry Pi. Проверка:

```bash
groups
ls -l /dev/ttyUSB0
```

Владелец/группа порта обычно должны быть `root dialout`. Не рекомендуется
запускать ROS 2 через `sudo`: это скрывает ошибки прав и создаёт файлы сборки
от имени `root`.

## 4. Создать постоянный адрес `/dev/imu_stm32`

В репозитории уже есть файл:

```text
ros2_ws/src/imu_stm32_bridge/udev/99-imu-stm32.rules
```

Откройте его и оставьте активной только строку, соответствующую своему
USB-UART. Например, для CH340:

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="imu_stm32", GROUP="dialout", MODE="0660"
```

Для других распространённых адаптеров:

```udev
# CP210x
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="imu_stm32", GROUP="dialout", MODE="0660"

# FTDI FT232
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="imu_stm32", GROUP="dialout", MODE="0660"
```

Если правило уже содержит `MODE="0666"`, его можно заменить на более
безопасную комбинацию `GROUP="dialout", MODE="0660"`. Не оставляйте активными
сразу правила для нескольких адаптеров: два устройства могут одновременно
попытаться занять один симлинк.

Установить правило и перечитать конфигурацию udev:

```bash
cd ~/Ros2_ws_orbbec/ros2_ws
sudo cp src/imu_stm32_bridge/udev/99-imu-stm32.rules \
  /etc/udev/rules.d/99-imu-stm32.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Лучше после этого физически отключить и снова подключить USB-UART. Проверить:

```bash
readlink -f /dev/imu_stm32
ls -l /dev/imu_stm32
udevadm info --query=property --name=/dev/imu_stm32 \
  | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL'
```

Ожидаемый результат — `/dev/imu_stm32` указывает на реальный `ttyUSB0` или
`ttyACM0`.

### Если у адаптера несколько одинаковых экземпляров

VID:PID недостаточно: оба адаптера получат один и тот же симлинк. Получите
серийный номер:

```bash
udevadm info --query=property --name=/dev/ttyUSB0 | grep ID_SERIAL
```

Затем добавьте в правило `ATTRS{serial}` с найденным значением:

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{serial}=="SERIAL_ADAPTER", SYMLINK+="imu_stm32", GROUP="dialout", MODE="0660"
```

## 5. Проверить UART до запуска ROS 2

Мост использует параметры:

- скорость: `115200` бод;
- формат: `8N1` — 8 бит данных, без чётности, 1 стоп-бит;
- бинарный протокол STM32, поэтому `cat` не является полноценной проверкой.

Проверить, что порт открывается:

```bash
sudo apt install -y python3-serial
python3 - <<'PY'
import serial

with serial.Serial('/dev/imu_stm32', 115200, timeout=1) as port:
    print('UART открыт:', port.name, port.baudrate, port.bytesize,
          port.parity, port.stopbits)
PY
```

Если появляется `PermissionError`, проверить группу `dialout` и выполнить
повторный вход в систему. Если появляется `No such file or directory`, сначала
исправить udev-ссылку. Если порт занят, найти процесс:

```bash
sudo fuser -v /dev/imu_stm32
sudo lsof /dev/imu_stm32
```

Иногда `ModemManager` захватывает USB-UART как модем. Если он установлен и
мешает работе адаптера, проверить его логи и отключить:

```bash
sudo systemctl disable --now ModemManager
```

## 6. Собрать и запустить IMU-мост

Из корня рабочего пространства:

```bash
cd ~/Ros2_ws_orbbec/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select imu_stm32_bridge
source install/setup.bash

ros2 launch imu_stm32_bridge imu.launch.py port:=/dev/imu_stm32
```

В другом терминале проверить данные:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Ros2_ws_orbbec/ros2_ws/install/setup.bash

ros2 topic list | grep '^/imu/'
ros2 topic echo /imu/data --once
ros2 topic echo /imu/azimuth --once
```

После запуска в логе должны появиться сообщения об открытии UART и кадрах
IMU. Если кадры не идут, проверить крестное подключение `TX/RX`, общий
`GND`, питание платы, выбранный `/dev/imu_stm32` и скорость `115200`.

## 7. Запуск всего проекта

`project_start` уже использует `/dev/imu_stm32` по умолчанию:

```bash
ros2 launch project_start start.launch.py
```

Полный запуск навигации:

```bash
ros2 launch robot_navigation bringup.launch.py
```

Если постоянный симлинк ещё не создан, можно временно указать фактический
порт:

```bash
ros2 launch project_start start.launch.py imu_port:=/dev/ttyUSB0
# или
ros2 launch robot_navigation bringup.launch.py imu_port:=/dev/ttyUSB0
```

После настройки рекомендуется всегда использовать `/dev/imu_stm32`, а не
`/dev/ttyUSB0`.

## 8. Калибровка после подключения

Сначала неподвижно установить робот и выполнить калибровку гироскопа:

```bash
ros2 service call /imu/imu_stm32_bridge/gyro_calib std_srvs/srv/Trigger
```

Калибровка магнитометра:

```bash
ros2 service call /imu/imu_stm32_bridge/mag_calib_start std_srvs/srv/Trigger
# вращать плату IMU во всех направлениях примерно 30 секунд
ros2 service call /imu/imu_stm32_bridge/mag_calib_stop_save std_srvs/srv/Trigger
```

Курс относительно текущего положения можно обнулить:

```bash
ros2 service call /imu/imu_stm32_bridge/zero_yaw std_srvs/srv/Trigger
```

После калибровки проверить, что при повороте робота изменяются `yaw` в
`/imu/data` и `azimuth` в `/imu/azimuth`, а `/wheel/odometry` продолжает
показывать расстояние от энкодеров.
