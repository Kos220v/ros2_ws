# imu_stm32_bridge

Мост между инерционным модулем на STM32F303 (MPU6050 + QMC5883L) и ROS 2 Jazzy.
Читает бинарный протокол по UART и публикует данные в ROS.

## Топики (в неймспейсе `imu`)

| Топик | Тип | Описание |
|---|---|---|
| `imu/data` | sensor_msgs/Imu | кватернион (ENU), угловые скорости, ускорения |
| `imu/mag` | sensor_msgs/MagneticField | магнитное поле, Тесла |
| `imu/azimuth` | std_msgs/Float32 | азимут с компенсацией наклона, 0..360° |

## Сервисы (std_srvs/Trigger)

| Сервис | Действие |
|---|---|
| `imu/imu_stm32_bridge/mag_calib_start` | начать калибровку магнитометра (вращать плату ~30 с) |
| `imu/imu_stm32_bridge/mag_calib_stop_save` | применить и сохранить калибровку |
| `imu/imu_stm32_bridge/mag_calib_cancel` | отменить калибровку |
| `imu/imu_stm32_bridge/gyro_calib` | калибровка гироскопа (не трогать ~2 с) |
| `imu/imu_stm32_bridge/zero_yaw` | обнулить курс относительно текущего |
| `imu/imu_stm32_bridge/clear_yaw` | снять смещение курса |
| `imu/imu_stm32_bridge/save_flash` | сохранить калибровку во flash |

## Параметры

`port` (/dev/imu_stm32), `baud` (115200), `frame_id` (imu_link),
`rate` (10/25/50/100, на лету), `declination` (град, на лету),
ковариации `*_stddev`, `publish_mag`.

## Сборка и запуск

```bash
# Пакет уже лежит в src вашего workspace (или скопируйте туда каталог ros2/imu_stm32_bridge)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select imu_stm32_bridge
source install/setup.bash

# Симлинк на порт (один раз)
sudo cp src/imu_stm32_bridge/udev/99-imu-stm32.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Запуск
ros2 launch imu_stm32_bridge imu.launch.py
# или с другим портом:
ros2 launch imu_stm32_bridge imu.launch.py port:=/dev/ttyUSB0
```

## Проверка

```bash
ros2 topic echo /imu/data --once
ros2 topic echo /imu/azimuth --once
ros2 service call /imu/imu_stm32_bridge/gyro_calib std_srvs/srv/Trigger
```

## Калибровка магнитометра

```bash
ros2 service call /imu/imu_stm32_bridge/mag_calib_start std_srvs/srv/Trigger
# ... вращайте плату во всех плоскостях ~30 с, прогресс виден в логе узла ...
ros2 service call /imu/imu_stm32_bridge/mag_calib_stop_save std_srvs/srv/Trigger
```

## Подключение через USB-UART к Raspberry Pi

Подробная инструкция по подключению TX/RX, настройке `dialout`, поиску
`ttyUSB`/`ttyACM`, созданию постоянного `/dev/imu_stm32`, проверке прав и
запуску на Raspberry Pi находится в
[`docs/USB_UART_RASPBERRY_PI.md`](docs/USB_UART_RASPBERRY_PI.md).

Кратко: после настройки udev основной запуск использует стабильный адрес:

```bash
ros2 launch imu_stm32_bridge imu.launch.py port:=/dev/imu_stm32
```

Если udev ещё не настроен, временно можно передать фактический порт:

```bash
ros2 launch imu_stm32_bridge imu.launch.py port:=/dev/ttyUSB0
```

Подробности протокола: `docs/PROTOCOL.md` в корне репозитория.
