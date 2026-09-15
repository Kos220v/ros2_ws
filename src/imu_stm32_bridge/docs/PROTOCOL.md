# Бинарный протокол STM32 IMU <-> робот

Физический уровень: UART 115200 8N1. Порядок байт little-endian.

## Кадр

| Поле | Размер | Значение |
|---|---|---|
| SYNC | 2 | `0xAA 0x55` |
| LEN | 1 | `1 + длина PAYLOAD` |
| ID | 1 | идентификатор сообщения |
| PAYLOAD | 0..96 | данные |
| CRC | 2 | CRC16-CCITT (poly `0x1021`, init `0xFFFF`) от `LEN + ID + PAYLOAD`, младший байт первым |

Проверка CRC: `CRC("123456789") = 0x29B1`.

## Сообщения STM32 -> робот

### 0x01 ORIENTATION (80 байт, поток 10/25/50/100 Гц)

| Смещение | Тип | Поле | Описание |
|---|---|---|---|
| 0 | u32 | ts_ms | метка времени платы, мс |
| 4 | f32×4 | qw,qx,qy,qz | кватернион корпус->ENU (для `sensor_msgs/Imu`) |
| 20 | f32 | roll_deg | крен, -180..180 |
| 24 | f32 | pitch_deg | тангаж, -90..90 (+ нос вниз) |
| 28 | f32 | yaw_deg | курс fused, -180..180, 0=север, + против часовой |
| 32 | f32 | azimuth_deg | **азимут** tilt-comp, 0..360, по часовой от севера, со склонением |
| 36 | f32×3 | wx,wy,wz | угловые скорости, рад/с, без дрейфа |
| 48 | f32×3 | ax,ay,az | ускорения, м/с² |
| 60 | f32×3 | mx,my,mz | магнитное поле, мкТл, калиброванное |
| 72 | f32 | temp_c | температура MPU6050, °C |
| 76 | u8 | status | биты: 0=MPU_OK, 1=MAG_OK, 2=MAG_CAL, 3=GYRO_CAL, 4=FUSED_9X |
| 77 | u8 | calib_state | 0=idle, 1=калибровка мага, 2=калибровка гиро |
| 78 | u8 | rate_hz | текущая частота |
| 79 | u8 | reserved | 0 |

### 0x02 INFO (36 байт, по запросу и при старте)

`fw_major, fw_minor, fw_patch, res0 (4×u8)`, `uptime_ms (u32)`,
`board[16] (строка)`, `mpu_ok, mag_ok, mag_cal, gyro_cal (4×u8)`,
`declination_deg (f32)`, `rate_hz, res[3] (4×u8)`.

### 0x03 ACK (4 байта, ответ на команду)

`cmd_id (u8)`, `result (u8)`, `info (u16)`.
Коды: 0=OK, 1=неверный аргумент, 2=недопустимо сейчас, 3=ошибка железа,
4=неизвестная команда.

### 0x04 CALIB (40 байт, прогресс и данные калибровки)

`state (u8)`, `progress_pct (u8)`, `res[2]`, `mag_hard[3] (f32, мкТл)`,
`mag_scale[3] (f32)`, `gyro_bias[3] (f32, рад/с)`.

## Команды робот -> STM32

| ID | Имя | Payload | Ответ |
|---|---|---|---|
| 0x80 | PING | — | ACK |
| 0x81 | SET_RATE | u8: 10/25/50/100 | ACK(info=rate) |
| 0x82 | MAG_CALIB_START | — | ACK, далее поток CALIB |
| 0x83 | MAG_CALIB_STOP | u8: 0=отменить, 1=применить+сохранить | ACK + CALIB |
| 0x84 | GYRO_CALIB | — (не трогать ~2 с) | ACK сразу, CALIB по готовности |
| 0x85 | ZERO_YAW | u8: 0=обнулить, 1=снять (или пусто=0) | ACK |
| 0x86 | SET_DECLINATION | f32, -30..+30 | ACK |
| 0x87 | SAVE_FLASH | — | ACK |
| 0x88 | GET_INFO | — | INFO |
| 0x89 | GET_CALIB | — | CALIB |

## Пример (Python)

```python
from imu_stm32_bridge import protocol as P
ser.write(P.cmd_set_rate(50))
for msg_id, payload in P.Decoder().feed(ser.read(256)):
    if msg_id == P.MSG_ORIENTATION:
        m = P.Orientation.from_payload(payload)
        print(m.azimuth_deg, m.qw, m.qx, m.qy, m.qz)
```

## Системы координат

- Оси корпуса: X вперёд, Y влево, Z вверх (REP-103).
- Кватернион: поворот корпус->ENU (x=восток, y=север, z=вверх).
- Азимут: угол оси X по часовой от географического севера (со склонением).
