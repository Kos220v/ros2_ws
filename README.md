# ros2_ws
разработка робота

## USB-порты: жёсткая схема (лидар = ttyUSB0, GPS = ttyUSB1)

Номера `ttyUSB*` назначает ядро по порядку опроса USB, поэтому udev не
может их переименовать. Чтобы порядок был **всегда** правильным, на роботе
установлены:

- `/etc/udev/rules.d/99-robot-usb.rules` — правило `RUN`, вызывающее
  фиксатор при каждом появлении `ttyUSB*` (загрузка/горячее подключение);
- `/usr/local/sbin/fix_ttyusb_order.sh` — перепривязывает интерфейсы:
  лидар (CP210x `10c4:ea60`) → `ttyUSB0`, GPS (PL2303 `067b:2303`) →
  `ttyUSB1` (если порядок нарушен).

Установка (один раз):
```bash
sudo cp 99-robot-usb.rules /etc/udev/rules.d/
sudo cp fix_ttyusb_order.sh /usr/local/sbin/
sudo chmod +x /usr/local/sbin/fix_ttyusb_order.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Проверка после перезагрузки:
```bash
udevadm info /dev/ttyUSB0 | grep ID_VENDOR_ID   # 10c4 (лидар)
udevadm info /dev/ttyUSB1 | grep ID_VENDOR_ID   # 067b (GPS)
journalctl -t fix_ttyusb_order | tail           # лог фиксатора
```

Стек (`start.launch.py`) дополнительно проверяет схему при старте и не
запустится с понятной ошибкой, если она нарушена.
