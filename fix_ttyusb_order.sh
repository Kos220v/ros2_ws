#!/bin/bash
# ============================================================================
# fix_ttyusb_order.sh — гарантирует порядок USB-serial портов на каждом
# старте системы:
#   ЛИДАР (Silicon Labs CP210x, VID:PID 10c4:ea60) -> /dev/ttyUSB0
#   GPS   (Prolific PL2303,    VID:PID 067b:2303) -> /dev/ttyUSB1
#
# КАК ЭТО РАБОТАЕТ:
#   Номера ttyUSB* назначает ЯДРО в порядке опроса USB; udev не может их
#   переименовать. Но можно ПЕРЕПРИВЯЗАТЬ (unbind/bind) оба интерфейса в
#   нужном порядке: лидар привязывается ПЕРВЫМ -> получает ttyUSB0,
#   GPS вторым -> ttyUSB1.
#
#   Скрипт идемпотентен: если порядок уже правильный — сразу выходит.
#   Вызывается udev-правилом (см. 99-robot-usb.rules) при появлении
#   любого ttyUSB* — т.е. при КАЖДОЙ загрузке и при горячем подключении.
#
# УСТАНОВКА:
#   sudo cp fix_ttyusb_order.sh /usr/local/sbin/
#   sudo chmod +x /usr/local/sbin/fix_ttyusb_order.sh
#   (правило RUN уже есть в /etc/udev/rules.d/99-robot-usb.rules)
#   sudo udevadm control --reload-rules && sudo udevadm trigger
#
# ВНИМАНИЕ:
#   Перепривязка на ~1 секунду «отключает» оба serial-порта. Это
#   происходит при загрузке ДО запуска стека — безопасно. Не запускайте
#   скрипт вручную, пока работает драйвер лидара/GPS.
# ============================================================================

LIDAR_VIDPID="10c4:ea60"   # CP210x (лидар)
GPS_VIDPID="067b:2303"     # PL2303 (GPS)

# Блокировка: не запускаться параллельно (udev дёргает на каждый ttyUSB)
exec 9>/tmp/fix_ttyusb_order.lock
flock -n 9 || exit 0

logger -t fix_ttyusb_order "Скрипт вызван (udev RUN / ручной запуск)"

# --- tty_of <vid:pid> : найти ttyUSBx по VID:PID ---------------------------
tty_of() {
    for d in /sys/class/tty/ttyUSB*/device; do
        [ -e "$d" ] || continue
        p=$(readlink -f "$d") || continue
        intf=$(dirname "$p")
        vid=$(cat "$(dirname "$intf")/idVendor" 2>/dev/null)
        pid=$(cat "$(dirname "$intf")/idProduct" 2>/dev/null)
        if [ "$vid:$pid" = "$1" ]; then
            basename "$p"
            return 0
        fi
    done
    return 1
}

# Текущее состояние (для диагностики)
cur_lidar=$(tty_of "$LIDAR_VIDPID")
cur_gps=$(tty_of "$GPS_VIDPID")
logger -t fix_ttyusb_order "Текущее: лидар=$cur_lidar GPS=$cur_gps"

# --- ищем оба устройства ----------------------------------------------------
lidar_tty=$cur_lidar
gps_tty=$cur_gps
if [ -z "$lidar_tty" ] || [ -z "$gps_tty" ]; then
    # Устройства ещё не все появились (или не подключены) — выходим.
    # udev RUN сработает снова при появлении последнего ttyUSB*.
    logger -t fix_ttyusb_order "Не все устройства на месте (лидар=$lidar_tty GPS=$gps_tty) — жду следующего события"
    exit 0
fi

# Порядок уже правильный?
if [ "$lidar_tty" = "ttyUSB0" ] && [ "$gps_tty" = "ttyUSB1" ]; then
    exit 0
fi

# --- вспомогательные: интерфейс и драйвер устройства ------------------------
intf_of() { dirname "$(readlink -f "/sys/class/tty/$1/device")"; }
drv_of() {
    local tail
    tail=$(basename "$1")
    basename "$(readlink -f "/sys/bus/usb/devices/$tail/driver" 2>/dev/null)" 2>/dev/null
}

lidar_intf=$(intf_of "$lidar_tty")
gps_intf=$(intf_of "$gps_tty")
lidar_drv=$(drv_of "$lidar_intf")
gps_drv=$(drv_of "$gps_intf")

if [ -z "$lidar_drv" ] || [ -z "$gps_drv" ]; then
    logger -t fix_ttyusb_order "Не удалось определить драйверы (lidar=$lidar_drv gps=$gps_drv)"
    exit 0
fi

lt=$(basename "$lidar_intf")   # например 1-1.2:1.0
gt=$(basename "$gps_intf")

logger -t fix_ttyusb_order "Порядок USB неверный: лидар=$lidar_tty, GPS=$gps_tty. Перепривязываю..."

# Отвязать оба интерфейса
echo "$lt" > "/sys/bus/usb/drivers/$lidar_drv/unbind" 2>/dev/null
echo "$gt" > "/sys/bus/usb/drivers/$gps_drv/unbind"    2>/dev/null
sleep 0.3

# Привязать СНАЧАЛА лидар (займёт ttyUSB0), потом GPS (ttyUSB1)
echo "$lt" > "/sys/bus/usb/drivers/$lidar_drv/bind" 2>/dev/null
sleep 0.3
echo "$gt" > "/sys/bus/usb/drivers/$gps_drv/bind"    2>/dev/null

logger -t fix_ttyusb_order "Готово: лидар -> ttyUSB0, GPS -> ttyUSB1"

# Финальная проверка
sleep 0.3
lidar_new=$(tty_of "$LIDAR_VIDPID")
gps_new=$(tty_of "$GPS_VIDPID")
logger -t fix_ttyusb_order "Проверка: лидар=$lidar_new, GPS=$gps_new"

exit 0
