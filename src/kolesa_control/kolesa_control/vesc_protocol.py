# -*- coding: utf-8 -*-
"""
Реализация бинарного протокола VESC поверх UART.

Модуль не зависит от GUI и от pyserial — это «чистая» логика, которую можно
тестировать отдельно. Здесь собраны:

* расчёт контрольной суммы CRC16 (CCITT/XMODEM, как в прошивке VESC);
* упаковка полезной нагрузки в кадр (start / length / payload / crc / stop);
* конечный автомат разбора входящего потока байт в кадры;
* конструкторы команд (запрос версии, чтение значений, управление и т.д.);
* разбор ответа COMM_GET_VALUES (телеметрия);
* расшифровка кодов неисправностей.

Раскладка полей COMM_GET_VALUES сверена с исходниками прошивки (commands.c
проекта bldc) и с реальным захваченным пакетом.

Раскладка COMM_GET_VALUES (после командного байта, offset 1):

  offset  поле                   тип     масштаб
  1       temp_fet               int16   / 10
  3       temp_motor             int16   / 10
  5       current_motor          int32   / 100
  9       current_in             int32   / 100
  13      id                     int32   / 100
  17      iq                     int32   / 100
  21      duty_now               int16   / 1000
  23      rpm (eRPM)             int32   / 1
  27      v_in                   int16   / 10
  29      amp_hours              int32   / 10000
  33      amp_hours_charged      int32   / 10000
  37      watt_hours             int32   / 10000
  41      watt_hours_charged     int32   / 10000
  45      tachometer             int32   / 1    (знаковый!)
  49      tachometer_abs         int32   / 1    (беззнаковый по смыслу)
  53      fault_code             uint8

Итого: 54 байта (offset 0 — командный байт, offset 1..53 — данные).
"""

import struct
from enum import IntEnum


# --------------------------------------------------------------------------
# Идентификаторы команд (COMM_PACKET_ID из datatypes.h).
# Команды с малыми номерами стабильны во всех версиях прошивки VESC,
# поэтому безопасны для прошивки 7.00.
# --------------------------------------------------------------------------
class Comm(IntEnum):
    FW_VERSION = 0
    GET_VALUES = 4
    SET_DUTY = 5
    SET_CURRENT = 6
    SET_CURRENT_BRAKE = 7
    SET_RPM = 8
    SET_POS = 9
    SET_HANDBRAKE = 10
    SET_DETECT = 11
    TERMINAL_CMD = 20
    PRINT = 21
    REBOOT = 29
    ALIVE = 30
    GET_DECODED_PPM = 31
    GET_DECODED_ADC = 32
    FORWARD_CAN = 34


# --------------------------------------------------------------------------
# Коды неисправностей (mc_fault_code) с человекочитаемым описанием на русском.
# --------------------------------------------------------------------------
FAULT_CODES = {
    0: "Нет неисправностей",
    1: "Превышение напряжения (OVER_VOLTAGE)",
    2: "Пониженное напряжение (UNDER_VOLTAGE)",
    3: "Ошибка драйвера затворов (DRV)",
    4: "Превышение тока, абсолютный предел (ABS_OVER_CURRENT)",
    5: "Перегрев силовых ключей (OVER_TEMP_FET)",
    6: "Перегрев двигателя (OVER_TEMP_MOTOR)",
    7: "Превышение напряжения драйвера затворов",
    8: "Пониженное напряжение драйвера затворов",
    9: "Пониженное напряжение микроконтроллера",
    10: "Перезагрузка по сторожевому таймеру (WATCHDOG)",
    11: "Ошибка SPI энкодера",
    12: "Сигнал sin/cos энкодера ниже минимума",
    13: "Сигнал sin/cos энкодера выше максимума",
    14: "Повреждение flash-памяти",
    15: "Большое смещение датчика тока 1",
    16: "Большое смещение датчика тока 2",
    17: "Большое смещение датчика тока 3",
    18: "Несбалансированные фазные токи",
    19: "Срабатывание защиты тормоза (BRK)",
    20: "Резолвер: потеря слежения (LOT)",
    21: "Резолвер: потеря сигнала возбуждения (DOS)",
    22: "Резолвер: потеря сигнала (LOS)",
    23: "Перегрев аккумулятора",
    24: "Защита по перегрузке аккумулятора",
}


def fault_to_text(code):
    """Вернуть текстовое описание кода неисправности."""
    return FAULT_CODES.get(code, f"Неизвестный код неисправности ({code})")


# --------------------------------------------------------------------------
# CRC16 — алгоритм CCITT/XMODEM (полином 0x1021, начальное значение 0x0000),
# идентичный crc16() из прошивки VESC.
# --------------------------------------------------------------------------
def crc16(data):
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


# --------------------------------------------------------------------------
# Упаковка полезной нагрузки в кадр VESC.
#   Короткий кадр: [0x02][len:1][payload][crc:2][0x03]
#   Длинный кадр:  [0x03][len:2][payload][crc:2][0x03]
# --------------------------------------------------------------------------
def encode_packet(payload):
    payload = bytes(payload)
    length = len(payload)
    out = bytearray()
    if length <= 255:
        out.append(0x02)
        out.append(length)
    else:
        out.append(0x03)
        out += struct.pack(">H", length)
    out += payload
    out += struct.pack(">H", crc16(payload))
    out.append(0x03)
    return bytes(out)


class PacketDecoder:
    """
    Конечный автомат сборки кадров из потока байт.

    Использование: вызывать feed(data) для каждой порции принятых байт; метод
    возвращает список готовых полезных нагрузок (payload без служебных байт),
    у которых корректно сошлась контрольная сумма.
    """

    def __init__(self):
        self.buf = bytearray()

    def reset(self):
        self.buf.clear()

    def feed(self, data):
        self.buf += bytes(data)
        payloads = []
        while True:
            payload = self._try_extract()
            if payload is None:
                break
            payloads.append(payload)
        return payloads

    def _try_extract(self):
        # Ищем стартовый байт 0x02 или 0x03, отбрасывая мусор перед ним.
        while self.buf and self.buf[0] not in (0x02, 0x03):
            del self.buf[0]
        if not self.buf:
            return None

        start = self.buf[0]
        if start == 0x02:
            header = 2
            if len(self.buf) < header:
                return None
            length = self.buf[1]
        else:  # 0x03
            header = 3
            if len(self.buf) < header:
                return None
            length = struct.unpack(">H", bytes(self.buf[1:3]))[0]

        total = header + length + 2 + 1  # header + payload + crc(2) + stop(1)
        if len(self.buf) < total:
            return None

        payload = bytes(self.buf[header:header + length])
        crc_rx = struct.unpack(
            ">H", bytes(self.buf[header + length:header + length + 2])
        )[0]
        stop = self.buf[header + length + 2]

        if stop == 0x03 and crc_rx == crc16(payload):
            del self.buf[:total]
            return payload

        # Кадр битый — сдвигаемся на один байт и пробуем ресинхронизироваться.
        del self.buf[0]
        return None


# --------------------------------------------------------------------------
# Помощники чтения значений из payload (big-endian, со знаком).
# --------------------------------------------------------------------------
class _Reader:
    def __init__(self, data, offset=0):
        self.data = data
        self.i = offset

    def remaining(self):
        return len(self.data) - self.i

    def i16(self, scale=1.0):
        v = struct.unpack_from(">h", self.data, self.i)[0]
        self.i += 2
        return v / scale

    def i32(self, scale=1.0):
        v = struct.unpack_from(">i", self.data, self.i)[0]
        self.i += 4
        if scale == 1.0:
            return v
        return v / scale

    def u8(self):
        v = self.data[self.i]
        self.i += 1
        return v


# --------------------------------------------------------------------------
# Конструкторы команд. Все возвращают payload (без обёртки кадра).
# --------------------------------------------------------------------------
def cmd_get_fw_version():
    return bytes([Comm.FW_VERSION])


def cmd_get_values():
    return bytes([Comm.GET_VALUES])


def cmd_set_current(amps):
    return bytes([Comm.SET_CURRENT]) + struct.pack(">i", int(round(amps * 1000.0)))


def cmd_set_current_brake(amps):
    return (
        bytes([Comm.SET_CURRENT_BRAKE])
        + struct.pack(">i", int(round(amps * 1000.0)))
    )


def cmd_set_duty(duty):
    return bytes([Comm.SET_DUTY]) + struct.pack(">i", int(round(duty * 100000.0)))


def cmd_set_rpm(erpm):
    return bytes([Comm.SET_RPM]) + struct.pack(">i", int(round(erpm)))


def cmd_set_handbrake(amps):
    return (
        bytes([Comm.SET_HANDBRAKE])
        + struct.pack(">i", int(round(amps * 1000.0)))
    )


def cmd_alive():
    return bytes([Comm.ALIVE])


def cmd_reboot():
    return bytes([Comm.REBOOT])


def cmd_terminal(text):
    return bytes([Comm.TERMINAL_CMD]) + text.encode("ascii", errors="ignore")


# --------------------------------------------------------------------------
# Разбор ответов.
# --------------------------------------------------------------------------
def parse_fw_version(payload):
    """payload включает командный байт. Возвращает dict или None."""
    if len(payload) < 1 or payload[0] != Comm.FW_VERSION:
        return None
    r = _Reader(payload, 1)
    res = {"major": None, "minor": None, "hw_name": "", "uuid": ""}
    if r.remaining() >= 2:
        res["major"] = r.u8()
        res["minor"] = r.u8()
    name = bytearray()
    while r.i < len(payload) and payload[r.i] != 0:
        name.append(payload[r.i])
        r.i += 1
    res["hw_name"] = name.decode("ascii", errors="replace")
    if r.i < len(payload):
        r.i += 1  # пропустить нулевой разделитель
    uuid_bytes = payload[r.i:r.i + 12]
    if uuid_bytes:
        res["uuid"] = " ".join("{:02X}".format(b) for b in uuid_bytes)
    res["version"] = (
        "{}.{:02d}".format(res["major"], res["minor"])
        if res["major"] is not None else "?"
    )
    return res


# --------------------------------------------------------------------------
# COMM_GET_VALUES: порядок и масштаб полей (после командного байта).
#
# VESC прошивка записывает поля через buffer_append_float16 / float32 /
# int32.  buffer_append_float16(buf, value, scale, &ind) сохраняет на
# проводе int16(value * scale), buffer_append_float32(buf, value, scale, &ind)
# сохраняет int32(value * scale), buffer_append_int32(buf, value, &ind)
# сохраняет int32(value) без масштабирования.
#
# _Reader.i16(scale) и i32(scale) делают обратное преобразование:
# читают int и делят на scale.
# --------------------------------------------------------------------------

_VALUES_FIELDS = [
    # (имя,       тип,   масштаб, единицы, описание)
    ("temp_fet",           "i16", 10.0,    "°C",    "Температура ключей (FET)"),
    ("temp_motor",         "i16", 10.0,    "°C",    "Температура двигателя"),
    ("current_motor",      "i32", 100.0,   "А",     "Ток двигателя"),
    ("current_in",         "i32", 100.0,   "А",     "Ток на входе (батарея)"),
    ("id",                 "i32", 100.0,   "А",     "Ток оси d"),
    ("iq",                 "i32", 100.0,   "А",     "Ток оси q"),
    ("duty",               "i16", 1000.0,  "",      "Коэффициент заполнения"),
    ("rpm",                "i32", 1.0,     "эл/мин","Электрические обороты (eRPM)"),
    ("v_in",               "i16", 10.0,    "В",     "Напряжение питания"),
    ("amp_hours",          "i32", 10000.0, "А·ч",   "Отдано (А·ч)"),
    ("amp_hours_charged",  "i32", 10000.0, "А·ч",   "Принято (А·ч)"),
    ("watt_hours",         "i32", 10000.0, "Вт·ч",  "Отдано (Вт·ч)"),
    ("watt_hours_charged", "i32", 10000.0, "Вт·ч",  "Принято (Вт·ч)"),
    # tachometer — знаковый int32, считает электрические обороты.
    # НЕ использовать tachometer_abs для одометрии с реверсом!
    ("tachometer",         "i32", 1.0,     "",      "Тахометр (знаковый)"),
    ("tachometer_abs",     "i32", 1.0,     "",      "Тахометр (абс. значение)"),
]


def values_field_meta():
    """Метаданные полей телеметрии для построения интерфейса."""
    meta = [(name, unit, label) for name, _, _, unit, label in _VALUES_FIELDS]
    meta.append(("fault_code", "", "Код неисправности"))
    meta.append(("fault_text", "", "Неисправность"))
    return meta


def parse_get_values(payload):
    """
    Разбор ответа COMM_GET_VALUES.

    Парсинг защитный: поля читаются только если в пакете осталось достаточно
    байт, поэтому функция работает с разными версиями прошивки VESC
    (включая 7.00+, которые могут добавлять дополнительные поля в конец).

    Возвращает dict или None.

    Пример возвращаемого dict:
    {
        'temp_fet': 25.3,
        'temp_motor': 30.1,
        'current_motor': 12.34,
        'current_in': 5.67,
        'id': 0.0,
        'iq': 12.34,
        'duty': 0.5,
        'rpm': 3500,
        'v_in': 48.2,
        'amp_hours': 0.0012,
        'amp_hours_charged': 0.0005,
        'watt_hours': 0.123,
        'watt_hours_charged': 0.045,
        'tachometer': 1548796,        # int, знаковый
        'tachometer_abs': 1548796,    # int, беззнаковый по смыслу
        'fault_code': 0,
        'fault_text': 'Нет неисправностей',
    }
    """
    if len(payload) < 1 or payload[0] != Comm.GET_VALUES:
        return None

    r = _Reader(payload, 1)
    res = {}

    for name, kind, scale, _unit, _label in _VALUES_FIELDS:
        need = 2 if kind == "i16" else 4
        if r.remaining() < need:
            res[name] = None
            continue
        if kind == "i16":
            res[name] = r.i16(scale)
        else:
            res[name] = r.i32(scale)

    # Код неисправности — следующий байт.
    if r.remaining() >= 1:
        res["fault_code"] = r.u8()
    else:
        res["fault_code"] = None

    res["fault_text"] = (
        fault_to_text(res["fault_code"])
        if res["fault_code"] is not None
        else "—"
    )

    return res