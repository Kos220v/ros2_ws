# -*- coding: utf-8 -*-
"""
Драйвер одного контроллера FS75100 / VESC поверх UART.

Класс инкапсулирует pyserial-соединение, фоновый поток чтения и разбор
кадров. Команды управления отправляются из потока ROS, телеметрия читается
фоновым потоком и хранится под блокировкой.

Авто-переподключение: если порт не открылся или связь оборвалась, поток
периодически пытается открыть его заново.

Телеметрия (словарь, возвращаемый get_telemetry()):

    rpm               — eRPM из VESC (электрические обороты в минуту)
    tachometer         — знаковый int32, считает электрические обороты
    tachometer_abs     — беззнаковый по смыслу int32
    tacho_abs          — alias для tachometer_abs (обратная совместимость)
    erpm               — alias для rpm
    _rx_time           — time.monotonic() момента приёма пакета
    _payload_len       — длина payload пакета
    + остальные поля из COMM_GET_VALUES
"""

import threading
import time

import serial

from . import vesc_protocol as vp


class VescDriver:
    def __init__(self, name, port, baud, logger=None, reopen_period=2.0):
        self.name = name
        self.port = port
        self.baud = baud
        self.logger = logger
        self.reopen_period = reopen_period

        self._ser = None
        self._decoder = vp.PacketDecoder()

        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()

        self._telemetry = None
        self._connected = False

        self._running = False
        self._thread = None

    # --------------------------------------------------------------- lifecycle

    def start(self):
        """Запуск фонового потока чтения UART."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(
            target=self._loop,
            name=f"vesc_{self.name}",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        """Остановка фонового потока и закрытие UART."""
        self._running = False

        # Перед закрытием пробуем остановить двигатель.
        try:
            self.set_current(0.0)
        except Exception:
            pass

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

        self._close()

    # --------------------------------------------------------------- свойства

    @property
    def connected(self):
        with self._state_lock:
            return self._connected

    def get_telemetry(self):
        """
        Возвращает последнюю телеметрию (копию словаря) или None.

        Копия возвращается для того, чтобы вызывающий код мог безопасно
        читать поля без блокировки.
        """
        with self._state_lock:
            return dict(self._telemetry) if self._telemetry else None

    # --------------------------------------------------------------- команды

    def set_rpm(self, erpm):
        """
        Задать скорость в eRPM.

        В VESC команда COMM_SET_RPM принимает электрические RPM.
        """
        return self._send(vp.cmd_set_rpm(erpm))

    def set_current(self, amps):
        """Задать ток двигателя (А)."""
        return self._send(vp.cmd_set_current(amps))

    def set_current_brake(self, amps):
        """Задать тормозной ток (А)."""
        return self._send(vp.cmd_set_current_brake(amps))

    def set_duty(self, duty):
        """Задать duty cycle (-1.0 .. 1.0)."""
        return self._send(vp.cmd_set_duty(duty))

    def request_telemetry(self):
        """Запросить COMM_GET_VALUES."""
        return self._send(vp.cmd_get_values())

    def request_fw(self):
        """Запросить версию прошивки."""
        return self._send(vp.cmd_get_fw_version())

    # --------------------------------------------------------------- внутреннее

    def _send(self, payload):
        """Отправить payload в VESC. Возвращает True при успехе."""
        if payload is None:
            return False

        ser = self._ser
        if ser is None:
            return False

        try:
            packet = vp.encode_packet(payload)
            with self._write_lock:
                ser.write(packet)
            return True

        except Exception as e:
            self._log_warn(f"[{self.name}] ошибка записи: {e}")
            self._close()
            return False

    def _open(self):
        """Открыть serial-порт."""
        try:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1,
            )

            # Очищаем буферы после открытия.
            try:
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
            except Exception:
                pass

            self._decoder.reset()

            with self._state_lock:
                self._connected = True
                self._telemetry = None

            self._log_info(
                f"[{self.name}] открыт порт {self.port} @ {self.baud}"
            )

            # Запрашиваем версию прошивки.
            self.request_fw()

            return True

        except Exception as e:
            self._log_warn(
                f"[{self.name}] не удалось открыть {self.port}: {e}"
            )
            self._ser = None

            with self._state_lock:
                self._connected = False
                self._telemetry = None

            return False

    def _close(self):
        """Закрыть serial-порт и отметить драйвер как отключенный."""
        ser = self._ser
        self._ser = None

        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

        self._mark_disconnected()

    def _mark_disconnected(self):
        with self._state_lock:
            self._connected = False
            self._telemetry = None

    def _loop(self):
        """Фоновый поток чтения UART и авто-переподключения."""
        last_open_try = 0.0

        while self._running:
            # Если порт не открыт — пытаемся переоткрыть.
            if self._ser is None:
                now = time.monotonic()
                if now - last_open_try >= self.reopen_period:
                    last_open_try = now
                    self._open()
                else:
                    time.sleep(0.1)
                continue

            # Читаем данные из порта.
            try:
                data = self._ser.read(256)
            except Exception as e:
                self._log_warn(f"[{self.name}] ошибка чтения: {e}")
                self._close()
                continue

            if data:
                try:
                    packets = self._decoder.feed(data)
                    for payload in packets:
                        self._handle_payload(payload)
                except Exception as e:
                    self._log_warn(
                        f"[{self.name}] ошибка декодирования: {e}"
                    )
                    self._decoder.reset()
            else:
                time.sleep(0.002)

        self._close()

    def _handle_payload(self, payload):
        """Обработка полезной нагрузки VESC-пакета."""
        if not payload:
            return

        cmd = payload[0]

        if cmd == vp.Comm.GET_VALUES:
            self._handle_get_values(payload)

        elif cmd == vp.Comm.FW_VERSION:
            self._handle_fw_version(payload)

    def _handle_get_values(self, payload):
        """
        Обработка COMM_GET_VALUES.

        Парсинг делегирован в vesc_protocol.parse_get_values().
        Добавляем только алиасы и метку времени.
        """
        vals = vp.parse_get_values(payload)

        if not vals:
            return

        # Alias: rpm от VESC фактически является eRPM.
        if "rpm" in vals and "erpm" not in vals:
            vals["erpm"] = vals["rpm"]

        # Alias для обратной совместимости со старым кодом.
        if "tachometer_abs" in vals and "tacho_abs" not in vals:
            vals["tacho_abs"] = vals["tachometer_abs"]

        vals["_rx_time"] = time.monotonic()
        vals["_payload_len"] = len(payload)

        with self._state_lock:
            self._telemetry = vals
            self._connected = True

    def _handle_fw_version(self, payload):
        """Обработка ответа COMM_FW_VERSION."""
        info = vp.parse_fw_version(payload)
        if info:
            self._log_info(
                f"[{self.name}] прошивка {info['version']}, "
                f"платформа {info['hw_name']}"
            )

    # --------------------------------------------------------------- логирование

    def _log_info(self, msg):
        if self.logger:
            self.logger.info(msg)

    def _log_warn(self, msg):
        if self.logger:
            self.logger.warning(msg)