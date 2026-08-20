#!/usr/bin/env python3
"""
Нода elrs_receiver

Исправления (Turn 6) — большая зона нечувствительности и "разворот вместо
прямой" на пульте:

1. КАЛИБРОВКА КАНАЛОВ БЫЛА НЕВЕРНОЙ. Раньше сырые 11-битные значения CRSF
   (0..2047) переводились в диапазон 0..1 простым делением на 2048:
       normalized = raw / 2048.0
   Но стандартный диапазон стика CRSF — НЕ 0..2047 с центром 1024, а
   172..1811 с центром ОКОЛО 992 (см. CRSF_CHANNEL_MIN/CENTER/MAX ниже).
   Из-за этого при делении на 2048:
     - физический центр стика (~992) давал нормализованное значение ~0.484,
       а не ровно 0.5 — то есть "центр" в коде и центр стика не совпадали;
     - полный отклон стика (172..1811) давал только ~0.084..0.884, а не
       0.0..1.0 — то есть робот НИКОГДА не получал полную заданную
       max_linear_velocity/max_angular_velocity, даже при стике до упора.
   Вместе со старым deadzone=0.20 (см. п.2) это и ощущалось как "огромная
   зона нечувствительности": стик до ~20% хода вообще ничего не делал,
   а после 20% реальный диапазон уже был обрезан сверху те же ~12%.
   ИСПРАВЛЕНО: добавлена нормализация по настоящим калибровочным точкам
   (channel_min/channel_center/channel_max, настраиваемые параметрами) —
   теперь центр стика точно даёт 0.5, а полный отклон в любую сторону даёт
   ровно 0.0/1.0.

2. deadzone=0.20 по умолчанию — это 20% полного хода стика ИЗ КАЖДОЙ
   стороны, то есть почти половина всего диапазона стика вообще не
   передавалась в команду. ИСПРАВЛЕНО: значение по умолчанию уменьшено до
   0.02 (2%) — обычная величина для устранения дрожания стика в нейтрали,
   не съедающая полезный ход. Подберите под свой пульт (см. README).

3. "Ехать прямо — робот разворачивается": канал газа и канал руля были
   жёстко закодированы как индексы 1 и 0 (channels[1]/channels[0]). Если на
   вашем передатчике/приёмнике каналы назначены иначе (другой порядок,
   например TAER вместо AETR, или газ/руль физически перепутаны местами),
   нажатие "вперёд" на самом деле читалось кодом как команда поворота.
   ИСПРАВЛЕНО: индексы каналов газа/руля и их инверсия вынесены в параметры
   (throttle_channel, steering_channel, invert_throttle, invert_steering) —
   можно откалибровать без правки кода. Чтобы определить, какой индекс
   реально ваш газ/руль: `ros2 topic echo /rc_channels` и подвигать ТОЛЬКО
   одну ручку стика — увидите, какой элемент массива меняется.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8
from geometry_msgs.msg import Twist
import serial

# Стандартные калибровочные точки канала CRSF (11 бит, "extended" диапазон
# производителей ELRS/TBS). Если ваш передатчик откалиброван иначе —
# переопределите параметрами channel_min/channel_center/channel_max.
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_CENTER = 992
CRSF_CHANNEL_MAX = 1811


class ControlModes:
    AUTO = 0
    MANUAL = 1
    AVOID = 2
    RETURN_HOME = 3

class CRSFParser:
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
    
    def __init__(self):
        self.channels = [0] * 16
        self.state = 'WAIT_SYNC'
        self.buffer = bytearray()
        self.expected_length = 0
        
    def crc8_dvb_s2(self, crc, a):
        crc ^= a
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc = crc << 1
        return crc & 0xFF
    
    def parse_rc_channels_packed(self, payload):
        channel_bits = []
        for byte in payload:
            for bit in range(8):
                channel_bits.append((byte >> bit) & 1)
        
        for i in range(16):
            start_bit = i * 11
            if start_bit + 11 <= len(channel_bits):
                value = 0
                for j in range(11):
                    if start_bit + j < len(channel_bits):
                        value |= (channel_bits[start_bit + j] << j)
                self.channels[i] = value
        return self.channels
    
    def process_byte(self, byte):
        if self.state == 'WAIT_SYNC':
            if byte == self.CRSF_ADDRESS_FLIGHT_CONTROLLER or byte == self.CRSF_ADDRESS_RADIO_TRANSMITTER:
                self.buffer = bytearray([byte])
                self.state = 'LENGTH'
        elif self.state == 'LENGTH':
            self.buffer.append(byte)
            self.expected_length = byte + 2
            self.state = 'DATA'
        elif self.state == 'DATA':
            self.buffer.append(byte)
            if len(self.buffer) >= self.expected_length:
                self.state = 'WAIT_SYNC'
                return self.process_frame()
        return None
    
    def process_frame(self):
        if len(self.buffer) < 5:
            return None
        
        crc = 0
        for i in range(2, len(self.buffer) - 1):
            crc = self.crc8_dvb_s2(crc, self.buffer[i])
        
        if crc != self.buffer[-1]:
            return None
        
        frame_type = self.buffer[2]
        if frame_type == self.CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            payload = self.buffer[3:-1]
            return self.parse_rc_channels_packed(payload)
        return None

class ELRSReceiverNode(Node):
    def __init__(self):
        super().__init__('elrs_receiver_node')
        
        # Параметры
        self.declare_parameter('port', '/dev/ttyAMA2')
        self.declare_parameter('baudrate', 420000)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 2.0)
        # ИСПРАВЛЕНО (Turn 6): было 0.20 (20% хода стика с каждой стороны —
        # огромная мёртвая зона). Уменьшено до типичных 2%.
        self.declare_parameter('deadzone', 0.02)
        self.declare_parameter('timeout_sec', 0.3)

        # Калибровка канала CRSF (сырые 11-битные значения). Стандартные
        # значения по умолчанию — 172/992/1811. Если ваш пульт/приёмник
        # откалиброван иначе, переопределите здесь.
        self.declare_parameter('channel_min', CRSF_CHANNEL_MIN)
        self.declare_parameter('channel_center', CRSF_CHANNEL_CENTER)
        self.declare_parameter('channel_max', CRSF_CHANNEL_MAX)

        # ИСПРАВЛЕНО (Turn 6): индексы каналов газа/руля и их инверсия были
        # жёстко зашиты в коде (channels[1]/channels[0]). Теперь настраиваются
        # параметрами — см. docstring модуля про калибровку через /rc_channels.
        self.declare_parameter('throttle_channel', 1)
        self.declare_parameter('steering_channel', 0)
        self.declare_parameter('invert_throttle', False)
        self.declare_parameter('invert_steering', False)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value

        self.channel_min = int(self.get_parameter('channel_min').value)
        self.channel_center = int(self.get_parameter('channel_center').value)
        self.channel_max = int(self.get_parameter('channel_max').value)

        self.throttle_channel = int(self.get_parameter('throttle_channel').value)
        self.steering_channel = int(self.get_parameter('steering_channel').value)
        self.invert_throttle = bool(self.get_parameter('invert_throttle').value)
        self.invert_steering = bool(self.get_parameter('invert_steering').value)

        self.publisher = self.create_publisher(Float32MultiArray, '/rc_channels', 10)
        self.mode_publisher = self.create_publisher(Int8, '/control_mode', 10)
        
        # <--- ИЗМЕНЕНО: Убрали публикацию в /cmd_vel отсюда.
        # Теперь этот узел отвечает ТОЛЬКО за пульт.
        self.manual_pub = self.create_publisher(Twist, '/cmd_vel/manual', 10) 
        
        self.mode_subscriber = self.create_subscription(
            Int8, '/control_mode_command', self.mode_callback, 10
        )
        
        self.current_mode = ControlModes.MANUAL
        self.get_logger().info(f'Initial control mode: MANUAL (1)')

        # Опубликовать стартовый режим сразу: после перезагрузки стека робот
        # всегда в РУЧНОМ режиме. Подписчики (waypoint_follower и др.) должны
        # узнать об этом немедленно, не дожидаясь первого пакета с пульта.
        mode_msg = Int8()
        mode_msg.data = self.current_mode
        self.mode_publisher.publish(mode_msg)
        self.get_logger().info('Published initial control mode: MANUAL')

        # Запоминание режима при потере сигнала пульта: режим НЕ меняется
        # самовольно, пока пульт молчит, и восстанавливается при его
        # возвращении (см. check_mode_switch / read_serial).
        self.remembered_mode = None
        self.signal_lost_logged = False
        
        self.serial = None
        try:
            self.serial = serial.Serial(
                port=port, baudrate=baudrate, timeout=0.01,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info(f'Connected to ELRS receiver on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
        
        self.parser = CRSFParser()
        
        self.mode_check_timer = self.create_timer(0.1, self.check_mode_switch) 
        self.timer = self.create_timer(0.01, self.read_serial)                  
        self.cmd_vel_timer = self.create_timer(0.05, self.publish_cmd_vel)        
        
        self.last_toggle_value = 0.5
        self.debounce_counter = 0
        self.debounce_threshold = 4
        
        self.last_normalized_channels = [0.5] * 16
        # Сырые значения каналов (0..2047) — нужны для точной калибровки
        # газа/руля по channel_min/center/max. Инициализированы центром,
        # чтобы до прихода первого пакета команда была "стоп", а не рывок.
        self.last_raw_channels = [self.channel_center] * 16
        self.last_packet_time = self.get_clock().now()

        self.get_logger().info('ELRS Receiver Node started (Manual Mode Only)')
        min_dead = 0.5 - self.deadzone
        max_dead = 0.5 + self.deadzone
        self.get_logger().info(f'--- DEADZONE SET TO: {self.deadzone} ---')
        self.get_logger().info(f'  Effective deadzone range: [{min_dead:.2f}, {max_dead:.2f}]')
        self.get_logger().info(
            f'  Channel calibration: min={self.channel_min} center={self.channel_center} '
            f'max={self.channel_max} | throttle_ch={self.throttle_channel} '
            f'steering_ch={self.steering_channel}'
        )
        self.get_logger().info('  NOTE: This node now publishes ONLY to /cmd_vel/manual')
        self.get_logger().info('--- ТУМБЛЕР НА КАНАЛЕ 9 (индекс 8) ---')
        self.get_logger().info(
            '  Калибровка газа/руля: `ros2 topic echo /rc_channels`, подвигать '
            'по одной ручке стика — какой индекс меняется, тот и указать в '
            'throttle_channel/steering_channel.'
        )
    
    def mode_callback(self, msg):
        new_mode = msg.data
        if self.set_control_mode(new_mode):
            self.get_logger().info(f'External command: Mode changed to {new_mode}')
    
    def set_control_mode(self, new_mode):
        valid_modes = [ControlModes.AUTO, ControlModes.MANUAL, ControlModes.AVOID, ControlModes.RETURN_HOME]
        if new_mode in valid_modes:
            if new_mode != self.current_mode:
                self.current_mode = new_mode
                mode_msg = Int8()
                mode_msg.data = self.current_mode
                self.mode_publisher.publish(mode_msg)
                
                mode_names = {0: 'AUTO', 1: 'MANUAL', 2: 'AVOID', 3: 'RETURN_HOME'}
                self.get_logger().info(f'Mode set to: {mode_names.get(self.current_mode)}')
                
                # <--- ИЗМЕНЕНО: Мы больше не делаем аварийную остановку здесь принудительно.
                # Пусть мультиплексор сам решит, что делать, если пульт замолчал.
                stop_msg = Twist()
                # self.manual_pub.publish(stop_msg) # Закомментировано: не нужно слать стоп в топик пульта
                
                return True
        return False
    
    def check_mode_switch(self):
        if not hasattr(self, 'last_normalized_channels') or len(self.last_normalized_channels) <= 8:
            return

        # Пульт потерян — НЕ переключаем режим: каналы заморожены на последних
        # значениях, и переключение по ним было бы «несанкционированным».
        # Режим запоминается (remembered_mode) и останется текущим, пока пульт
        # не вернётся (тогда тумблер применится заново).
        time_diff = (self.get_clock().now() - self.last_packet_time).nanoseconds / 1e9
        if time_diff > self.timeout_sec:
            return

        toggle_val = self.last_normalized_channels[8]
        
        target_zone = -1
        if toggle_val < 0.33:
            target_zone = 0 
        elif toggle_val > 0.67:
            target_zone = 2 
        else:
            target_zone = 1 

        current_zone = -1
        if self.current_mode == ControlModes.MANUAL:
            current_zone = 0
        elif self.current_mode == ControlModes.RETURN_HOME:
            current_zone = 2
        else:
            current_zone = 1 

        if target_zone != current_zone:
            if abs(toggle_val - self.last_toggle_value) > 0.15: 
                self.debounce_counter += 1
                if self.debounce_counter >= self.debounce_threshold:
                    self.apply_toggle_mode(target_zone)
                    self.last_toggle_value = toggle_val
                    self.debounce_counter = 0
            else:
                self.debounce_counter = 0
        else:
            self.last_toggle_value = (self.last_toggle_value * 0.8) + (toggle_val * 0.2)
            self.debounce_counter = 0

    def apply_toggle_mode(self, zone):
        new_mode = None
        mode_name = ""
        
        if zone == 0:
            new_mode = ControlModes.MANUAL
            mode_name = "MANUAL (Position 1)"
        elif zone == 2:
            new_mode = ControlModes.RETURN_HOME
            mode_name = "RETURN_HOME (Position 3)"
        else:
            new_mode = ControlModes.AUTO
            mode_name = "AUTO + AVOID (Position 2)"

        if new_mode is not None and new_mode != self.current_mode:
            self.get_logger().warn(f'TOGGLE SWITCH: Changing to {mode_name}')
            self.set_control_mode(new_mode)
            
    def apply_deadzone(self, value):
        if abs(value - 0.5) < self.deadzone:
            return 0.5
        return value

    def normalize_stick(self, raw):
        """
        Правильная калибровка сырого канала CRSF в 0..1 с центром РОВНО 0.5
        и полным ходом стика в любую сторону, дающим РОВНО 0.0/1.0.

        В отличие от старого `raw / 2048.0` (центр и края были смещены —
        см. docstring модуля), здесь используется настоящая калибровка по
        channel_min/channel_center/channel_max.
        """
        raw = float(raw)
        if raw >= self.channel_center:
            span = max(1.0, float(self.channel_max - self.channel_center))
            frac = (raw - self.channel_center) / span
        else:
            span = max(1.0, float(self.channel_center - self.channel_min))
            frac = (raw - self.channel_center) / span

        frac = max(-1.0, min(1.0, frac))
        return 0.5 + frac * 0.5

    def map_channels_to_velocity(self, raw_channels):
        max_ch = max(self.throttle_channel, self.steering_channel)
        if len(raw_channels) <= max_ch:
            return 0.0, 0.0

        throttle = self.normalize_stick(raw_channels[self.throttle_channel])
        steering = self.normalize_stick(raw_channels[self.steering_channel])

        throttle = self.apply_deadzone(throttle)
        steering = self.apply_deadzone(steering)

        linear_cmd = (throttle - 0.5) * 2.0
        angular_cmd = (steering - 0.5) * 2.0

        if self.invert_throttle:
            linear_cmd = -linear_cmd
        if self.invert_steering:
            angular_cmd = -angular_cmd

        return linear_cmd * self.max_linear_vel, angular_cmd * self.max_angular_vel
    
    def publish_cmd_vel(self):
        """
        ИСПРАВЛЕНО (Turn 8): раньше при потере/отсутствии сигнала с пульта
        (`time_diff > timeout_sec`) нода ВСЁ РАВНО публиковала нулевой Twist
        в /cmd_vel/manual — по логике "мультиплексор должен узнать, что пульт
        молчит". Это была ошибка: cmd_mux_node и так считает канал неактивным,
        если в нём давно не было СВЕЖЕГО сообщения (см. timeout_manual в
        cmd_mux_node.py) — публиковать явный "стоп" не нужно, а НАОБОРОТ
        вредно: это держит /cmd_vel/manual вечно "свежим", из-за чего
        физический пульт по приоритету навсегда перебивает управление с
        клавиатуры/джойстика из desktop-приложения (/cmd_vel/app_manual) —
        даже если пульт вообще выключен или ни разу не привязан. Поскольку
        режим по умолчанию при старте ноды — MANUAL, это воспроизводилось
        сразу после каждого перезапуска стека, пока тумблер на пульте не
        переключат хотя бы раз.
        Теперь при отсутствии сигнала узел НЕ публикует вообще — топик
        /cmd_vel/manual сам "стареет", и мультиплексор корректно переключается
        на следующий по приоритету канал (app_manual/home/auto).
        """
        now = self.get_clock().now()
        time_diff = (now - self.last_packet_time).nanoseconds / 1e9

        if time_diff > self.timeout_sec:
            # Запоминаем режим при потере пульта (один раз за «эпизод»):
            # режим не должен меняться самовольно, пока пульт молчит.
            if not self.signal_lost_logged:
                self.signal_lost_logged = True
                self.remembered_mode = self.current_mode
                mode_names = {0: 'AUTO', 1: 'MANUAL', 2: 'AVOID', 3: 'RETURN_HOME'}
                self.get_logger().warn(
                    f'Пульт потерян (нет сигнала {time_diff:.1f}с). '
                    f'Режим запомнен: {mode_names.get(self.current_mode, self.current_mode)}. '
                    'Переключений режима не будет, пока пульт не вернётся.'
                )
            self.get_logger().debug(
                f"No signal for {time_diff:.2f}s. Not publishing to /cmd_vel/manual "
                "(let the mux fail over to the next priority channel)."
            )
            return

        # Публикуем ТОЛЬКО если режим MANUAL И есть живой сигнал с пульта.
        if self.current_mode == ControlModes.MANUAL:
            linear_vel, angular_vel = self.map_channels_to_velocity(self.last_raw_channels)
            twist_msg = Twist()
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = angular_vel
            self.manual_pub.publish(twist_msg)
        # Если режим НЕ ручной (AUTO/AVOID/RETURN_HOME) — не публикуем,
        # управление берёт на себя robot_commander.
    
    def read_serial(self):
        if not self.serial or not self.serial.is_open:
            return
        
        while self.serial.in_waiting > 0:
            try:
                byte = self.serial.read(1)[0] 
                channels = self.parser.process_byte(byte)
                
                if channels is not None:
                    # Восстановление сигнала после потери: применяем режим
                    # с пульта заново (тумблер). Сбрасываем дебаунс, чтобы
                    # check_mode_switch гарантированно перечитал положение.
                    if self.signal_lost_logged:
                        self.signal_lost_logged = False
                        self.debounce_counter = 0
                        self.last_toggle_value = 0.5
                        self.get_logger().info(
                            'Пульт восстановлен — применяю режим с пульта'
                        )
                    # Раздельно храним:
                    #  - "сырые" 11-битные значения (для точной калибровки
                    #    газа/руля через channel_min/center/max);
                    #  - грубо нормализованные raw/2048 (используются только
                    #    для тумблера режима — его пороги 0.33/0.67 рассчитаны
                    #    именно под эту грубую шкалу и трогать не нужно).
                    self.last_raw_channels = list(channels)
                    normalized = [ch / 2048.0 for ch in channels]
                    self.last_normalized_channels = normalized
                    self.last_packet_time = self.get_clock().now()

                    msg = Float32MultiArray()
                    msg.data = normalized
                    self.publisher.publish(msg)
                    
            except Exception as e:
                self.get_logger().error(f'Serial error: {e}')
                break
    
    def get_control_mode(self):
        return self.current_mode

def main(args=None):
    rclpy.init(args=args)
    node = ELRSReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down ELRS Receiver Node...')
    finally:
        if node.serial and node.serial.is_open:
            node.serial.close()
            node.get_logger().info('Serial port closed.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
