import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import smbus
import time

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050')

        # Параметры ноды
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('i2c_bus', 1)       # Шина I2C (обычно 1 на Raspberry Pi 4)
        self.declare_parameter('address', 0x68)    # Адрес устройства (0x68 или 0x69)
        self.declare_parameter('publish_frequency', 100.0)
        # ДОБАВЛЕНО: калибровка смещения нуля (bias) гироскопа при старте.
        # Без неё необкалиброванный MPU6050 отдаёт по оси Z (курс) постоянное
        # смещение в единицы град/с. robot_odom смешивает курс из колёс и из
        # гироскопа с весом ~0.98 в пользу гироскопа (публикация на 100 Гц),
        # поэтому даже небольшой bias за 15-20 секунд поездки превращается в
        # десятки градусов паразитного разворота — прямая линия в /odom
        # превращается в дугу/петлю с сильно заниженным итоговым смещением,
        # хотя колёсная одометрия (/joint_states) при этом верна.
        self.declare_parameter('gyro_calibration_samples', 200)

        self.frame_id = self.get_parameter('frame_id').value
        bus_num = self.get_parameter('i2c_bus').value
        addr = self.get_parameter('address').value
        freq = self.get_parameter('publish_frequency').value
        gyro_cal_samples = int(self.get_parameter('gyro_calibration_samples').value)

        self.bus = None
        
        try:
            # Инициализация шины I2C
            self.get_logger().info(f"Opening I2C bus {bus_num}...")
            self.bus = smbus.SMBus(bus_num)
            
            # --- ВАЖНО: Пропускаем проверку WHO_AM_I ---
            # Это предотвращает падение ноды, если чип MPU6500 (ID 0x70) 
            # или если чтение регистра дало сбой из-за помех.
            # MPU6050 и MPU6500 используют одни и те же регистры для данных.
            
            self.get_logger().info(f"Attempting to initialize device at 0x{addr:02X}...")
            
            # Сброс устройства и выход из спящего режима (регистр 0x6B)
            # Значение 0x00: устройство активно, авто-сброс отключен.
            self.bus.write_byte_data(addr, 0x6B, 0x00)
            self.get_logger().info(f"Device initialized successfully at 0x{addr:02X} on bus {bus_num}")
            
        except PermissionError:
            self.get_logger().fatal("Permission denied! You must add user to 'i2c' group and REBOOT.")
            self.get_logger().fatal("Run: sudo usermod -aG i2c $USER && sudo reboot")
            raise RuntimeError("I2C Permission Denied")
        except FileNotFoundError:
            self.get_logger().fatal(f"I2C bus {bus_num} not found!")
            raise RuntimeError("I2C Bus Not Found")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize hardware: {e}")
            raise RuntimeError("Hardware Init Failed")

        self.addr = addr

        # --------------------------------------------------- калибровка гироскопа
        # Робот должен стоять неподвижно во время калибровки (несколько секунд
        # при старте ноды). Усредняем сырые показания и вычитаем их из каждого
        # последующего измерения — иначе постоянный bias гироскопа будет
        # накапливаться в курсе (theta) в robot_odom и портить одометрию даже
        # при идеально верной калибровке колёс.
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self._calibrate_gyro(gyro_cal_samples)

        # Создаем publisher и таймер
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)

        self.get_logger().info("MPU6050 Node is running and publishing to /imu")

    def _calibrate_gyro(self, num_samples):
        if num_samples <= 0:
            self.get_logger().warn("gyro_calibration_samples <= 0, калибровка гироскопа пропущена (bias = 0).")
            return

        self.get_logger().info(
            f"Калибровка гироскопа: {num_samples} замеров. РОБОТ ДОЛЖЕН СТОЯТЬ НЕПОДВИЖНО..."
        )

        GYRO_SCALE = (3.14159265359 / 180.0) / 131.0
        sum_x = sum_y = sum_z = 0.0
        ok_samples = 0

        for _ in range(num_samples):
            gx_raw = self.read_int16(0x43)
            gy_raw = self.read_int16(0x45)
            gz_raw = self.read_int16(0x47)
            sum_x += gx_raw * GYRO_SCALE
            sum_y += gy_raw * GYRO_SCALE
            sum_z += gz_raw * GYRO_SCALE
            ok_samples += 1
            time.sleep(0.005)

        if ok_samples > 0:
            self.gyro_bias_x = sum_x / ok_samples
            self.gyro_bias_y = sum_y / ok_samples
            self.gyro_bias_z = sum_z / ok_samples

        bias_deg_z = self.gyro_bias_z * 180.0 / 3.14159265359
        self.get_logger().info(
            f"Калибровка гироскопа завершена. Bias (рад/с): "
            f"x={self.gyro_bias_x:.5f} y={self.gyro_bias_y:.5f} z={self.gyro_bias_z:.5f} "
            f"(bias_z = {bias_deg_z:.3f} град/с)"
        )
        if abs(bias_deg_z) > 2.0:
            self.get_logger().warn(
                f"Bias по оси Z необычно большой ({bias_deg_z:.2f} град/с) — "
                "либо робот двигали во время калибровки, либо гироскоп сильно "
                "уходит от температуры. Перезапустите ноду, не трогая робота "
                "первые пару секунд после старта."
            )

    def read_int16(self, reg):
        """
        Читает 16-битное знаковое число из двух регистров.
        MPU6050 хранит данные в формате Big Endian (старший байт первым).
        """
        try:
            high = self.bus.read_byte_data(self.addr, reg)
            low = self.bus.read_byte_data(self.addr, reg + 1)
            value = (high << 8) | low
            
            # Преобразование в знаковое число (если бит 15 установлен, вычитаем 65536)
            if value > 32767:
                value -= 65536
            return value
        except Exception as e:
            self.get_logger().debug(f"Read error at reg {reg}: {e}")
            return 0

    def timer_callback(self):
        # --- Чтение Акселерометра (регистры 0x3B - 0x40) ---
        ax_raw = self.read_int16(0x3B)
        ay_raw = self.read_int16(0x3D)
        az_raw = self.read_int16(0x3F)

        # --- Чтение Гироскопа (регистры 0x43 - 0x48) ---
        gx_raw = self.read_int16(0x43)
        gy_raw = self.read_int16(0x45)
        gz_raw = self.read_int16(0x47)

        # Коэффициенты чувствительности (для диапазона +/- 2g и +/- 250 deg/s по умолчанию)
        # Акселерометр: 16384 counts/g
        ACC_SCALE = 9.80665 / 16384.0
        
        # Гироскоп: 131 counts/(deg/s) -> переводим в рад/с
        # 1 deg/s = pi/180 рад/с
        GYRO_SCALE = (3.14159265359 / 180.0) / 131.0

        # Конвертация в СИ
        accel_x = ax_raw * ACC_SCALE
        accel_y = ay_raw * ACC_SCALE
        accel_z = az_raw * ACC_SCALE 
        # Примечание: гравитация уже включена в показания акселерометра. 
        # Не нужно искусственно прибавлять 9.81, иначе данные будут неверными при движении.

        # ДОБАВЛЕНО: вычитаем bias, полученный при калибровке на старте ноды
        # (см. _calibrate_gyro). Без этого курс в robot_odom уходит даже при
        # неподвижном/прямолинейно едущем роботе — см. комментарий в __init__.
        gyro_x = gx_raw * GYRO_SCALE - self.gyro_bias_x
        gyro_y = gy_raw * GYRO_SCALE - self.gyro_bias_y
        gyro_z = gz_raw * GYRO_SCALE - self.gyro_bias_z

        # Формирование сообщения ROS2
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # ИСПРАВЛЕНО: ориентация неизвестна (нет фьюжна), но кватернион
        # по умолчанию (0,0,0,0) математически невалиден (нулевая длина).
        # Явно ставим единичный кватернион, чтобы инструменты (rviz, tf2,
        # другие ноды), которые не проверяют orientation_covariance,
        # не падали и не ругались на "Quaternion has length close to zero".
        # Потребители ОБЯЗАНЫ ориентироваться на orientation_covariance[0] == -1
        # ("ориентация недоступна") и не использовать msg.orientation отсюда —
        # реальный курс публикует compass_node в /imu/data.
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z

        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z

        # Заглушки ковариации (нужны для фильтров Калмана в навигации)
        # [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                               0.0, 0.01, 0.0,
                                               0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.001, 0.0, 0.0,
                                           0.0, 0.001, 0.0,
                                           0.0, 0.0, 0.001]
        
        # Ориентация пока неизвестна (-1.0 означает "недоступно")
        msg.orientation_covariance = [-1.0, -1.0, -1.0, 
                                      -1.0, -1.0, -1.0, 
                                      -1.0, -1.0, -1.0]

        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()