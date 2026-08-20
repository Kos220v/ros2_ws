#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Header


from .mag_driver import Magnetometer


class CompassNode(Node):
    """
    ROS2-узел для чтения данных с магнитометра QMC5883L/HMC5883L
    и публикации их в топик /imu/mag (или пользовательский).
    """

    def __init__(self):
        super().__init__('compass_control')

        # --- Объявление параметров ---
        self.declare_parameter('bus_num', 1)
        self.declare_parameter('mag_yaw_offset_deg', 0.0)
        self.declare_parameter('mag_z_invert', False)
        self.declare_parameter('mag_hard_iron_x', 0.0)
        self.declare_parameter('mag_hard_iron_y', 0.0)
        self.declare_parameter('mag_hard_iron_z', 0.0)
        self.declare_parameter('mag_scale_x', 1.0)
        self.declare_parameter('mag_scale_y', 1.0)
        self.declare_parameter('mag_scale_z', 1.0)
        # Коэффициент перевода LSB -> Тесла.
        #
        # Значение по умолчанию не задано намеренно: если параметр не указан,
        # узел сам определит коэффициент по типу найденной микросхемы
        # (QMC5883L или HMC5883L) — см. блок автоопределения ниже.
        #
        # ParameterDescriptor(dynamic_typing=True) здесь обязателен. Без него
        # объявление параметра со значением None считается устаревшим, и rclpy
        # печатает предупреждение "declaring a parameter only providing its
        # name is deprecated". В будущих версиях ROS это станет ошибкой.
        self.declare_parameter(
            'mag_lsb_to_tesla', None,
            ParameterDescriptor(
                dynamic_typing=True,
                description='Коэффициент LSB -> Тесла. '
                            'Если не задан, определяется автоматически '
                            'по типу микросхемы магнитометра.'))
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 10.0)      # Гц
        self.declare_parameter('topic', '/imu/mag')

        # --- Чтение параметров ---
        bus_num = self.get_parameter('bus_num').value
        mag_yaw_offset_deg = self.get_parameter('mag_yaw_offset_deg').value
        mag_z_invert = self.get_parameter('mag_z_invert').value
        mag_hard_iron_x = self.get_parameter('mag_hard_iron_x').value
        mag_hard_iron_y = self.get_parameter('mag_hard_iron_y').value
        mag_hard_iron_z = self.get_parameter('mag_hard_iron_z').value
        mag_scale_x = self.get_parameter('mag_scale_x').value
        mag_scale_y = self.get_parameter('mag_scale_y').value
        mag_scale_z = self.get_parameter('mag_scale_z').value
        self.lsb_to_tesla = self.get_parameter('mag_lsb_to_tesla').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        topic = self.get_parameter('topic').value

        # --- Создание издателя ---
        self.publisher = self.create_publisher(MagneticField, topic, 10)

        # --- Инициализация магнитометра ---
        try:
            self.mag = Magnetometer(
                bus_num=bus_num,
                logger=self.get_logger(),
                mag_yaw_offset_deg=mag_yaw_offset_deg,
                mag_z_invert=mag_z_invert,
                mag_hard_iron_x=mag_hard_iron_x,
                mag_hard_iron_y=mag_hard_iron_y,
                mag_hard_iron_z=mag_hard_iron_z,
                mag_scale_x=mag_scale_x,
                mag_scale_y=mag_scale_y,
                mag_scale_z=mag_scale_z,
            )
        except Exception as e:
            self.get_logger().error(f"Не удалось инициализировать магнитометр: {e}")
            self.mag = None

        # --- Автоматический выбор коэффициента перевода LSB -> Тесла ---
        if self.mag is not None and self.mag.mag_type is not None:
            if self.lsb_to_tesla is None:
                if self.mag.mag_type == 'QMC':
                    # QMC5883L при диапазоне ±8G: 1 LSB = 0.03 μT = 3e-8 T
                    self.lsb_to_tesla = 3e-8
                    self.get_logger().info("Автоопределён коэффициент для QMC: 3e-8 T/LSB")
                elif self.mag.mag_type == 'HMC':
                    # HMC5883L при gain=0 (1370 LSB/G): 1 LSB = (1/1370) G = 7.29927e-8 T
                    self.lsb_to_tesla = 7.29927e-8
                    self.get_logger().info("Автоопределён коэффициент для HMC: 7.29927e-8 T/LSB")
                else:
                    self.lsb_to_tesla = 1e-6  # fallback
                    self.get_logger().warning("Неизвестный тип магнитометра, использую 1e-6 T/LSB")
            else:
                self.get_logger().info(f"Используется пользовательский коэффициент: {self.lsb_to_tesla} T/LSB")
        else:
            self.lsb_to_tesla = 1.0  # если магнитометр не инициализирован, чтобы не было ошибок
            self.get_logger().warning("Магнитометр не инициализирован, публикация невозможна")

        # --- Таймер для периодического опроса ---
        if publish_rate > 0.0:
            period = 1.0 / publish_rate
            self.timer = self.create_timer(period, self.timer_callback)
            self.get_logger().info(f"Запущен таймер с частотой {publish_rate} Гц")
        else:
            self.timer = None
            self.get_logger().warning("publish_rate <= 0, публикация отключена")

    def timer_callback(self):
        """Периодически читает магнитометр и публикует сообщение."""
        if self.mag is None:
            return

        mag_raw = self.mag.read_mag()  # вектор в LSB

        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Преобразование в Тесла
        msg.magnetic_field.x = mag_raw[0] * self.lsb_to_tesla
        msg.magnetic_field.y = mag_raw[1] * self.lsb_to_tesla
        msg.magnetic_field.z = mag_raw[2] * self.lsb_to_tesla

        self.publisher.publish(msg)

    def destroy_node(self):
        """Закрывает I2C-шину при завершении."""
        if self.mag is not None:
            self.mag.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CompassNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()