#!/usr/bin/env python3
"""
!!! ВНИМАНИЕ: ЭТОТ УЗЕЛ НЕ ИСПОЛЬЗУЕТСЯ В ПРОЕКТЕ И СЛОМАН. !!!

Причины (не запускайте эту ноду вместе с robot_odom):

  1. Публикует nav_msgs/Odometry в ТОТ ЖЕ топик /odom, что и пакет
     robot_odom (реальная колёсная/гусеничная одометрия). Если запустить
     обе ноды одновременно, EKF будет получать вперемешку два разных,
     несовместимых источника — это гарантированно сломает локализацию.
  2. odom_msg.pose.pose.orientation берётся из msg.orientation узла
     mpu6050_node, а тот СОЗНАТЕЛЬНО не считает ориентацию (ставит
     orientation_covariance[0] = -1, "ориентация недоступна") — то есть
     сюда попадает мусор (в старой версии — невалидный нулевой кватернион).
  3. Позиция всегда (0,0,0) — фактически не одометрия, а имитация.
  4. pose.covariance и twist.covariance = все нули. robot_localization
     трактует нулевую дисперсию на диагонали как "не использовать эту
     величину", то есть даже если бы данные были верными, EKF всё
     равно не стал бы их использовать.

Курс (yaw) робота теперь считает compass_node (пакет compass_control,
топик /imu/data, комплементарный фильтр компас+гироскоп) — это и есть
правильная замена той функции, которую пытался выполнять этот узел.

Файл оставлен в репозитории для истории и НЕ подключён ни в один launch-файл.
Рекомендуется либо удалить пакет mpu_odom_pkg из src/, либо не собирать его
(colcon build --packages-skip mpu_odom_pkg).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3


class MpuToOdomNode(Node):
    def __init__(self):
        super().__init__('mpu_to_odom')
        self.get_logger().error(
            'mpu_to_odom ЗАПУЩЕН, НО ЭТОТ УЗЕЛ УСТАРЕЛ И НЕ ДОЛЖЕН ИСПОЛЬЗОВАТЬСЯ. '
            'Он публикует в /odom и может конфликтовать с robot_odom. '
            'См. докстринг файла mpu_to_odom.py.'
        )
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_mpu_DEPRECATED', 10)
        self.get_logger().info(
            'Node mpu_to_odom started (DEPRECATED, publishing to /odom_mpu_DEPRECATED, not /odom)'
        )

    def imu_callback(self, msg: Imu):
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Не копируем msg.orientation — источник (mpu6050_node) явно
        # помечает ориентацию как недоступную (orientation_covariance[0] == -1).
        odom_msg.pose.pose.position = Point(x=0.0, y=0.0, z=0.0)

        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[35] = -1.0  # yaw недоступен
        odom_msg.twist.covariance = [0.0] * 36

        odom_msg.twist.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(
            x=msg.angular_velocity.x,
            y=msg.angular_velocity.y,
            z=msg.angular_velocity.z
        )

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MpuToOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
