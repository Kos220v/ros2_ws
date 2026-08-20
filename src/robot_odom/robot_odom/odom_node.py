#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 нода одометрии, публикующая данные из JointState.

Публикации:
  * /odom (nav_msgs/Odometry)
  * TF odom -> base_link (если publish_tf := true)

Подписки:
  * /joint_states (sensor_msgs/JointState) – положение X/Y и Yaw

Параметры:
  * robot_x_joint   (str) — имя сустава по оси X
  * robot_y_joint   (str) — имя сустава по оси Y
  * robot_yaw_joint (str) — имя сустава для курса (опционально, если не задан — yaw = 0)
  * odom_frame      (str) — кадр одометрии (по умолчанию "odom")
  * base_frame      (str) — кадр робота (по умолчанию "base_link")
  * publish_tf      (bool) — публиковать TF (по умолчанию True)
"""

import math
import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw):
    """Создаёт кватернион из угла рысканья (roll=pitch=0)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)


def wrap_angle(a):
    """Приводит угол к диапазону [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class OdomNode(Node):
    def __init__(self):
        super().__init__("odom_node")

        # --- Параметры ---
        self.declare_parameter("robot_x_joint", "robot_x")
        self.declare_parameter("robot_y_joint", "robot_y")
        self.declare_parameter("robot_yaw_joint", "robot_yaw")   # опционально
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        self.x_name = self.get_parameter("robot_x_joint").value
        self.y_name = self.get_parameter("robot_y_joint").value
        self.yaw_name = self.get_parameter("robot_yaw_joint").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value

        # --- Подписки и издатели ---
        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10
        )
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # --- Состояние одометрии ---
        self.prev_time = None
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = None   # для вычисления угловой скорости

    def _on_joint_states(self, msg: JointState):
        # Извлекаем индексы нужных суставов
        try:
            idx_x = msg.name.index(self.x_name)
            idx_y = msg.name.index(self.y_name)
        except ValueError:
            self.get_logger().warn(f"Суставы '{self.x_name}' или '{self.y_name}' не найдены")
            return

        if len(msg.position) <= max(idx_x, idx_y):
            return

        x = float(msg.position[idx_x])
        y = float(msg.position[idx_y])

        # Получаем yaw, если сустав задан
        yaw = 0.0
        if self.yaw_name:
            try:
                idx_yaw = msg.name.index(self.yaw_name)
                yaw = float(msg.position[idx_yaw])
            except ValueError:
                self.get_logger().warn(f"Сустав '{self.yaw_name}' не найден, yaw = 0")

        # Время (используем штамп сообщения или текущее)
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            now = time.time()

        # Вычисление скоростей
        vx = vy = 0.0
        dt = 0.0
        if self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 1e-6:
                vx = (x - self.prev_x) / dt
                vy = (y - self.prev_y) / dt

        # Угловая скорость
        angular_z = 0.0
        if self.prev_yaw is not None and dt > 1e-6:
            angular_z = wrap_angle(yaw - self.prev_yaw) / dt

        # Сохраняем текущее состояние
        self.prev_time = now
        self.prev_x = x
        self.prev_y = y
        self.prev_yaw = yaw

        # Преобразование скорости из мировой СК в локальную (робота)
        # twist.linear.x, .y — локальные скорости
        local_vx = vx * math.cos(yaw) + vy * math.sin(yaw)
        local_vy = -vx * math.sin(yaw) + vy * math.cos(yaw)

        # --- Формирование сообщения Odometry ---
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_yaw(yaw)

        odom.twist.twist.linear.x = local_vx
        odom.twist.twist.linear.y = local_vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = angular_z

        # Ковариации (константы для примера)
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.05

        self.pub_odom.publish(odom)

        # --- Публикация TF, если включено ---
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()