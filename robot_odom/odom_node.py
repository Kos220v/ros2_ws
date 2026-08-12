#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState, NavSatFix, Imu
import math

class EnhancedOdometryNode(Node):
    def __init__(self):
        super().__init__('enhanced_odometry_node')

        # --------------------------------------------------------- параметры
        self.declare_parameter('wheel_base', 0.48)
        self.declare_parameter('wheel_radius', 0.0723)
        self.declare_parameter('left_wheel_joint', 'left_track_joint')
        self.declare_parameter('right_wheel_joint', 'right_track_joint')
        
        # ВАЖНО: False! Коррекцию делает EKF, а не этот узел.
        self.declare_parameter('use_gps_correction', False) 
        
        self.declare_parameter('gps_alpha', 0.1)
        
        # ВАЖНО: False! TF публикует EKF.
        self.declare_parameter('publish_tf', False) 
        
        self.declare_parameter('odom_rate', 10.0)
        self.declare_parameter('use_imu_angular_velocity', True)
        self.declare_parameter('imu_topic', '/imu')
        # ДОБАВЛЕНО: раньше вес гироскопа в смешивании курса доходил до ~0.98
        # (IMU публикует на 100 Гц, imu_dt~0.01с -> weight=1-0.01/0.5=0.98),
        # то есть курс считался почти целиком по сырому гироскопу и почти не
        # по колёсам. Колёсная одометрия проверена экспериментально и точна;
        # гироскоп же реагирует на любую вибрацию/тряску на реальном грунте
        # (особенно если IMU не установлена строго горизонтально — тогда часть
        # крена/тангажа от тряски попадает в ось Z). Из-за доминирования IMU
        # курс "гулял" в середине заезда и почти возвращался к 0 к концу —
        # путь по тикам большой, а итоговое x/y в /odom получалось маленьким.
        # Ограничиваем вес IMU сверху — курс в основном по колёсам, IMU только
        # слегка сглаживает.
        self.declare_parameter('imu_omega_max_weight', 0.3)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.left_joint = self.get_parameter('left_wheel_joint').value
        self.right_joint = self.get_parameter('right_wheel_joint').value
        self.use_gps = self.get_parameter('use_gps_correction').value
        self.gps_alpha = self.get_parameter('gps_alpha').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_rate = self.get_parameter('odom_rate').value
        self.use_imu_omega = self.get_parameter('use_imu_angular_velocity').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.imu_omega_max_weight = float(self.get_parameter('imu_omega_max_weight').value)

        # --------------------------------------------------- состояние одометрии
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.last_angular_vel = 0.0
        self.last_angular_vel_time = None
        self.angular_acceleration = 0.0

        self.last_linear_vel = 0.0
        self.last_vel_time = None

        # GPS данные (только хранение)
        self.gps_x = 0.0
        self.gps_y = 0.0
        self.gps_initialized = False
        self.gps_initial_x = 0.0
        self.gps_initial_y = 0.0
        self.prev_gps_x = 0.0
        self.prev_gps_y = 0.0

        # Wheel odometry state
        self.last_left_angle = None
        self.last_right_angle = None
        self.last_time = None

        # IMU state
        self.last_imu_time = None
        self.last_imu_angular_z = None

        # --------------------------------------------------- QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --------------------------------------------------- подписки
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, qos_profile
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, qos_profile
        )
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, qos_profile
        )

        # --------------------------------------------------- публикации
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TransformBroadcaster УДАЛЕН. TF публикует только EKF.

        # --------------------------------------------------- таймер
        self.timer = self.create_timer(1.0 / self.odom_rate, self.publish_odometry_timer)

        self.get_logger().info('Enhanced Odometry Node started (Pure Odom Mode)')
        self.get_logger().info(f'  wheel_base={self.wheel_base}, wheel_radius={self.wheel_radius}')
        self.get_logger().info(f'  GPS correction enabled: {self.use_gps} (Must be False)')
        self.get_logger().info(f'  TF publishing enabled: {self.publish_tf} (Must be False)')

    # ------------------------------------------------------------ joint_states
    def joint_callback(self, msg):
        try:
            left_idx = msg.name.index(self.left_joint)
            right_idx = msg.name.index(self.right_joint)
        except ValueError:
            self.get_logger().warn(
                f'Joint {self.left_joint} or {self.right_joint} not found in: {msg.name}',
                throttle_duration_sec=5.0
            )
            return

        current_left = msg.position[left_idx]
        current_right = msg.position[right_idx]
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            self.last_left_angle = current_left
            self.last_right_angle = current_right
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.5:
            self.last_time = current_time
            self.last_left_angle = current_left
            self.last_right_angle = current_right
            return

        delta_left = (current_left - self.last_left_angle) * self.wheel_radius
        delta_right = (current_right - self.last_right_angle) * self.wheel_radius

        v_left = delta_left / dt
        v_right = delta_right / dt

        v = (v_right + v_left) / 2.0
        omega_wheel = (v_right - v_left) / self.wheel_base

        # УБРАНО: фильтр "защиты от проскальзывания" по ускорению между
        # соседними /joint_states. При разомкнутом duty-cycle управлении
        # (kolesa_control) реальная скорость дёргается рывками (особенно на
        # страгивании с места), это регулярно превышало порог max_accel=2.0
        # м/с^2 и НЕ было реальным проскальзыванием — фильтр просто замораживал
        # v на предыдущем (часто нулевом) значении почти на каждом цикле,
        # из-за чего пройденная дистанция в /odom занижалась в разы (проверено
        # экспериментально: 2.12 м реального пробега при ~1.95 м по тикам
        # kolesa_control, но всего ~0.06 м в /odom до удаления фильтра).
        # Анти-выбросная защита по самим тикам тахометра уже есть в
        # kolesa_control (_tacho_delta_is_reasonable) — она осмысленнее
        # (работает с сырыми тиками, а не с дважды продифференцированной
        # скоростью) и её достаточно.
        self.last_linear_vel = v
        self.last_vel_time = current_time

        # Смешивание IMU и колёс для угловой скорости.
        # ИСПРАВЛЕНО: раньше omega считалась безусловно как
        # weight_imu * self.last_imu_angular_z + ..., даже когда
        # last_imu_angular_z ещё None (ни одного сообщения /imu не пришло) —
        # 0.0 * None бросает TypeError и роняет ноду. Теперь блендинг с IMU
        # включается только если реально есть свежие данные гироскопа.
        omega = omega_wheel
        weight_imu = 0.0
        if self.use_imu_omega and self.last_imu_time is not None and self.last_imu_angular_z is not None:
            imu_dt = (current_time - self.last_imu_time).nanoseconds / 1e9
            if imu_dt <= 0.5:
                weight_imu = max(0.0, 1.0 - imu_dt / 0.5)
                # ОГРАНИЧЕНО: не даём гироскопу доминировать над колёсами
                # (см. комментарий у declare_parameter imu_omega_max_weight).
                weight_imu = min(weight_imu, self.imu_omega_max_weight)
                omega = weight_imu * self.last_imu_angular_z + (1.0 - weight_imu) * omega_wheel

        # Вычисление углового ускорения
        if self.last_angular_vel_time is not None and self.last_imu_time is not None:
            omega_dt = (current_time - self.last_angular_vel_time).nanoseconds / 1e9
            if omega_dt > 0:
                raw_alpha = (omega - self.last_angular_vel) / omega_dt
                alpha_smooth = 0.2
                self.angular_acceleration = alpha_smooth * raw_alpha + (1.0 - alpha_smooth) * self.angular_acceleration
        else:
            self.angular_acceleration = 0.0

        self.last_angular_vel = omega
        self.last_angular_vel_time = current_time

        # Интегрирование
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Блок коррекции GPS теперь никогда не сработает (use_gps=False)
        if self.use_gps and self.gps_initialized:
            self.apply_gps_correction()

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.linear_vel = v
        self.angular_vel = omega

        self.last_time = current_time
        self.last_left_angle = current_left
        self.last_right_angle = current_right

    # ------------------------------------------------------------ IMU
    def imu_callback(self, msg: Imu):
        self.last_imu_angular_z = msg.angular_velocity.z
        self.last_imu_time = self.get_clock().now()

    # ------------------------------------------------------------ GPS
    def gps_callback(self, msg: NavSatFix):
        if msg.status.status == -1:
            return

        current_gps_x = msg.longitude * 111320 * math.cos(math.radians(msg.latitude))
        current_gps_y = msg.latitude * 110540

        if not self.gps_initialized:
            self.gps_initial_x = current_gps_x
            self.gps_initial_y = current_gps_y
            self.gps_x = 0.0
            self.gps_y = 0.0
            self.prev_gps_x = 0.0
            self.prev_gps_y = 0.0
            self.gps_initialized = True
            self.get_logger().info('GPS initialized (data stored, not applied)')
        else:
            dist = math.hypot(current_gps_x - self.gps_initial_x - self.prev_gps_x,
                              current_gps_y - self.gps_initial_y - self.prev_gps_y)
            if dist > 5.0:  # защита от выбросов
                return

            self.prev_gps_x = self.gps_x
            self.prev_gps_y = self.gps_y
            self.gps_x = (current_gps_x - self.gps_initial_x)
            self.gps_y = (current_gps_y - self.gps_initial_y)

    def apply_gps_correction(self):
        """Эта функция мертва из-за параметра use_gps_correction=False"""
        self.x = (1 - self.gps_alpha) * self.x + self.gps_alpha * self.gps_x
        self.y = (1 - self.gps_alpha) * self.y + self.gps_alpha * self.gps_y

    # ------------------------------------------------------------ публикация
    def publish_odometry_timer(self):
        if self.last_time is None:
            return

        current_time = self.get_clock().now()
        age = (current_time - self.last_time).nanoseconds / 1e9

        if age > 0.5:
            linear_vel = 0.0
            angular_vel = 0.0
        else:
            linear_vel = self.linear_vel
            angular_vel = self.angular_vel

        self.publish_odometry(current_time, linear_vel, angular_vel)

    def publish_odometry(self, stamp, linear_vel, angular_vel):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = self.euler_to_quaternion(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        pos_cov = 0.1 
        ori_cov = 0.05

        odom_msg.pose.covariance = [
            pos_cov, 0.0,     0.0,     0.0,     0.0,     0.0,
            0.0,     pos_cov, 0.0,     0.0,     0.0,     0.0,
            0.0,     0.0,     pos_cov, 0.0,     0.0,     0.0,
            0.0,     0.0,     0.0,     ori_cov, 0.0,     0.0,
            0.0,     0.0,     0.0,     0.0,     ori_cov, 0.0,
            0.0,     0.0,     0.0,     0.0,     0.0,     ori_cov,
        ]

        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.angular.z = angular_vel

        vel_cov = 0.01
        odom_msg.twist.covariance = [
            vel_cov, 0.0,     0.0,     0.0,     0.0,     0.0,
            0.0,     vel_cov, 0.0,     0.0,     0.0,     0.0,
            0.0,     0.0,     vel_cov, 0.0,     0.0,     0.0,
            0.0,     0.0,     0.0,     0.01,    0.0,     0.0,
            0.0,     0.0,     0.0,     0.0,     0.01,    0.0,
            0.0,     0.0,     0.0,     0.0,     0.0,     vel_cov
        ]

        self.odom_pub.publish(odom_msg)
        # TF НЕ публикуется здесь!

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]

# ==============================================================================
# ТОЧКА ВХОДА В ПРОГРАММУ
# ==============================================================================

def main(args=None):
    """
    Стандартная точка входа для ROS2 узла.
    Инициализирует rclpy, создает узел и запускает цикл обработки событий.
    """
    rclpy.init(args=args)
    
    node = EnhancedOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

