# -*- coding: utf-8 -*-

"""
Нода kolesa_control

Дифференциальное управление гусеничным роботом через два контроллера
FS75100 / VESC по UART.

Подписки:
  /cmd_vel            geometry_msgs/Twist

Публикации:
  /joint_states       sensor_msgs/JointState
      Плоская поза робота: position = [X, Y, yaw], где X/Y заданы в
      метрах, yaw — в радианах. Имена полей задаются параметрами
      robot_x_joint, robot_y_joint и robot_yaw_joint.
  /kolesa/diagnostics diagnostic_msgs/DiagnosticArray
      Содержит в том числе текущий угол поворота.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .vesc_driver import VescDriver


def velocity_to_duty_cycle(velocity, max_velocity, duty_min=0.03, duty_max=1.0):
    """
    Преобразует целевую скорость в скважность (duty cycle) для VESC.

    ВАЖНО: Эта функция ожидает, что направление уже учтено в знаке velocity.
    Она НЕ должна знать про настройки invert_left/right.
    """
    if abs(velocity) < 0.001:
        return 0.0
    normalized = abs(velocity) / max_velocity
    normalized = min(1.0, max(0.0, normalized))
    duty_magnitude = duty_min + normalized * (duty_max - duty_min)
    return duty_magnitude if velocity >= 0 else -duty_magnitude


def delta_i32(current, previous):
    """Разница двух int32-счетчиков с учетом переполнения."""
    current = int(current)
    previous = int(previous)
    delta = current - previous
    if delta > 2147483647:
        delta -= 4294967296
    elif delta < -2147483648:
        delta += 4294967296
    return delta


def normalize_angle(angle):
    """Приводит угол (рад) к диапазону (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class KolesaControl(Node):
    def __init__(self):
        super().__init__("kolesa_control")

        # ----------------------------------------------------------- параметры
        p = self.declare_parameter

        p("left_port", "/dev/ttyAMA4")
        p("right_port", "/dev/ttyAMA5")
        p("baud", 115200)

        # Геометрия шасси
        p("wheel_separation", 0.55)

        # Калибровка одометрии по оборотам выходного вала
        p("tacho_counts_per_revolution", 2157.0)
        p("distance_per_revolution", 2.011)
        p("odometry_scale", 1.0)
        p("turn_counts_per_rad_left", 384.16)
        p("turn_counts_per_rad_right", 344.73)

        # Инверсии
        p("invert_left", False)
        p("invert_right", False)
        p("encoder_invert_left", False)
        p("encoder_invert_right", True)
        p("invert_angular", False)

        # Скважность (duty cycle) VESC
        p("duty_min", 0.03)
        p("duty_max", 1.0)
        p("max_linear_velocity", 1.0)
        p("max_angular_velocity", 1.0)
        p("control_rate", 50.0)
        p("telemetry_rate", 20.0)
        p("cmd_timeout", 0.5)
        p("telemetry_stale_timeout", 0.5)

        # Фильтрация скачков тахометра
        p("tacho_jump_margin", 3.0)
        p("min_tacho_jump_threshold", 500.0)

        # Публикация и joint states
        p("publish_joint_states", True)
        p("robot_x_joint", "robot_x")
        p("robot_y_joint", "robot_y")
        p("robot_yaw_joint", "robot_yaw")
        p("publish_diagnostics", True)

        g = self.get_parameter

        self.left_port = str(g("left_port").value)
        self.right_port = str(g("right_port").value)
        self.baud = int(g("baud").value)

        self.separation = float(g("wheel_separation").value)
        self.tacho_counts_per_revolution = float(g("tacho_counts_per_revolution").value)
        self.distance_per_revolution = float(g("distance_per_revolution").value)
        self.odometry_scale = float(g("odometry_scale").value)

        self.radius = self.distance_per_revolution / (2.0 * math.pi)

        self.turn_counts_per_rad_left = float(g("turn_counts_per_rad_left").value)
        self.turn_counts_per_rad_right = float(g("turn_counts_per_rad_right").value)

        self.kin_inv_left = -1 if g("invert_left").value else 1
        self.kin_inv_right = -1 if g("invert_right").value else 1
        self.enc_inv_left = -1 if g("encoder_invert_left").value else 1
        self.enc_inv_right = -1 if g("encoder_invert_right").value else 1
        self.invert_angular = bool(g("invert_angular").value)

        self.duty_min = float(g("duty_min").value)
        self.duty_max = float(g("duty_max").value)
        self.max_linear_velocity = float(g("max_linear_velocity").value)
        self.max_angular_velocity = float(g("max_angular_velocity").value)
        self.control_rate = float(g("control_rate").value)
        self.telemetry_rate = float(g("telemetry_rate").value)
        self.cmd_timeout = float(g("cmd_timeout").value)
        self.telemetry_stale_timeout = float(g("telemetry_stale_timeout").value)
        self.tacho_jump_margin = float(g("tacho_jump_margin").value)
        self.min_tacho_jump_threshold = float(g("min_tacho_jump_threshold").value)

        self.pub_js = bool(g("publish_joint_states").value)
        self.pub_diag = bool(g("publish_diagnostics").value)
        self.robot_x_joint = str(g("robot_x_joint").value)
        self.robot_y_joint = str(g("robot_y_joint").value)
        self.robot_yaw_joint = str(g("robot_yaw_joint").value)

        self._validate_params()

        self.distance_per_tacho_count = (
            self.distance_per_revolution / self.tacho_counts_per_revolution
        )
        self.rad_per_tacho_count = (2.0 * math.pi) / self.tacho_counts_per_revolution

        self.get_logger().info("=" * 60)
        self.get_logger().info("КОНФИГУРАЦИЯ ПРИВОДА (DUTY CYCLE)")
        self.get_logger().info(f"  Радиус: {self.radius:.4f} м | База: {self.separation:.4f} м")
        self.get_logger().info(f"  Инверсия L/R: {self.kin_inv_left}/{self.kin_inv_right} | "
                               f"invert_angular: {self.invert_angular}")
        self.get_logger().info(f"  Макс. линейная: {self.max_linear_velocity:.2f} м/с")
        self.get_logger().info(f"  Скважность: min={self.duty_min:.3f} max={self.duty_max:.3f}")
        self.get_logger().info(
            f"  Одометрия: {self.tacho_counts_per_revolution:.2f} тиков/оборот, "
            f"{self.distance_per_revolution:.4f} м/оборот "
            f"({self.distance_per_tacho_count * 1000.0:.5f} мм/тик)"
        )
        self.get_logger().info(
            f"  Поворот (yaw): {self.turn_counts_per_rad_left:.2f} тиков/рад (левое), "
            f"{self.turn_counts_per_rad_right:.2f} тиков/рад (правое)"
        )
        self.get_logger().info("  Логика: pos/distance — абсолютный тахометр, speed/omega — дельты.")
        self.get_logger().info("=" * 60)

        # ----------------------------------------------------------- драйверы
        self.left = VescDriver("left", self.left_port, self.baud, self.get_logger())
        self.right = VescDriver("right", self.right_port, self.baud, self.get_logger())
        self.left.start()
        self.right.start()

        # ----------------------------------------------------------- состояние
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.theta_z = None
        self._pose_prev_offsets = None

        self.wheels = {
            "left": self._make_wheel_state(),
            "right": self._make_wheel_state(),
        }
        self._warned_no_tacho = {"left": False, "right": False}

        # ----------------------------------------------------------- топики
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        if self.pub_js:
            self.js_pub = self.create_publisher(JointState, "joint_states", 10)
        if self.pub_diag:
            self.diag_pub = self.create_publisher(
                DiagnosticArray, "kolesa/diagnostics", qos_profile_sensor_data,
            )

        # ----------------------------------------------------------- таймеры
        self.create_timer(1.0 / self.control_rate, self._control_tick)
        self.create_timer(1.0 / self.telemetry_rate, self._telemetry_tick)

        self.get_logger().info("kolesa_control запущена (абсолютный тахометр)")

    # ------------------------------------------------------------- параметры
    def _validate_params(self):
        if (
            self.radius <= 0.0
            or self.separation <= 0.0
            or self.control_rate <= 0.0
            or self.telemetry_rate <= 0.0
            or self.max_linear_velocity <= 0.0
            or self.max_angular_velocity <= 0.0
        ):
            raise ValueError("Некорректные параметры конфигурации геометрии или скорости")
        if self.tacho_counts_per_revolution <= 0.0:
            raise ValueError("tacho_counts_per_revolution должен быть > 0")
        if self.distance_per_revolution <= 0.0:
            raise ValueError("distance_per_revolution должен быть > 0")
        if self.duty_min < 0.0 or self.duty_max <= 0.0 or self.duty_min >= self.duty_max:
            raise ValueError("Некорректные duty_min/duty_max (должно быть 0 <= duty_min < duty_max)")
        if self.duty_max > 1.0:
            raise ValueError("duty_max не может быть больше 1.0 (100% скважности)")
        if self.turn_counts_per_rad_left <= 0.0 or self.turn_counts_per_rad_right <= 0.0:
            raise ValueError(
                "turn_counts_per_rad_left/right должны быть > 0 "
                "(калибровка поворота вокруг оси Z)"
            )

    def _make_wheel_state(self):
        return {
            "pos": 0.0, "distance": 0.0, "speed": 0.0, "omega": 0.0,
            "raw_tacho": None, "raw_tacho_abs": None,
            "prev_tacho": None,
            "initial_tacho": None,
            "total_abs_counts": 0,
            "turn_offset_ticks": 0.0,
            "last_delta_counts": 0, "erpm": 0.0,
            "duty_measured": 0.0, "duty_target": 0.0,
            "last_rx_time": None, "last_tacho_time": None,
            "telemetry_age": float("inf"), "stale": True,
        }

    # ------------------------------------------------------------- callbacks
    def _on_cmd_vel(self, msg: Twist):
        self.cmd_v = float(msg.linear.x)
        self.cmd_w = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def _control_tick(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9

        if dt > self.cmd_timeout:
            v = 0.0
            w = 0.0
        else:
            v = self.cmd_v
            w = self.cmd_w

        v = max(-self.max_linear_velocity, min(self.max_linear_velocity, v))
        w = max(-self.max_angular_velocity, min(self.max_angular_velocity, w))

        if self.invert_angular:
            w = -w

        half_sep = self.separation / 2.0

        v_left_target = v - (w * half_sep)
        v_right_target = v + (w * half_sep)

        v_left_final = v_left_target * self.kin_inv_left
        v_right_final = v_right_target * self.kin_inv_right

        duty_left = velocity_to_duty_cycle(
            v_left_final,
            self.max_linear_velocity,
            self.duty_min,
            self.duty_max,
        )
        duty_right = velocity_to_duty_cycle(
            v_right_final,
            self.max_linear_velocity,
            self.duty_min,
            self.duty_max,
        )

        self.wheels["left"]["duty_target"] = duty_left
        self.wheels["right"]["duty_target"] = duty_right

        self.left.set_duty(duty_left)
        self.right.set_duty(duty_right)

    def _telemetry_tick(self):
        self.left.request_telemetry()
        self.right.request_telemetry()

        tl = self.left.get_telemetry()
        tr = self.right.get_telemetry()

        self._update_wheel_from_tacho("left", tl, direction_sign=self.enc_inv_left)
        self._update_wheel_from_tacho("right", tr, direction_sign=self.enc_inv_right)
        self._update_stale_state("left")
        self._update_stale_state("right")
        self._update_robot_pose()

        if self.pub_js:
            self._publish_joint_states()
        if self.pub_diag:
            self._publish_diagnostics()

    def _update_wheel_from_tacho(self, side, telemetry, direction_sign):
        st = self.wheels[side]

        if telemetry is None:
            return

        if "tachometer" not in telemetry:
            if not self._warned_no_tacho[side]:
                self.get_logger().warning(
                    f"[{side}] в телеметрии нет поля 'tachometer'."
                )
                self._warned_no_tacho[side] = True
            return

        self._warned_no_tacho[side] = False

        rx_time_raw = telemetry.get("_rx_time", None)
        if rx_time_raw is not None:
            rx_time = float(rx_time_raw)
            if st["last_rx_time"] == rx_time:
                return
        else:
            rx_time = time.monotonic()

        st["last_rx_time"] = rx_time
        st["telemetry_age"] = 0.0
        st["stale"] = False

        current_tacho = int(telemetry["tachometer"])
        st["raw_tacho"] = current_tacho

        if "tachometer_abs" in telemetry:
            st["raw_tacho_abs"] = int(telemetry["tachometer_abs"])
        elif "tacho_abs" in telemetry:
            st["raw_tacho_abs"] = int(telemetry["tacho_abs"])

        raw_erpm = telemetry.get("erpm", telemetry.get("rpm", 0.0))
        st["erpm"] = float(raw_erpm)
        st["duty_measured"] = float(telemetry.get("duty", 0.0))

        if st["initial_tacho"] is None:
            st["initial_tacho"] = current_tacho
            st["prev_tacho"] = current_tacho
            st["last_tacho_time"] = rx_time
            st["speed"] = 0.0
            st["omega"] = 0.0
            st["last_delta_counts"] = 0
            st["turn_offset_ticks"] = 0.0
            self.get_logger().info(f"[{side}] initial_tacho = {current_tacho}")
            return

        offset = delta_i32(current_tacho, st["initial_tacho"]) * direction_sign
        st["pos"] = offset * self.rad_per_tacho_count * self.odometry_scale
        st["distance"] = offset * self.distance_per_tacho_count * self.odometry_scale
        st["turn_offset_ticks"] = float(offset)

        dt = rx_time - st["last_tacho_time"]
        delta_raw = delta_i32(current_tacho, st["prev_tacho"])
        st["prev_tacho"] = current_tacho
        st["last_tacho_time"] = rx_time

        if not self._tacho_delta_is_reasonable(delta_raw, dt):
            self.get_logger().warning(
                f"[{side}] подозрительный скачок tachometer: "
                f"delta={delta_raw}, dt={dt:.3f} c."
            )
            st["speed"] = 0.0
            st["omega"] = 0.0
            st["last_delta_counts"] = 0
            return

        delta = delta_raw * direction_sign
        st["last_delta_counts"] = delta
        st["total_abs_counts"] += abs(delta_raw)

        if dt > 1e-6:
            distance_delta = delta * self.distance_per_tacho_count * self.odometry_scale
            angle_delta = delta * self.rad_per_tacho_count * self.odometry_scale
            st["omega"] = angle_delta / dt
            st["speed"] = distance_delta / dt
        else:
            st["omega"] = 0.0
            st["speed"] = 0.0

    def _tacho_delta_is_reasonable(self, delta_counts, dt):
        if dt <= 0.0:
            return True
        min_dt = 1.0 / self.telemetry_rate
        max_counts_per_sec = self.max_linear_velocity / self.distance_per_tacho_count
        threshold = max(
            self.min_tacho_jump_threshold,
            max_counts_per_sec * max(dt, min_dt) * self.tacho_jump_margin,
        )
        return abs(delta_counts) <= threshold

    def _update_stale_state(self, side):
        st = self.wheels[side]
        if st["last_rx_time"] is None:
            st["telemetry_age"] = float("inf")
            st["stale"] = True
            st["speed"] = 0.0
            st["omega"] = 0.0
            st["erpm"] = 0.0
            st["duty_measured"] = 0.0
            return

        age = time.monotonic() - st["last_rx_time"]
        st["telemetry_age"] = age
        if age > self.telemetry_stale_timeout:
            st["stale"] = True
            st["speed"] = 0.0
            st["omega"] = 0.0
            st["erpm"] = 0.0
            st["duty_measured"] = 0.0
        else:
            st["stale"] = False

    def _update_robot_pose(self):
        """Интегрирует X, Y и yaw по эмпирической калибровке тиков обоих бортов."""
        left = self.wheels["left"]
        right = self.wheels["right"]

        if (
            left["initial_tacho"] is None
            or right["initial_tacho"] is None
            or left["stale"]
            or right["stale"]
        ):
            self._pose_prev_offsets = None
            return

        current_offsets = (
            left["turn_offset_ticks"],
            right["turn_offset_ticks"],
        )

        if self._pose_prev_offsets is None:
            self._pose_prev_offsets = current_offsets
            self.theta_z = self.robot_yaw
            return

        delta_left = current_offsets[0] - self._pose_prev_offsets[0]
        delta_right = current_offsets[1] - self._pose_prev_offsets[1]
        self._pose_prev_offsets = current_offsets

        distance_left = (
            delta_left * self.distance_per_tacho_count * self.odometry_scale
        )
        distance_right = (
            delta_right * self.distance_per_tacho_count * self.odometry_scale
        )
        distance_center = (distance_left + distance_right) / 2.0

        # ИСПРАВЛЕНИЕ: инвертирован знак d_theta для ROS-совместимости
        d_theta = 0.5 * (
            (delta_left / self.turn_counts_per_rad_left)
            - (delta_right / self.turn_counts_per_rad_right)
        )

        mid_theta = self.robot_yaw + d_theta / 2.0
        self.robot_x += distance_center * math.cos(mid_theta)
        self.robot_y += distance_center * math.sin(mid_theta)
        self.robot_yaw += d_theta
        self.robot_yaw = normalize_angle(self.robot_yaw)
        self.theta_z = self.robot_yaw

    # ------------------------------------------------------------- публикации
    def _publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            self.robot_x_joint,
            self.robot_y_joint,
            self.robot_yaw_joint,
        ]
        js.position = [
            self.robot_x,
            self.robot_y,
            normalize_angle(self.robot_yaw),
        ]
        self.js_pub.publish(js)

    def _publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        self._append_wheel_diag(arr, "left", "Левая гусеница", self.left)
        self._append_wheel_diag(arr, "right", "Правая гусеница", self.right)
        self._append_yaw_diag(arr)
        self.diag_pub.publish(arr)

    def _append_wheel_diag(self, arr, side, display_name, driver):
        st = self.wheels[side]
        status = DiagnosticStatus()
        status.name = display_name
        status.hardware_id = driver.port

        if not driver.connected:
            status.level = DiagnosticStatus.WARN
            status.message = "Нет соединения с VESC"
        elif st["stale"]:
            status.level = DiagnosticStatus.WARN
            status.message = "Телеметрия устарела"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Норма"

        total_abs_revolutions = (
            st["total_abs_counts"] / self.tacho_counts_per_revolution
        )

        status.values.append(KeyValue(key="connected", value=str(driver.connected)))
        status.values.append(KeyValue(
            key="telemetry_age_s",
            value="inf" if math.isinf(st["telemetry_age"]) else f"{st['telemetry_age']:.3f}",
        ))
        status.values.append(KeyValue(key="raw_tachometer", value=str(st["raw_tacho"])))
        status.values.append(KeyValue(key="raw_tachometer_abs", value=str(st["raw_tacho_abs"])))
        status.values.append(KeyValue(key="initial_tacho", value=str(st["initial_tacho"])))
        status.values.append(KeyValue(key="last_delta_counts", value=str(st["last_delta_counts"])))
        status.values.append(KeyValue(key="total_abs_counts", value=str(st["total_abs_counts"])))
        status.values.append(KeyValue(key="total_abs_revolutions", value=f"{total_abs_revolutions:.2f}"))
        status.values.append(KeyValue(key="turn_offset_ticks", value=f"{st['turn_offset_ticks']:.1f}"))
        status.values.append(KeyValue(key="duty_target", value=f"{st['duty_target']:.3f}"))
        status.values.append(KeyValue(key="duty_measured", value=f"{st['duty_measured']:.3f}"))
        status.values.append(KeyValue(key="erpm", value=f"{st['erpm']:.1f}"))
        status.values.append(KeyValue(key="speed_m_s", value=f"{st['speed']:.3f}"))
        status.values.append(KeyValue(key="omega_rad_s", value=f"{st['omega']:.3f}"))
        arr.status.append(status)

    def _append_yaw_diag(self, arr):
        status = DiagnosticStatus()
        status.name = "Поза робота (X, Y, yaw)"
        status.hardware_id = "kolesa_control/pose"

        if self.theta_z is None:
            status.level = DiagnosticStatus.WARN
            status.message = "Нет данных (ожидание телеметрии обоих колёс)"
            theta_raw = 0.0
            theta_wrapped = 0.0
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Норма"
            theta_raw = self.theta_z
            theta_wrapped = normalize_angle(self.theta_z)

        status.values.append(KeyValue(key="robot_x_m", value=f"{self.robot_x:.4f}"))
        status.values.append(KeyValue(key="robot_y_m", value=f"{self.robot_y:.4f}"))
        status.values.append(KeyValue(key="theta_z_raw_rad", value=f"{theta_raw:.4f}"))
        status.values.append(KeyValue(key="theta_z_rad", value=f"{theta_wrapped:.4f}"))
        status.values.append(KeyValue(key="theta_z_deg", value=f"{math.degrees(theta_wrapped):.2f}"))
        status.values.append(KeyValue(
            key="turn_counts_per_rad_left", value=f"{self.turn_counts_per_rad_left:.2f}",
        ))
        status.values.append(KeyValue(
            key="turn_counts_per_rad_right", value=f"{self.turn_counts_per_rad_right:.2f}",
        ))
        arr.status.append(status)


def main(args=None):
    rclpy.init(args=args)
    node = KolesaControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Остановка узла по сигналу пользователя")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()