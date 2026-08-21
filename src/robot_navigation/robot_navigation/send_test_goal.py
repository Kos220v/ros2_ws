#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
send_test_goal — пробная поездка на заданное расстояние.

ЗАЧЕМ
-----
Перед тем как отдавать роботу маршрут из GPS-точек, нужно убедиться, что
работает сама связка «Nav2 -> мультиплексор -> моторы»: робот вообще едет,
едет ТУДА, КУДА НАДО, и останавливается там, где просили.

Отправить цель напрямую через `ros2 action send_goal` можно, но координаты
придётся считать вручную во фрейме map, а ошибка в знаке отправит робота
в противоположную сторону. Эта утилита берёт текущее положение робота из
TF и сама считает точку в нужном направлении.

ИСПОЛЬЗОВАНИЕ
-------------
Проехать 3 метра вперёд (значение по умолчанию):

    ros2 run robot_navigation send_test_goal

Проехать 5 метров вперёд:

    ros2 run robot_navigation send_test_goal --ros-args -p distance:=5.0

Проехать 3 метра вправо (в сторону от текущего курса на -90°):

    ros2 run robot_navigation send_test_goal --ros-args -p bearing_deg:=-90.0

Только показать цель, никуда не ехать:

    ros2 run robot_navigation send_test_goal --ros-args -p dry_run:=true

ПОРЯДОК ПЕРВОГО ЗАПУСКА
-----------------------
1. Поставьте робота на подставку, чтобы гусеницы не касались земли.
   Убедитесь, что они крутятся в нужную сторону.
2. Повторите на земле, на открытой площадке, с пультом в руках.

Утилита ВСЕГДА спрашивает подтверждение перед отправкой цели: робот
физически поедет, и случайный запуск не должен этого вызвать.

ЭКСТРЕННАЯ ОСТАНОВКА
--------------------
  * двиньте стик на пульте — у ручного управления приоритет выше;
  * либо Ctrl+C в этом окне — цель будет отменена.
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import tf2_ros


GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BOLD = '\033[1m'
RESET = '\033[0m'


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TestGoalSender(Node):

    def __init__(self):
        super().__init__('send_test_goal')

        self.declare_parameter('distance', 3.0)
        # Направление относительно ТЕКУЩЕГО курса робота, градусы.
        # 0 — прямо вперёд, +90 — влево, -90 — вправо, 180 — назад.
        self.declare_parameter('bearing_deg', 0.0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('tf_timeout', 10.0)

        self._distance = float(self.get_parameter('distance').value)
        self._bearing = math.radians(
            float(self.get_parameter('bearing_deg').value))
        self._dry_run = bool(self.get_parameter('dry_run').value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._last_report = None

        # Даём TF время наполниться, потом работаем
        self.create_timer(1.0, self._start_once)
        self._started = False

    # ------------------------------------------------------------------ старт
    def _start_once(self):
        if self._started:
            return
        self._started = True

        pose = self._current_pose()
        if pose is None:
            rclpy.shutdown()
            return

        x, y, yaw = pose
        heading = yaw + self._bearing
        gx = x + self._distance * math.cos(heading)
        gy = y + self._distance * math.sin(heading)

        print()
        print(f'{BOLD}ПРОБНАЯ ПОЕЗДКА{RESET}')
        print('=' * 66)
        print(f'Робот сейчас:  x={x:+.2f}  y={y:+.2f}  '
              f'курс={math.degrees(yaw):+.0f}° (азимут '
              f'{(90.0 - math.degrees(yaw)) % 360.0:.0f}°)')
        print(f'Цель:          x={gx:+.2f}  y={gy:+.2f}')
        print(f'Расстояние:    {self._distance:.1f} м, '
              f'направление {math.degrees(self._bearing):+.0f}° '
              f'от текущего курса')
        print('=' * 66)

        if self._dry_run:
            print(f'{YELLOW}Режим просмотра (dry_run) — цель не отправлена.'
                  f'{RESET}')
            print()
            rclpy.shutdown()
            return

        print()
        print(f'{YELLOW}РОБОТ СЕЙЧАС ПОЕДЕТ.{RESET} Держите пульт наготове: '
              f'движение стика')
        print('немедленно перехватит управление.')
        print()

        try:
            answer = input('Отправить цель? [y/N] ')
        except (EOFError, KeyboardInterrupt):
            answer = ''

        if answer.strip().lower() not in ('y', 'yes', 'д', 'да'):
            print('Отменено.')
            rclpy.shutdown()
            return

        self._send(gx, gy, heading)

    def _current_pose(self):
        """Положение робота во фрейме map по данным TF."""
        timeout = float(self.get_parameter('tf_timeout').value)
        deadline = self.get_clock().now().nanoseconds + timeout * 1e9

        while self.get_clock().now().nanoseconds < deadline:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time())
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.2)
                continue

            return (tf.transform.translation.x,
                    tf.transform.translation.y,
                    quaternion_to_yaw(tf.transform.rotation))

        print()
        print(f'{RED}Не удалось получить положение робота (TF map -> '
              f'base_link).{RESET}')
        print('Значит, локализация не работает. Проверьте:')
        print('  ros2 run robot_navigation nav_preflight_check')
        print()
        return None

    # ------------------------------------------------------------------ цель
    def _send(self, gx, gy, heading):
        print('Ожидание Nav2 (экшен navigate_to_pose)...')
        if not self._client.wait_for_server(timeout_sec=30.0):
            print(f'{RED}Nav2 не отвечает.{RESET} Узлы не поднялись или '
                  f'lifecycle-менеджер их не активировал:')
            print('  ros2 lifecycle get /bt_navigator')
            rclpy.shutdown()
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.pose.orientation.w = math.cos(heading / 2.0)

        print('Цель отправлена. Ctrl+C — отмена.')
        print()

        future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        handle = future.result()
        if not handle.accepted:
            print(f'{RED}Nav2 отклонил цель.{RESET}')
            rclpy.shutdown()
            return

        self._goal_handle = handle
        print(f'{GREEN}Цель принята, робот поехал.{RESET}')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg):
        remaining = msg.feedback.distance_remaining
        # Печатаем не чаще, чем раз в полметра, чтобы не залить экран
        if self._last_report is None or abs(remaining - self._last_report) > 0.5:
            self._last_report = remaining
            sys.stdout.write(f'\r  осталось {remaining:5.2f} м   ')
            sys.stdout.flush()

    def _on_result(self, future):
        print()
        status = future.result().status
        # 4 = SUCCEEDED в action_msgs/GoalStatus
        if status == 4:
            print(f'{GREEN}Цель достигнута.{RESET}')
            print()
            print('Проверьте глазами:')
            print('  * робот поехал В ТУ сторону, куда вы ожидали?')
            print('  * остановился примерно там, где просили?')
            print()
            print('Если поехал не туда — почти всегда виноват курс.')
            print('Вернитесь к heading_check и перепроверьте угол монтажа.')
        else:
            print(f'{RED}Цель не достигнута (код состояния {status}).{RESET}')
            print('Смотрите сообщения controller_server и planner_server '
                  'в логе запуска.')
        rclpy.shutdown()

    def cancel(self):
        if self._goal_handle is not None:
            print('\nОтмена цели...')
            self._goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = TestGoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel()
        # Даём отмене уйти на сервер
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
