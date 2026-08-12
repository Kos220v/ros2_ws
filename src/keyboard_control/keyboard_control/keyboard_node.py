#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Keyboard Control Node Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  w/s - forward/backward')
        self.get_logger().info('  a/d - turn left/right')
        self.get_logger().info('  q/e - increase/decrease speed')
        self.get_logger().info('  space - stop')
        self.get_logger().info('  Ctrl+C - exit')
        
        # Настройки скорости
        self.linear_vel = 0.8  # м/с
        self.angular_vel = 1.0  # рад/с
        self.linear_step = 0.1
        self.angular_step = 0.2
        
        self.settings = termios.tcgetattr(sys.stdin)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                twist = Twist()
                
                if key == 'w':
                    twist.linear.x = self.linear_vel
                    self.get_logger().info(f'Forward: {self.linear_vel} m/s')
                elif key == 's':
                    twist.linear.x = -self.linear_vel
                    self.get_logger().info(f'Backward: {self.linear_vel} m/s')
                elif key == 'a':
                    twist.angular.z = self.angular_vel
                    self.get_logger().info(f'Turn Left: {self.angular_vel} rad/s')
                elif key == 'd':
                    twist.angular.z = -self.angular_vel
                    self.get_logger().info(f'Turn Right: {self.angular_vel} rad/s')
                elif key == 'q':
                    self.linear_vel += self.linear_step
                    self.angular_vel += self.angular_step
                    self.get_logger().info(f'Speed increased - Linear: {self.linear_vel}, Angular: {self.angular_vel}')
                    continue
                elif key == 'e':
                    self.linear_vel = max(0.1, self.linear_vel - self.linear_step)
                    self.angular_vel = max(0.2, self.angular_vel - self.angular_step)
                    self.get_logger().info(f'Speed decreased - Linear: {self.linear_vel}, Angular: {self.angular_vel}')
                    continue
                elif key == ' ':
                    self.get_logger().info('Stop')
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    if key and key != '\n':
                        self.get_logger().info(f'Unknown command: {key}')
                    continue
                
                self.publisher.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Останавливаем робота при выходе
            twist = Twist()
            self.publisher.publish(twist)
            self.get_logger().info('Node shutdown, robot stopped')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()