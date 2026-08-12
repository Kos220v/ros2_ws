import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus   # ИЗМЕНЕНО: добавлен NavSatStatus
import serial
import pynmea2
import math


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_interface_node')

        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)

        self.declare_parameter('port', '/dev/gps_usb')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'GPS Node started on {port} at {baud} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial_conn = None

        self.timer = self.create_timer(0.1, self.timer_callback)

    def safe_float(self, value, default=float('nan')):
        """Безопасное преобразование в float"""
        if value is None:
            return default
        try:
            return float(value)
        except (ValueError, TypeError):
            return default

    def timer_callback(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            return

        try:
            line = self.serial_conn.readline().decode('ascii', errors='replace').strip()

            if not line.startswith('$') or (not line.startswith('$GPGGA') and not line.startswith('$GNGGA')):
                return

            msg = pynmea2.parse(line)

            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'gps_link'

            latitude = self.safe_float(getattr(msg, 'latitude', None))
            longitude = self.safe_float(getattr(msg, 'longitude', None))
            altitude = self.safe_float(getattr(msg, 'altitude', None))

            # ИЗМЕНЕНО: gps_qual приходит строкой -> приводим к int
            gps_qual = int(getattr(msg, 'gps_qual', 0) or 0)
            has_fix = (gps_qual > 0)

            if has_fix and (abs(latitude) < 0.0001 and abs(longitude) < 0.0001):
                has_fix = False

            if has_fix:
                # ИЗМЕНЕНО: STATUS_* и SERVICE_* берём из NavSatStatus
                gps_msg.status.status = NavSatStatus.STATUS_FIX
                gps_msg.status.service = NavSatStatus.SERVICE_GPS
                gps_msg.position_covariance = [9.0, 0.0, 0.0,
                                               0.0, 9.0, 0.0,
                                               0.0, 0.0, 9.0]
                # COVARIANCE_TYPE_* остаётся в NavSatFix — это правильно
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                self.get_logger().info(
                    f'GPS FIX: lat={latitude:.6f}, lon={longitude:.6f}, alt={altitude:.2f}')
            else:
                # ИЗМЕНЕНО: STATUS_NO_FIX из NavSatStatus
                gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
                gps_msg.status.service = 0
                gps_msg.position_covariance = [0.0] * 9
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                self.get_logger().debug(
                    f'No FIX: lat={latitude}, lon={longitude}, gps_qual={gps_qual}')

            gps_msg.latitude = latitude
            gps_msg.longitude = longitude
            gps_msg.altitude = altitude

            self.publisher_.publish(gps_msg)

        except pynmea2.ParseError:
            pass
        except UnicodeDecodeError:
            pass
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
        except Exception as e:
            self.get_logger().warn(f'Error reading GPS: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()