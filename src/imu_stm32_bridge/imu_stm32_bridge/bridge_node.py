"""ROS 2 узел моста STM32 IMU: UART -> sensor_msgs/Imu + MagneticField + азимут.

Топики:
  ~/data    (sensor_msgs/Imu)           - кватернион (ENU), гироскоп, акселерометр
  ~/mag     (sensor_msgs/MagneticField) - магнитное поле, Тесла
  ~/azimuth (std_msgs/Float32)          - азимут tilt-comp, градусы 0..360

Сервисы (std_srvs/Trigger):
  ~/mag_calib_start, ~/mag_calib_stop_save, ~/mag_calib_cancel,
  ~/accel_calib_start, ~/accel_calib_stop_save, ~/accel_calib_cancel,
  ~/gyro_calib, ~/zero_yaw, ~/clear_yaw, ~/save_flash

Параметры rate (10/25/50/100) и declination (град) применяются на лету.
"""
import queue
import struct
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from . import protocol as P


def _cov3(stddev: float):
    """Ковариация 3x3 из СКО. stddev<=0 -> неизвестна ([0]=-1)."""
    c = [0.0] * 9
    if stddev is None or stddev <= 0.0:
        c[0] = -1.0
    else:
        v = stddev * stddev
        c[0] = v
        c[4] = v
        c[8] = v
    return c


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_stm32_bridge')

        self.declare_parameter('port', '/dev/imu_stm32')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_topic', 'data')
        self.declare_parameter('mag_topic', 'mag')
        self.declare_parameter('azimuth_topic', 'azimuth')
        self.declare_parameter('publish_mag', True)
        self.declare_parameter('orientation_stddev', 0.02)
        self.declare_parameter('angular_velocity_stddev', 0.01)
        self.declare_parameter('linear_acceleration_stddev', 0.05)
        self.declare_parameter('magnetic_stddev', 2e-6)
        self.declare_parameter('rate', 50)
        self.declare_parameter('declination', 11.5)

        gp = self.get_parameter
        self._frame_id = gp('frame_id').value
        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._pub_imu = self.create_publisher(Imu, gp('imu_topic').value, qos)
        self._pub_mag = self.create_publisher(MagneticField, gp('mag_topic').value, qos)
        self._pub_az = self.create_publisher(Float32, gp('azimuth_topic').value, qos)

        self._mk_srv('mag_calib_start', P.cmd_mag_calib_start, P.CMD_MAG_CALIB_START,
                     'вращайте плату ~30 с, затем mag_calib_stop_save')
        self._mk_srv('mag_calib_stop_save', lambda: P.cmd_mag_calib_stop(True),
                     P.CMD_MAG_CALIB_STOP, 'калибровка применена и сохранена')
        self._mk_srv('mag_calib_cancel', lambda: P.cmd_mag_calib_stop(False),
                     P.CMD_MAG_CALIB_STOP, 'калибровка отменена')
        self._mk_srv('accel_calib_start', P.cmd_accel_calib_start,
                     P.CMD_ACCEL_CALIB_START,
                     'положите плату поочерёдно каждой из 6 граней вверх '
                     '~5 с на грань, затем accel_calib_stop_save')
        self._mk_srv('accel_calib_stop_save',
                     lambda: P.cmd_accel_calib_stop(True),
                     P.CMD_ACCEL_CALIB_STOP, 'калибровка применена и сохранена')
        self._mk_srv('accel_calib_cancel', lambda: P.cmd_accel_calib_stop(False),
                     P.CMD_ACCEL_CALIB_STOP, 'калибровка отменена')
        self._mk_srv('gyro_calib', P.cmd_gyro_calib, P.CMD_GYRO_CALIB,
                     'не трогайте плату ~2 с')
        self._mk_srv('zero_yaw', lambda: P.cmd_zero_yaw(False), P.CMD_ZERO_YAW,
                     'курс обнулён')
        self._mk_srv('clear_yaw', lambda: P.cmd_zero_yaw(True), P.CMD_ZERO_YAW,
                     'смещение курса снято')
        self._mk_srv('save_flash', P.cmd_save_flash, P.CMD_SAVE_FLASH,
                     'калибровка сохранена во flash')

        self.add_on_set_parameters_callback(self._on_params)

        self._rx_queue: queue.Queue = queue.Queue()
        self._ack_lock = threading.Lock()
        self._acks = {}  # cmd_id -> (result, info, monotonic_time)
        self._ser = None
        self._ser_lock = threading.Lock()
        self._running = True
        self._decoder = P.Decoder()
        self._frames = 0
        self._last_status = 0
        self._last_azimuth = 0.0
        self._last_rate = 0
        self._connected_once = False

        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()
        self._timer = self.create_timer(0.005, self._pump)
        self._stat_timer = self.create_timer(1.0, self._log_stats)
        self.get_logger().info(f'порт {gp("port").value}, {gp("baud").value} бод')

    # ---------- Сервисы ----------

    def _mk_srv(self, name, builder, cmd_id, hint):
        def handler(req, resp):
            ok, msg = self._send_and_wait_ack(builder(), cmd_id, timeout=3.0)
            resp.success = ok
            resp.message = f'{msg}. {hint}' if ok else msg
            return resp
        self.create_service(Trigger, name, handler)

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'rate':
                if p.value not in (10, 25, 50, 100):
                    return SetParametersResult(successful=False,
                                               reason='rate: 10/25/50/100')
                self._write(P.cmd_set_rate(p.value))
            elif p.name == 'declination':
                if not -30.0 <= p.value <= 30.0:
                    return SetParametersResult(successful=False,
                                               reason='declination: -30..30')
                self._write(P.cmd_set_declination(float(p.value)))
        return SetParametersResult(successful=True)

    # ---------- UART ----------

    def _write(self, frame: bytes):
        with self._ser_lock:
            if self._ser is not None and self._ser.is_open:
                try:
                    self._ser.write(frame)
                except serial.SerialException as e:
                    self.get_logger().warn(f'UART write: {e}')

    def _send_and_wait_ack(self, frame: bytes, cmd_id: int, timeout: float):
        with self._ack_lock:
            self._acks.pop(cmd_id, None)
        self._write(frame)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._ack_lock:
                if cmd_id in self._acks:
                    result, info, _ = self._acks.pop(cmd_id)
                    if result == P.ACK_OK:
                        return True, f'ACK ok (info={info})'
                    return False, f'ACK error {result} (info={info})'
            time.sleep(0.02)
        return False, 'таймаут ожидания ACK'

    def _reader_loop(self):
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        while self._running:
            if self._ser is None:
                try:
                    self._ser = serial.Serial(port, baud, timeout=0.1)
                    self.get_logger().info(f'UART открыт: {port}')
                    if not self._connected_once:
                        self._connected_once = True
                        self._write(P.cmd_get_info())
                        rate = self.get_parameter('rate').value
                        if rate in (10, 25, 50, 100):
                            self._write(P.cmd_set_rate(rate))
                        decl = float(self.get_parameter('declination').value)
                        if -30.0 <= decl <= 30.0:
                            self._write(P.cmd_set_declination(decl))
                except serial.SerialException:
                    self._ser = None
                    time.sleep(2.0)
                    continue
            try:
                data = self._ser.read(256)
            except serial.SerialException:
                with self._ser_lock:
                    try:
                        self._ser.close()
                    except Exception:
                        pass
                    self._ser = None
                self.get_logger().warn('UART потерян, переподключение...')
                continue
            if data:
                for msg in self._decoder.feed(data):
                    self._rx_queue.put(msg)

    # ---------- Обработка входящих ----------

    def _pump(self):
        while True:
            try:
                msg_id, payload = self._rx_queue.get_nowait()
            except queue.Empty:
                break
            try:
                if msg_id == P.MSG_ORIENTATION:
                    self._on_orientation(P.Orientation.from_payload(payload))
                elif msg_id == P.MSG_ACK:
                    ack = P.Ack.from_payload(payload)
                    with self._ack_lock:
                        self._acks[ack.cmd_id] = (ack.result, ack.info,
                                                  time.monotonic())
                elif msg_id == P.MSG_INFO:
                    info = P.Info.from_payload(payload)
                    self.get_logger().info(
                        f'IMU {info.board} fw {info.fw_major}.{info.fw_minor}.'
                        f'{info.fw_patch} mpu={info.mpu_ok} mag={info.mag_ok} '
                        f'mag_cal={info.mag_cal} gyro_cal={info.gyro_cal} '
                        f'decl={info.declination_deg:.2f} rate={info.rate_hz}')
                elif msg_id == P.MSG_CALIB:
                    cal = P.Calib.from_payload(payload)
                    self.get_logger().info(
                        f'CALIB state={cal.state} progress={cal.progress_pct}% '
                        f'hard={tuple(round(v, 1) for v in cal.mag_hard)} '
                        f'gyro_bias={tuple(round(v, 5) for v in cal.gyro_bias)}')
            except struct.error as e:
                self.get_logger().warn(f'разбор кадра 0x{msg_id:02X}: {e}')

    def _on_orientation(self, m: P.Orientation):
        self._frames += 1
        self._last_status = m.status
        self._last_azimuth = m.azimuth_deg
        self._last_rate = m.rate_hz
        now = self.get_clock().now().to_msg()
        gp = self.get_parameter

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self._frame_id
        imu.orientation.w = m.qw
        imu.orientation.x = m.qx
        imu.orientation.y = m.qy
        imu.orientation.z = m.qz
        imu.orientation_covariance = _cov3(gp('orientation_stddev').value)
        imu.angular_velocity.x = m.wx
        imu.angular_velocity.y = m.wy
        imu.angular_velocity.z = m.wz
        imu.angular_velocity_covariance = _cov3(gp('angular_velocity_stddev').value)
        imu.linear_acceleration.x = m.ax
        imu.linear_acceleration.y = m.ay
        imu.linear_acceleration.z = m.az
        imu.linear_acceleration_covariance = _cov3(
            gp('linear_acceleration_stddev').value)
        self._pub_imu.publish(imu)

        if gp('publish_mag').value:
            mag = MagneticField()
            mag.header.stamp = now
            mag.header.frame_id = self._frame_id
            mag.magnetic_field.x = m.mx * 1e-6
            mag.magnetic_field.y = m.my * 1e-6
            mag.magnetic_field.z = m.mz * 1e-6
            mag.magnetic_field_covariance = _cov3(gp('magnetic_stddev').value)
            self._pub_mag.publish(mag)

        az = Float32()
        az.data = m.azimuth_deg
        self._pub_az.publish(az)

    def _log_stats(self):
        st = self._last_status
        mode = '9x' if st & P.STATUS_FUSED_9X else '6x'
        self.get_logger().info(
            f'{self._frames} кадров, {self._last_rate} Гц, режим {mode}, '
            f'азимут {self._last_azimuth:.1f}°, '
            f'mpu={bool(st & P.STATUS_MPU_OK)} mag={bool(st & P.STATUS_MAG_OK)} '
            f'mag_cal={bool(st & P.STATUS_MAG_CAL)} '
            f'gyro_cal={bool(st & P.STATUS_GYRO_CAL)}')
        self._frames = 0

    def destroy_node(self):
        self._running = False
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
