"""Питоновское зеркало бинарного протокола IMU (см. Core/Inc/imu_protocol.h).

Кадр: AA 55 LEN ID PAYLOAD CRC16-CCITT-LE(LEN, ID, PAYLOAD).
"""
import struct
from dataclasses import dataclass

SYNC0 = 0xAA
SYNC1 = 0x55
MAX_PAYLOAD = 96

MSG_ORIENTATION = 0x01
MSG_INFO = 0x02
MSG_ACK = 0x03
MSG_CALIB = 0x04

CMD_PING = 0x80
CMD_SET_RATE = 0x81
CMD_MAG_CALIB_START = 0x82
CMD_MAG_CALIB_STOP = 0x83
CMD_GYRO_CALIB = 0x84
CMD_ZERO_YAW = 0x85
CMD_SET_DECLINATION = 0x86
CMD_SAVE_FLASH = 0x87
CMD_GET_INFO = 0x88
CMD_GET_CALIB = 0x89
CMD_ACCEL_CALIB_START = 0x8A
CMD_ACCEL_CALIB_STOP = 0x8B

ACK_OK = 0
ACK_ERR_ARG = 1
ACK_ERR_STATE = 2
ACK_ERR_HW = 3
ACK_ERR_UNKNOWN = 4

STATUS_MPU_OK = 1 << 0
STATUS_MAG_OK = 1 << 1
STATUS_MAG_CAL = 1 << 2
STATUS_GYRO_CAL = 1 << 3
STATUS_FUSED_9X = 1 << 4
STATUS_ACCEL_CAL = 1 << 5

CALIB_IDLE = 0
CALIB_MAG_RUN = 1
CALIB_GYRO_RUN = 2
CALIB_ACCEL_RUN = 3

ORIENTATION_STRUCT = struct.Struct('<I18f4B')
INFO_STRUCT = struct.Struct('<BBBBI16sBBBBfBBBB')
ACK_STRUCT = struct.Struct('<BBH')
CALIB_STRUCT = struct.Struct('<BBBB15f')


def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT (poly 0x1021, init 0xFFFF). b'123456789' -> 0x29B1."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_id: int, payload: bytes = b'') -> bytes:
    """Упаковать кадр."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload too long')
    body = bytes([len(payload) + 1, msg_id]) + payload
    crc = crc16_ccitt(body)
    return bytes([SYNC0, SYNC1]) + body + struct.pack('<H', crc)


class Decoder:
    """Потоковый декодер. feed(data) -> список (msg_id, payload)."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes):
        out = []
        self._buf += data
        while True:
            start = self._buf.find(b'\xaa\x55')
            if start < 0:
                # Хвост из одного 0xAA может быть началом кадра
                if self._buf and self._buf[-1] == SYNC0:
                    self._buf = bytearray(b'\xaa')
                else:
                    self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]
            if len(self._buf) < 4:
                break
            length = self._buf[2]
            if length < 1 or length > MAX_PAYLOAD + 1:
                del self._buf[:2]
                continue
            need = 2 + 1 + length + 2
            if len(self._buf) < need:
                break
            body = bytes(self._buf[2:3 + length])
            crc_rx = struct.unpack_from('<H', self._buf, 3 + length)[0]
            if crc16_ccitt(body) != crc_rx:
                del self._buf[:2]
                continue
            out.append((body[1], body[2:]))
            del self._buf[:need]
        return out


@dataclass
class Orientation:
    ts_ms: int
    qw: float
    qx: float
    qy: float
    qz: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    azimuth_deg: float
    wx: float
    wy: float
    wz: float
    ax: float
    ay: float
    az: float
    mx: float
    my: float
    mz: float
    temp_c: float
    status: int
    calib_state: int
    rate_hz: int

    @classmethod
    def from_payload(cls, payload: bytes) -> 'Orientation':
        f = ORIENTATION_STRUCT.unpack(payload)
        return cls(ts_ms=f[0], qw=f[1], qx=f[2], qy=f[3], qz=f[4],
                   roll_deg=f[5], pitch_deg=f[6], yaw_deg=f[7], azimuth_deg=f[8],
                   wx=f[9], wy=f[10], wz=f[11],
                   ax=f[12], ay=f[13], az=f[14],
                   mx=f[15], my=f[16], mz=f[17], temp_c=f[18],
                   status=f[19], calib_state=f[20], rate_hz=f[21])

    def to_payload(self) -> bytes:
        return ORIENTATION_STRUCT.pack(
            self.ts_ms, self.qw, self.qx, self.qy, self.qz,
            self.roll_deg, self.pitch_deg, self.yaw_deg, self.azimuth_deg,
            self.wx, self.wy, self.wz, self.ax, self.ay, self.az,
            self.mx, self.my, self.mz, self.temp_c,
            self.status, self.calib_state, self.rate_hz, 0)


@dataclass
class Info:
    fw_major: int
    fw_minor: int
    fw_patch: int
    uptime_ms: int
    board: str
    mpu_ok: int
    mag_ok: int
    mag_cal: int
    gyro_cal: int
    declination_deg: float
    rate_hz: int

    @classmethod
    def from_payload(cls, payload: bytes) -> 'Info':
        f = INFO_STRUCT.unpack(payload)
        return cls(fw_major=f[0], fw_minor=f[1], fw_patch=f[2], uptime_ms=f[4],
                   board=bytes(f[5]).split(b'\x00')[0].decode('ascii', 'replace'),
                   mpu_ok=f[6], mag_ok=f[7], mag_cal=f[8], gyro_cal=f[9],
                   declination_deg=f[10], rate_hz=f[11])

    def to_payload(self) -> bytes:
        board = self.board.encode('ascii', 'replace')[:16].ljust(16, b'\x00')
        return INFO_STRUCT.pack(self.fw_major, self.fw_minor, self.fw_patch, 0,
                                self.uptime_ms, board,
                                self.mpu_ok, self.mag_ok, self.mag_cal,
                                self.gyro_cal, self.declination_deg,
                                self.rate_hz, 0, 0, 0)


@dataclass
class Ack:
    cmd_id: int
    result: int
    info: int

    @classmethod
    def from_payload(cls, payload: bytes) -> 'Ack':
        cmd_id, result, info = ACK_STRUCT.unpack(payload)
        return cls(cmd_id, result, info)

    def to_payload(self) -> bytes:
        return ACK_STRUCT.pack(self.cmd_id, self.result, self.info)


@dataclass
class Calib:
    state: int
    progress_pct: int
    mag_hard: tuple
    mag_scale: tuple
    gyro_bias: tuple
    accel_offset: tuple
    accel_scale: tuple

    @classmethod
    def from_payload(cls, payload: bytes) -> 'Calib':
        f = CALIB_STRUCT.unpack(payload)
        return cls(state=f[0], progress_pct=f[1],
                   mag_hard=(f[4], f[5], f[6]),
                   mag_scale=(f[7], f[8], f[9]),
                   gyro_bias=(f[10], f[11], f[12]),
                   accel_offset=(f[13], f[14], f[15]),
                   accel_scale=(f[16], f[17], f[18]))


# --- Построители команд ---
def cmd_ping() -> bytes:
    return encode_frame(CMD_PING)


def cmd_set_rate(rate_hz: int) -> bytes:
    return encode_frame(CMD_SET_RATE, struct.pack('<B', rate_hz))


def cmd_mag_calib_start() -> bytes:
    return encode_frame(CMD_MAG_CALIB_START)


def cmd_mag_calib_stop(save: bool) -> bytes:
    return encode_frame(CMD_MAG_CALIB_STOP, struct.pack('<B', 1 if save else 0))


def cmd_gyro_calib() -> bytes:
    return encode_frame(CMD_GYRO_CALIB)


def cmd_accel_calib_start() -> bytes:
    return encode_frame(CMD_ACCEL_CALIB_START)


def cmd_accel_calib_stop(save: bool) -> bytes:
    return encode_frame(CMD_ACCEL_CALIB_STOP, struct.pack('<B', 1 if save else 0))


def cmd_zero_yaw(clear: bool = False) -> bytes:
    return encode_frame(CMD_ZERO_YAW, struct.pack('<B', 1 if clear else 0))


def cmd_set_declination(deg: float) -> bytes:
    return encode_frame(CMD_SET_DECLINATION, struct.pack('<f', deg))


def cmd_save_flash() -> bytes:
    return encode_frame(CMD_SAVE_FLASH)


def cmd_get_info() -> bytes:
    return encode_frame(CMD_GET_INFO)


def cmd_get_calib() -> bytes:
    return encode_frame(CMD_GET_CALIB)
