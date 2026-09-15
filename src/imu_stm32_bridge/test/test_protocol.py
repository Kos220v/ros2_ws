"""Тесты питоновского протокола. Запуск: pytest или python3 test_protocol.py."""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from imu_stm32_bridge import protocol as P


def test_crc_vector():
    assert P.crc16_ccitt(b'123456789') == 0x29B1
    assert P.crc16_ccitt(b'') == 0xFFFF


def test_frame_roundtrip():
    payload = bytes(range(80))
    frame = P.encode_frame(P.MSG_ORIENTATION, payload)
    assert frame[:2] == b'\xaa\x55'
    assert frame[2] == 81
    assert frame[3] == P.MSG_ORIENTATION
    assert len(frame) == 86
    dec = P.Decoder()
    msgs = dec.feed(frame)
    assert len(msgs) == 1
    assert msgs[0] == (P.MSG_ORIENTATION, payload)


def test_decoder_resync_and_bad_crc():
    good = P.encode_frame(P.MSG_ACK, b'\x80\x00\x00\x00')
    bad = bytearray(good)
    bad[-1] ^= 0xFF
    stream = b'\x00\xff\xaa\x00\xaa\x55\xc8' + bytes(bad) + good
    dec = P.Decoder()
    msgs = dec.feed(stream)
    assert len(msgs) == 1
    assert msgs[0][0] == P.MSG_ACK
    # Доставка по частям
    dec = P.Decoder()
    assert dec.feed(good[:3]) == []
    msgs = dec.feed(good[3:])
    assert len(msgs) == 1


def test_orientation_struct():
    m = P.Orientation(
        ts_ms=123456, qw=0.7071, qx=-0.5, qy=0.5, qz=0.1,
        roll_deg=-179.5, pitch_deg=45.25, yaw_deg=90.0, azimuth_deg=270.5,
        wx=0.01, wy=-0.02, wz=1.5, ax=-9.8, ay=0.1, az=0.2,
        mx=20.5, my=-30.25, mz=48.0, temp_c=36.5,
        status=0x1F, calib_state=1, rate_hz=50)
    payload = m.to_payload()
    assert len(payload) == 80
    m2 = P.Orientation.from_payload(payload)
    for field in ('qw', 'qx', 'qy', 'qz', 'roll_deg', 'pitch_deg', 'yaw_deg',
                  'azimuth_deg', 'wx', 'wy', 'wz', 'ax', 'ay', 'az',
                  'mx', 'my', 'mz', 'temp_c'):
        assert math.isclose(getattr(m, field), getattr(m2, field),
                            rel_tol=1e-6, abs_tol=1e-6), field
    assert (m2.ts_ms, m2.status, m2.calib_state, m2.rate_hz) == \
        (m.ts_ms, m.status, m.calib_state, m.rate_hz)


def test_ack_info_calib_structs():
    ack = P.Ack.from_payload(struct.pack('<BBH', 0x81, 0, 100))
    assert (ack.cmd_id, ack.result, ack.info) == (0x81, 0, 100)
    info = P.Info.from_payload(
        P.INFO_STRUCT.pack(0, 1, 0, 0, 999, b'STM32F303-IMU\x00\x00\x00\x00',
                           1, 1, 1, 1, 11.5, 50, 0, 0, 0))
    assert (info.fw_minor, info.board, info.rate_hz) == (1, 'STM32F303-IMU', 50)
    cal = P.Calib.from_payload(P.CALIB_STRUCT.pack(
        1, 42, 0, 0, 1, 2, 3, 1, 1, 1, 0.01, -0.02, 0.0,
        0.05, -0.1, 0.35, 0.98, 1.01, 0.925))
    assert (cal.state, cal.progress_pct) == (1, 42)
    assert cal.mag_hard == (1.0, 2.0, 3.0)
    for got, want in ((cal.gyro_bias, (0.01, -0.02, 0.0)),
                      (cal.accel_offset, (0.05, -0.1, 0.35)),
                      (cal.accel_scale, (0.98, 1.01, 0.925))):
        assert all(abs(g - wv) <= 1e-6 for g, wv in zip(got, want)), \
            (got, want)
    assert P.CALIB_STRUCT.size == 64


def test_command_builders():
    dec = P.Decoder()
    assert dec.feed(P.cmd_set_rate(100))[0] == (P.CMD_SET_RATE, b'\x64')
    assert dec.feed(P.cmd_ping())[0] == (P.CMD_PING, b'')
    assert dec.feed(P.cmd_mag_calib_stop(True))[0][1] == b'\x01'
    assert dec.feed(P.cmd_accel_calib_start())[0] == (P.CMD_ACCEL_CALIB_START, b'')
    assert dec.feed(P.cmd_accel_calib_stop(True))[0][1] == b'\x01'
    assert dec.feed(P.cmd_accel_calib_stop(False))[0][1] == b'\x00'
    assert dec.feed(P.cmd_zero_yaw())[0][1] == b'\x00'
    assert struct.unpack('<f', dec.feed(P.cmd_set_declination(11.5))[0][1])[0] == 11.5


if __name__ == '__main__':
    test_crc_vector()
    test_frame_roundtrip()
    test_decoder_resync_and_bad_crc()
    test_orientation_struct()
    test_ack_info_calib_structs()
    test_command_builders()
    print('python protocol tests OK')
