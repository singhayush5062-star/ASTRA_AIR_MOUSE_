#!/usr/bin/env python3
"""Unit tests for the real-drone nodes' pure logic and the hardware config generator.

No ROS needed (rospy is stubbed when absent); pymavlink, when installed, is used as the
reference MAVLink implementation for the commander's hand-rolled COMMAND_ACK / COMMAND_LONG.

    python3 -m unittest discover -s catkin_ws/src/nidar_hardware/test -v
"""
import copy
import os
import struct
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, '..', 'scripts')
CONFIG_SCRIPTS = os.path.join(HERE, '..', '..', 'nidar_config', 'scripts')
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, CONFIG_SCRIPTS)

try:
    import rospy  # noqa: F401
except ImportError:  # minimal stand-in: the modules only touch rospy inside their node classes
    rospy = types.ModuleType('rospy')

    class AnyMsg(object):
        _type, _md5sum, _full_text, _has_header = '*', '*', '', False

        def __init__(self, *args):
            self._buff = None

    rospy.AnyMsg = AnyMsg
    sys.modules['rospy'] = rospy
    for name in ('sensor_msgs', 'sensor_msgs.msg'):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules['sensor_msgs.msg'].Range = type('Range', (), {'INFRARED': 1})

import apply_hardware_config as ahc  # noqa: E402
import livox_bridge as lb  # noqa: E402
import mission_commander as mc  # noqa: E402
import rangefinder_node as rf  # noqa: E402

try:
    os.environ.setdefault('MAVLINK20', '1')
    from pymavlink.dialects.v20 import common as mavlink2
    HAVE_PYMAVLINK = True
except ImportError:
    HAVE_PYMAVLINK = False


class LivoxBridgeTest(unittest.TestCase):
    def test_stamp_round_trip(self):
        buf = bytearray(struct.pack('<III', 7, 1234, 500000000) + b'\x05\x00\x00\x00frame')
        self.assertAlmostEqual(lb.read_stamp(buf), 1234.5)
        lb.write_stamp(buf, 1791100000.25)
        self.assertAlmostEqual(lb.read_stamp(buf), 1791100000.25, places=6)
        self.assertEqual(struct.unpack_from('<I', buf, 0)[0], 7)   # seq untouched
        self.assertEqual(bytes(buf[12:]), b'\x05\x00\x00\x00frame')

    def test_unsynced_clock_is_shifted_by_min_delay(self):
        m = lb.ClockMapper(window_s=10.0)
        # LiDAR clock = time since power-on; ROS time ~1.79e9; transport delay 2..9 ms.
        for i, delay in enumerate((0.009, 0.004, 0.002, 0.006)):
            sensor_t = 100.0 + 0.005 * i
            m.observe(sensor_t, 1791100000.0 + 0.005 * i + delay)
        self.assertTrue(m.active)
        self.assertAlmostEqual(m.to_ros(100.0), 1791100000.002, places=6)

    def test_ptp_synced_clock_is_left_alone(self):
        m = lb.ClockMapper()
        m.observe(1791100000.000, 1791100000.003)
        self.assertFalse(m.active)
        self.assertEqual(m.to_ros(1791100000.5), 1791100000.5)

    def test_window_forgets_old_minimum(self):
        m = lb.ClockMapper(window_s=1.0)
        m.observe(0.0, 10.000)          # offset 10.000
        m.observe(5.0, 15.050)          # 5 s later the first sample left the window
        self.assertAlmostEqual(m.offset, 10.050)


class RangefinderTest(unittest.TestCase):
    @staticmethod
    def frame(dist_cm, strength):
        body = b'\x59\x59' + struct.pack('<HHH', dist_cm, strength, 2000)
        return body + bytes([sum(body) & 0xFF])

    def test_parse_with_garbage_and_split_frames(self):
        f1, f2 = self.frame(150, 900), self.frame(1203, 40)
        buf = bytearray(b'\x00\x59\x13' + f1 + f2[:4])
        self.assertEqual(rf.parse_frames(buf), [(1.5, 900)])
        buf.extend(f2[4:])
        self.assertEqual(rf.parse_frames(buf), [(12.03, 40)])
        self.assertEqual(len(buf), 0)

    def test_bad_checksum_resyncs(self):
        bad = bytearray(self.frame(150, 900))
        bad[8] ^= 0xFF
        buf = bad + self.frame(80, 500)
        self.assertEqual(rf.parse_frames(buf), [(0.8, 500)])

    def test_validity(self):
        self.assertTrue(rf.valid(1.5, 900, 0.1, 12.0, 100))
        self.assertFalse(rf.valid(1.5, 50, 0.1, 12.0, 100))      # weak return
        self.assertFalse(rf.valid(0.05, 900, 0.1, 12.0, 100))    # below range
        self.assertFalse(rf.valid(1.5, 65535, 0.1, 12.0, 100))   # saturation flag


class CommanderMavlinkTest(unittest.TestCase):
    @unittest.skipUnless(HAVE_PYMAVLINK, 'pymavlink not installed')
    def test_command_ack_parses_with_pymavlink(self):
        payload, crc = mc.encode_command_ack(31010, 0, 255, 190, sysid=1, compid=240, seq=7)
        frame = (bytes([0xFD, len(payload), 0, 0, 7, 1, 240]) + struct.pack('<I', 77)[:3]
                 + payload + struct.pack('<H', crc))
        mav = mavlink2.MAVLink(None)
        msg = mav.parse_char(frame)
        self.assertIsNotNone(msg)
        self.assertEqual(msg.get_type(), 'COMMAND_ACK')
        self.assertEqual((msg.command, msg.result, msg.target_system, msg.target_component),
                         (31010, 0, 255, 190))
        self.assertEqual((msg.get_srcSystem(), msg.get_srcComponent()), (1, 240))

    @unittest.skipUnless(HAVE_PYMAVLINK, 'pymavlink not installed')
    def test_command_long_from_ros_payload(self):
        mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=190)
        msg = mavlink2.MAVLink_command_long_message(1, 240, 31010, 0, 2.0, 0, 0, 0, 0, 0, 0)
        msg.pack(mav)
        payload = msg.get_payload()   # MAVLink 2: trailing zeros truncated
        self.assertLess(len(payload), 33)
        p64 = mc.payload64_from_bytes(payload)
        cmd = mc.decode_command_long(mc.payload_from_ros(p64, len(payload)))
        self.assertEqual((cmd['command'], cmd['target_system'], cmd['target_component']),
                         (31010, 1, 240))
        self.assertEqual(cmd['params'][0], 2.0)

    def test_grid_label_matches_gcs(self):
        self.assertEqual(mc.grid_label(3, 2), 'C4')
        self.assertEqual(mc.grid_label(-1, 0), 'OFF')


class CommanderReadinessTest(unittest.TestCase):
    CFG = {'min_battery_pct': 60}

    def ready_state(self, now):
        seen = {k: now for k in ('fastlio', 'lidar', 'localization', 'local_pose', 'range',
                                 'setpoint', 'edm')}
        return {'seen': seen, 'mavros_connected': True, 'armed': False, 'edm_state': 'TAKEOFF',
                'localization_healthy': True, 'fuel_alive': True, 'on_pad': True,
                'battery_pct': 95.0}

    def test_ready(self):
        self.assertEqual(mc.readiness(100.0, self.ready_state(100.0), self.CFG), (True, ''))

    def test_each_gate(self):
        cases = [
            (lambda s: s.update(mavros_connected=False), 'MAVROS'),
            (lambda s: s.update(armed=True), 'armed'),
            (lambda s: s['seen'].pop('fastlio'), 'FAST-LIO'),
            (lambda s: s['seen'].update(range=90.0), 'rangefinder'),
            (lambda s: s.update(edm_state='EXPLORATION'), 'EXPLORATION'),
            (lambda s: s.update(fuel_alive=False), 'FUEL'),
            (lambda s: s.update(on_pad=False), 'launch pad'),
            (lambda s: s.update(battery_pct=40.0), 'battery'),
        ]
        for mutate, needle in cases:
            s = copy.deepcopy(self.ready_state(100.0))
            mutate(s)
            ok, why = mc.readiness(100.0, s, self.CFG)
            self.assertFalse(ok, needle)
            self.assertIn(needle, why)


class HardwareConfigTest(unittest.TestCase):
    def setUp(self):
        self.hw = ahc.load(ahc.HW_CFG, 'hardware')
        self.mission = ahc.load(ahc.MISSION_CFG, 'nidar')

    def test_repo_config_is_valid_and_generated_files_current(self):
        self.assertEqual(ahc.main(['--check']), 0)

    def test_livox_imu_body_and_extrinsic(self):
        d = ahc.derive(self.hw, self.mission)
        if d['imu_source'] == 'livox':
            self.assertEqual(d['ext_T'], [-0.011, -0.02329, 0.04412])
            lm = d['lidar_mount']
            self.assertAlmostEqual(d['imu_in_base'][2], lm['z'] - 0.04412, places=5)

    def test_rotated_lidar_with_its_own_imu_is_refused(self):
        hw = copy.deepcopy(self.hw)
        hw['lidar']['imu_source'] = 'livox'
        hw['lidar']['mount']['roll'] = 180.0
        with self.assertRaises(ahc.ConfigError):
            ahc.derive(hw, self.mission)

    def test_fcu_imu_takes_mount_as_extrinsic(self):
        hw = copy.deepcopy(self.hw)
        hw['lidar'].update(imu_source='fcu', time_sync='ptp')
        hw['lidar']['mount'] = {'x': 0.05, 'y': 0.0, 'z': 0.1, 'roll': 180.0, 'pitch': 0, 'yaw': 0}
        d = ahc.derive(hw, self.mission)
        self.assertEqual(d['ext_T'], [0.05, 0.0, 0.1])
        self.assertEqual(d['ext_R'][4], -1.0)   # inverted: y and z flip
        self.assertEqual(d['imu_topic'], '/mavros/imu/data')

    def test_rangefinder_must_be_below(self):
        hw = copy.deepcopy(self.hw)
        hw['rangefinder']['mount']['z'] = 0.05
        with self.assertRaises(ahc.ConfigError):
            ahc.derive(hw, self.mission)

    def test_px4_params_signs(self):
        hw = copy.deepcopy(self.hw)
        hw['rangefinder']['mount'] = {'x': 0.02, 'y': 0.03, 'z': -0.07}
        d = ahc.derive(hw, self.mission)
        p = {n: v for n, v, _t, _w in ahc.px4_params(hw, d)}
        self.assertAlmostEqual(p['EKF2_RNG_POS_Z'], 0.07)     # FRD: down is +
        self.assertAlmostEqual(p['EKF2_RNG_POS_Y'], -0.03)    # FRD: right is +, mount y is left
        self.assertEqual(p['EKF2_EV_CTRL'], 11)
        self.assertEqual(p['EKF2_EV_POS_Z'], 0.0)


if __name__ == '__main__':
    unittest.main()
