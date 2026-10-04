#!/usr/bin/env python3
"""Onboard mission commander: the drone-side end of the GCS's TAKEOFF / RTL / LAND buttons.

In the simulation scripts/test_takeoff.sh arms and switches to OFFBOARD itself. On the real drone
nothing may arm until an operator says so, and only once the autonomy stack is verifiably up --
that is this node. It runs after hw_bringup.sh has brought the stack up and:

  * TAKEOFF  checks the stack is ready (below), then flies the same sequence test_takeoff.sh
             flies in SITL: AUTO.TAKEOFF -> arm (retries while EKF2 aligns) -> OFFBOARD. From
             there the mission manager (EDM) owns the vehicle: climb, door, FUEL, return, land.
  * RTL      asks the mission manager to return (/mission/return_request): it stops FUEL, flies
             its breadcrumb trail back out of the door and lands on the pad. Never PX4 AUTO.RTL,
             whose straight line home crosses the arena walls.
  * LAND     PX4 AUTO.LAND where the vehicle is.

Commands arrive two ways:
  ROS      std_srvs/Trigger /nidar/cmd/{takeoff,rtl,land} -- the GCS over Wi-Fi (gcs_ros_bridge)
  MAVLink  COMMAND_LONG MAV_CMD_USER_1 (param1: 1 takeoff, 2 rtl, 3 land) from the GCS over the
           T12/SiK radio, forwarded by PX4 (MAV_0_FORWARD/MAV_1_FORWARD) and seen on MAVROS's
           /mavlink/from; answered with COMMAND_ACK on /mavlink/to.

It also publishes /nidar/onboard/status (JSON, 2 Hz: topic ages, ready + reason) for the GCS
health panel, reports mission state / survivors / readiness to a radio-only GCS as STATUSTEXT
("NIDAR ST <state>", "NIDAR SV <id> <grid> <conf> <x> <y>", "NIDAR RDY <0|1> <why>"), and asks
for RTL once if the battery falls below hardware.yaml commander.auto_rtl_battery_pct in flight.
"""
import json
import math
import string
import struct
import threading
import time

import rospy

MAV_CMD_USER_1 = 31010
MSG_COMMAND_LONG = 76
MSG_COMMAND_ACK = 77
CRC_EXTRA_COMMAND_ACK = 143
MAV_RESULT_ACCEPTED, MAV_RESULT_TEMPORARILY_REJECTED, MAV_RESULT_DENIED = 0, 1, 2
MAV_RESULT_FAILED = 4
ACTIONS = {1: 'takeoff', 2: 'rtl', 3: 'land'}
RETURNING = ('RETURN', 'DESCEND', 'LAND')


# ─── MAVLink bytes (no pymavlink needed onboard) ──────────────────────────────────────────────

def x25_crc(data, crc=0xFFFF):
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def payload_from_ros(payload64, length):
    """mavros_msgs/Mavlink payload64 (little-endian uint64 words) -> payload bytes."""
    raw = struct.pack('<%dQ' % len(payload64), *payload64) if payload64 else b''
    return raw[:length]


def payload64_from_bytes(payload):
    padded = payload + b'\0' * ((-len(payload)) % 8)
    return list(struct.unpack('<%dQ' % (len(padded) // 8), padded)) if padded else []


def decode_command_long(payload):
    """COMMAND_LONG payload (possibly MAVLink2-truncated) -> dict."""
    p = payload + b'\0' * max(0, 33 - len(payload))
    f = struct.unpack_from('<7fHBBB', p)
    return {'params': f[:7], 'command': f[7], 'target_system': f[8], 'target_component': f[9],
            'confirmation': f[10]}


def encode_command_ack(command, result, target_system, target_component, sysid, compid, seq):
    """MAVLink2 COMMAND_ACK -> (payload, checksum). Payload trailing zeros trimmed (MAVLink2)."""
    payload = struct.pack('<HBBiBB', command, result, 0, 0, target_system, target_component)
    payload = payload.rstrip(b'\0') or b'\0'
    header = struct.pack('<BBBBBB', len(payload), 0, 0, seq & 0xFF, sysid, compid) + \
        struct.pack('<I', MSG_COMMAND_ACK)[:3]
    crc = x25_crc(header + payload + bytes([CRC_EXTRA_COMMAND_ACK]))
    return payload, crc


def grid_label(grid_x, grid_y):
    """Same convention as the GCS and nidar_map2d/grid_visualizer.py (row letter from y)."""
    if grid_x < 0 or grid_y < 0:
        return 'OFF'
    return '%s%d' % (string.ascii_uppercase[min(grid_y, 25)], grid_x + 1)


def readiness(now, s, cfg):
    """(ready, reason) from the latest onboard state s. Pure, so it is unit tested."""
    def age(key):
        t = s['seen'].get(key)
        return float('inf') if t is None else now - t

    checks = [
        (s.get('mavros_connected'), 'no flight controller (MAVROS not connected)'),
        (not s.get('armed'), 'already armed'),
        (age('fastlio') < 0.5, 'FAST-LIO not tracking'),
        (age('lidar') < 1.5, 'no LiDAR data'),
        (s.get('localization_healthy', False) and age('localization') < 2.0,
         'vision relay unhealthy'),
        (age('local_pose') < 0.5, 'no PX4 local position (EKF2 not using vision)'),
        (age('range') < 1.0, 'no rangefinder (/tfmini/range)'),
        (age('setpoint') < 0.5, 'flight envelope guard not streaming setpoints'),
        (age('edm') < 2.0, 'mission manager not running'),
        (s.get('edm_state') == 'TAKEOFF', 'mission manager in %s, not TAKEOFF' % s.get('edm_state')),
        (s.get('fuel_alive', False), 'FUEL exploration planner not running'),
        (s.get('on_pad', False), 'vehicle not on the launch pad origin'),
        (s.get('battery_pct') is None or s['battery_pct'] >= cfg['min_battery_pct'],
         'battery %s%% below %d%%' % (s.get('battery_pct'), cfg['min_battery_pct'])),
    ]
    for ok, why in checks:
        if not ok:
            return False, why
    return True, ''


class MissionCommander(object):
    def __init__(self):
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import Mavlink, PositionTarget, State, StatusText
        from mavros_msgs.srv import CommandBool, SetMode
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import BatteryState, Range
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger, TriggerResponse

        self.Mavlink, self.StatusText, self.String = Mavlink, StatusText, String
        self.TriggerResponse = TriggerResponse
        self.SetMode, self.CommandBool = SetMode, CommandBool
        c = lambda k, d: rospy.get_param('/hardware/commander/' + k, d)  # noqa: E731
        self.cfg = {'min_battery_pct': int(c('min_battery_pct', 60)),
                    'auto_rtl_battery_pct': int(c('auto_rtl_battery_pct', 30)),
                    'offboard_delay_s': float(c('offboard_delay_s', 1.0)),
                    'arm_attempts': int(c('arm_attempts', 10))}
        self.compid = int(rospy.get_param('/mavros/component_id', rospy.get_param('~mavros_compid', 240)))
        self.sysid = int(rospy.get_param('/mavros/system_id', 1))
        self.lock = threading.Lock()
        self.s = {'seen': {}, 'armed': False, 'mavros_connected': False, 'mode': '',
                  'edm_state': None, 'localization_healthy': False, 'battery_pct': None,
                  'fuel_alive': False, 'on_pad': False, 'local_pose': None}
        self.busy = False            # a takeoff sequence is running
        self.auto_rtl_done = False
        self.reported_state = None
        self.reported_ready = None
        self.ready_sent_at = 0.0
        self.survivor_cells = {}
        self.seq = 0

        def seen(key):
            def cb(_msg):
                self.s['seen'][key] = time.time()
            return cb

        S = rospy.Subscriber
        S('/mavros/state', State, self.state_cb, queue_size=5)
        S('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=1)
        S('/mavros/battery', BatteryState, self.battery_cb, queue_size=1)
        S('/mavros/setpoint_raw/local', PositionTarget, seen('setpoint'), queue_size=1)
        S('/Fast_LIO/odometry', Odometry, seen('fastlio'), queue_size=1)
        S('/localization/healthy', Bool, self.health_cb, queue_size=1)
        S('/tfmini/range', Range, seen('range'), queue_size=1)
        S('/edm/mission_state', String, self.edm_cb, queue_size=5)
        S('/nidar/livox/status', String, self.livox_cb, queue_size=1)
        S('/camera/camera_info', rospy.AnyMsg, seen('camera'), queue_size=1)
        S('/survivor_detector/detections', String, seen('yolo'), queue_size=1)
        S('/planning/pos_cmd', rospy.AnyMsg, seen('planner'), queue_size=1)
        S('/flight_envelope_guard/status', rospy.AnyMsg, seen('guard'), queue_size=1)
        S(rospy.get_param('~imu_topic', '/nidar/livox/imu'), rospy.AnyMsg, seen('imu'), queue_size=1)
        try:
            from nidar_msgs.msg import SurvivorArray
            S('/survivors', SurvivorArray, self.survivors_cb, queue_size=1)
        except ImportError:
            rospy.logwarn('[commander] nidar_msgs not built: survivors not reported over MAVLink')
        S('/mavlink/from', Mavlink, self.mavlink_cb, queue_size=50)

        self.pub_status = rospy.Publisher('/nidar/onboard/status', String, queue_size=1)
        self.pub_return = rospy.Publisher('/mission/return_request', String, queue_size=1)
        self.pub_text = rospy.Publisher('/mavros/statustext/send', StatusText, queue_size=10)
        self.pub_mavlink = rospy.Publisher('/mavlink/to', Mavlink, queue_size=10)
        for name in ('takeoff', 'rtl', 'land'):
            rospy.Service('/nidar/cmd/%s' % name, Trigger,
                          lambda _req, n=name: self._service(n))
        rospy.Timer(rospy.Duration(0.5), self.tick)
        rospy.loginfo('[commander] ready: /nidar/cmd/{takeoff,rtl,land}, MAV_CMD_USER_1 on '
                      'component %d', self.compid)

    # -- inputs --------------------------------------------------------------------------------

    def state_cb(self, msg):
        self.s.update(mavros_connected=msg.connected, armed=msg.armed, mode=msg.mode)
        if not msg.armed:
            self.auto_rtl_done = False

    def pose_cb(self, msg):
        self.s['seen']['local_pose'] = time.time()
        p = msg.pose.position
        self.s['local_pose'] = (p.x, p.y, p.z)
        # camera_init is planted on the pad: the EDM requires |xy| < pad_radius, z < 0.5.
        self.s['on_pad'] = math.hypot(p.x, p.y) < 1.0 and p.z < 0.5

    def battery_cb(self, msg):
        if msg.percentage == msg.percentage and msg.percentage >= 0:   # not NaN
            self.s['battery_pct'] = round(100.0 * msg.percentage, 1)

    def health_cb(self, msg):
        self.s['seen']['localization'] = time.time()
        self.s['localization_healthy'] = bool(msg.data)

    def edm_cb(self, msg):
        self.s['seen']['edm'] = time.time()
        self.s['edm_state'] = msg.data

    def livox_cb(self, msg):
        try:
            if json.loads(msg.data).get('lidar_hz', 0) > 0:
                self.s['seen']['lidar'] = time.time()
        except ValueError:
            pass

    def survivors_cb(self, msg):
        for sv in msg.survivors:
            label = grid_label(int(sv.grid_x), int(sv.grid_y))
            if self.survivor_cells.get(sv.id) == label:
                continue
            self.survivor_cells[sv.id] = label
            self.text('NIDAR SV %d %s %.0f %.2f %.2f' % (sv.id, label, 100.0 * sv.confidence,
                                                         sv.position.x, sv.position.y))

    def mavlink_cb(self, msg):
        if msg.msgid != MSG_COMMAND_LONG:
            return
        cmd = decode_command_long(payload_from_ros(msg.payload64, msg.len))
        if cmd['command'] != MAV_CMD_USER_1 or cmd['target_system'] not in (0, self.sysid) \
                or cmd['target_component'] not in (0, self.compid):
            return
        action = ACTIONS.get(int(round(cmd['params'][0])))
        if action is None:
            self.ack(cmd, msg, MAV_RESULT_DENIED)
            return
        ok, why = self.execute(action, via='MAVLink')
        self.ack(cmd, msg, MAV_RESULT_ACCEPTED if ok else MAV_RESULT_DENIED)
        if not ok:
            self.text('NIDAR %s refused: %s' % (action.upper(), why)[:50], severity=4)

    # -- outputs -------------------------------------------------------------------------------

    def text(self, text, severity=6):
        m = self.StatusText(severity=severity, text=text[:50])
        m.header.stamp = rospy.Time.now()
        self.pub_text.publish(m)

    def ack(self, cmd, msg, result):
        self.seq = (self.seq + 1) & 0xFF
        payload, crc = encode_command_ack(MAV_CMD_USER_1, result, msg.sysid, msg.compid,
                                          self.sysid, self.compid, self.seq)
        out = self.Mavlink(framing_status=self.Mavlink.FRAMING_OK, magic=self.Mavlink.MAVLINK_V20,
                           len=len(payload), incompat_flags=0, compat_flags=0, seq=self.seq,
                           sysid=self.sysid, compid=self.compid, msgid=MSG_COMMAND_ACK,
                           checksum=crc, payload64=payload64_from_bytes(payload))
        out.header.stamp = rospy.Time.now()
        self.pub_mavlink.publish(out)

    # -- commands ------------------------------------------------------------------------------

    def _service(self, action):
        ok, why = self.execute(action, via='ROS')
        return self.TriggerResponse(success=ok, message=why)

    def execute(self, action, via):
        rospy.logwarn('[commander] %s requested via %s', action.upper(), via)
        if action == 'takeoff':
            ready, why = readiness(time.time(), self.s, self.cfg)
            if not ready:
                rospy.logwarn('[commander] TAKEOFF refused: %s', why)
                return False, why
            with self.lock:
                if self.busy:
                    return False, 'takeoff already in progress'
                self.busy = True
            threading.Thread(target=self._takeoff_sequence, daemon=True).start()
            return True, 'takeoff sequence started (AUTO.TAKEOFF -> arm -> OFFBOARD)'
        if action == 'rtl':
            if not self.s.get('armed'):
                return False, 'not flying'
            if self.s.get('edm_state') in RETURNING:
                return True, 'already returning (%s)' % self.s['edm_state']
            if self.pub_return.get_num_connections() == 0:
                return False, 'mission manager not listening on /mission/return_request'
            self.pub_return.publish(self.String(data='operator RTL via %s' % via))
            return True, 'mission return requested'
        if action == 'land':
            return self._set_mode('AUTO.LAND')
        return False, 'unknown action'

    def _set_mode(self, mode):
        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=2.0)
            ok = rospy.ServiceProxy('/mavros/set_mode', self.SetMode)(custom_mode=mode).mode_sent
            return bool(ok), '%s %s' % (mode, 'sent' if ok else 'rejected by PX4')
        except Exception as e:
            return False, '%s failed: %s' % (mode, e)

    def _takeoff_sequence(self):
        """test_takeoff.sh's tested order: AUTO.TAKEOFF, arm, OFFBOARD."""
        try:
            self.text('NIDAR TAKEOFF: arming')
            ok, why = self._set_mode('AUTO.TAKEOFF')
            if not ok:
                self._takeoff_failed(why)
                return
            arm = rospy.ServiceProxy('/mavros/cmd/arming', self.CommandBool)
            for attempt in range(self.cfg['arm_attempts']):
                try:
                    arm(value=True)
                except Exception as e:
                    rospy.logwarn('[commander] arming call failed: %s', e)
                rospy.sleep(1.0)
                if self.s.get('armed'):
                    break
                rospy.logwarn('[commander] arming rejected (attempt %d, EKF2 aligning?)', attempt + 1)
            if not self.s.get('armed'):
                self._takeoff_failed('PX4 refused to arm (QGC/PX4 messages say why)')
                return
            rospy.sleep(self.cfg['offboard_delay_s'])
            ok, why = self._set_mode('OFFBOARD')
            if not ok:
                # Armed in AUTO.TAKEOFF: PX4 climbs to MIS_TAKEOFF_ALT and holds. Land instead of
                # hovering with no mission in control.
                self._set_mode('AUTO.LAND')
                self._takeoff_failed('OFFBOARD rejected (%s) -- landing' % why)
                return
            rospy.logwarn('[commander] TAKEOFF: armed and in OFFBOARD; the mission manager has it')
            self.text('NIDAR TAKEOFF: OFFBOARD, mission running')
        finally:
            self.busy = False

    def _takeoff_failed(self, why):
        rospy.logerr('[commander] TAKEOFF failed: %s', why)
        self.text(('NIDAR TAKEOFF FAILED: %s' % why)[:50], severity=3)

    # -- periodic ----------------------------------------------------------------------------

    def tick(self, _evt):
        now = time.time()
        self.s['fuel_alive'] = self._node_alive('/exploration_node')
        ready, why = readiness(now, self.s, self.cfg)
        ages = {k: round(now - t, 2) for k, t in self.s['seen'].items()}
        self.pub_status.publish(self.String(data=json.dumps({
            'ages': ages, 'ready': ready, 'reason': why, 'edm_state': self.s.get('edm_state'),
            'armed': self.s.get('armed'), 'mode': self.s.get('mode'),
            'battery_pct': self.s.get('battery_pct'), 'busy': self.busy})))

        edm = self.s.get('edm_state')
        if edm and edm != self.reported_state:
            self.reported_state = edm
            self.text('NIDAR ST %s' % edm)
        if not self.s.get('armed') and ((ready, why) != self.reported_ready or now - self.ready_sent_at > 30):
            self.reported_ready, self.ready_sent_at = (ready, why), now
            self.text(('NIDAR RDY 1' if ready else 'NIDAR RDY 0 %s' % why)[:50])

        bat = self.s.get('battery_pct')
        if (self.s.get('armed') and not self.auto_rtl_done and bat is not None
                and bat < self.cfg['auto_rtl_battery_pct'] and edm not in RETURNING):
            self.auto_rtl_done = True
            rospy.logerr('[commander] battery %.0f%% < %d%%: mission RTL', bat,
                         self.cfg['auto_rtl_battery_pct'])
            self.text('NIDAR LOW BATTERY %.0f%%: RTL' % bat, severity=2)
            self.execute('rtl', via='battery')

    def _node_alive(self, name):
        try:
            import rosgraph
            state = rosgraph.Master(rospy.get_name()).getSystemState()
            return any(name in nodes for _topic, nodes in state[0])
        except Exception:
            return False


if __name__ == '__main__':
    rospy.init_node('mission_commander')
    MissionCommander()
    rospy.spin()
