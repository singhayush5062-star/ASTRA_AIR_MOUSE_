#!/usr/bin/env python3
"""ROS -> GCS bridge: reads the running stack and streams it to the GCS backend as JSON lines.

The GCS backend (nidar_gcs/backend, FastAPI) does not import rospy; it runs this node as a child
process and reads one JSON object per line from its stdout. Commands go the other way, one JSON
object per line on stdin. Keeping ROS in a separate, restartable process means the backend keeps
serving the UI while the simulation is started, reset or restarted underneath it.

Lifecycle: waits for a ROS master, initialises, and exits as soon as the master goes away (a
rospy node cannot re-register with a new master). The backend restarts it, so it re-attaches to
every new run, including runs started outside the UI.

Output messages (field "type"):
  bridge      lifecycle: {"status": "waiting_master" | "connected" | "master_lost"}
  state       10 Hz snapshot: vehicle pose/velocity/attitude in the arena `world` frame, MAVROS
              state, battery, EDM mission state, coverage, FUEL goal, topic liveness, paused flag
  map         /map_2d (nav_msgs/OccupancyGrid, frame `world`), at most 1 Hz, only when changed.
              Cells: -1 unknown, 0 free, 100 wall (75 frontier = free next to unexplored space,
              only with NIDAR_GCS_MAP_FRONTIER=1), run-length encoded as a flat
              [value, count, value, count, ...] list ("rle")
  survivors   /survivors (nidar_msgs/SurvivorArray), whenever it changes
  log         filtered /rosout_agg lines (mission transitions, FCU messages, warnings, errors)
  frame       annotated camera JPEG (base64), only while the backend has enabled the camera
  ack         result of a command

Commands: {"cmd": "camera", "enable": bool} | {"cmd": "pause"} | {"cmd": "unpause"} |
          {"cmd": "land"}
"""
import base64
import json
import math
import os
import sys
import threading
import time

import rosgraph

OUT_LOCK = threading.Lock()
PARENT_PID = os.getppid()


def parent_gone():
    """The backend that started us has exited (we were re-parented): stop, never linger."""
    return os.getppid() != PARENT_PID


def emit(msg_type, **fields):
    fields['type'] = msg_type
    fields['t'] = time.time()
    line = json.dumps(fields, separators=(',', ':'))
    with OUT_LOCK:
        try:
            sys.stdout.write(line + '\n')
            sys.stdout.flush()
        except (BrokenPipeError, ValueError):
            os._exit(0)  # backend went away


def wait_for_master():
    announced = False
    while not rosgraph.is_master_online():
        if parent_gone():
            os._exit(0)
        if not announced:
            emit('bridge', status='waiting_master')
            announced = True
        time.sleep(1.0)


wait_for_master()

import cv2  # noqa: E402  (imported after the master wait so an idle bridge stays light)
import numpy as np  # noqa: E402
import rospy  # noqa: E402
import tf2_ros  # noqa: E402
import tf.transformations as tft  # noqa: E402
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
from mavros_msgs.msg import State  # noqa: E402
from mavros_msgs.srv import SetMode  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rosgraph_msgs.msg import Log  # noqa: E402
from sensor_msgs.msg import BatteryState, Image  # noqa: E402
from std_msgs.msg import Bool, Float64MultiArray, String  # noqa: E402

try:
    from nidar_msgs.msg import SurvivorArray
except ImportError:  # workspace not built; survivors simply never arrive
    SurvivorArray = None
try:
    from bspline.msg import Bspline
except ImportError:
    Bspline = None
try:
    from std_srvs.srv import Empty
    from gazebo_msgs.srv import GetPhysicsProperties
except ImportError:
    Empty = GetPhysicsProperties = None

WORLD = 'world'
LOCAL = 'camera_init'   # MAVROS local frame == FAST-LIO camera_init in this stack (phase plan A1)

# rosout lines worth showing on the operator timeline at INFO level. WARN and above always pass.
INFO_PATTERNS = ('[EDM] State Transition', '[EDM] DESCEND', '[EDM] LAND', '[EDM] RETURN',
                 'FCU: ', '[detector] Phase 4', 'exploration completed', 'Exploration completed')
# Nodes whose warnings are expected chatter rather than operator information.
QUIET_NODES = ('/gazebo', '/gazebo_gui', '/rosout', '/rviz')


# Frontier marking is off by default: the edges of the floor wedges the LiDAR sees through a
# doorway are frontier by definition, and drawn as thin amber lines they read like diagonal walls.
MARK_FRONTIER = os.environ.get('NIDAR_GCS_MAP_FRONTIER', '0') == '1'


def frontier_and_rle(data, width, height, mark_frontier=True):
    """Optionally mark frontier cells (free with an unknown 4-neighbour) as 75, and run-length
    encode. RLE keeps a 0.05 m map of the whole arena (~120k cells) to tens of kilobytes per
    update, which matters once this runs over the real radio link.
    """
    g = np.asarray(data, dtype=np.int8).reshape(height, width).copy()
    if not mark_frontier:
        return _rle(g)
    # Unknown cells hugging a wall are the wall's own shadow (rays stop just short of the hit),
    # not unexplored space: they must not turn the whole wall outline into "frontier".
    near_wall = cv2.dilate((g == 100).astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
    unknown = (g == -1) & ~near_wall
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    g[(g == 0) & near_unknown] = 75
    return _rle(g)


def _rle(g):
    flat = g.ravel()
    starts = np.flatnonzero(np.concatenate(([True], flat[1:] != flat[:-1])))
    counts = np.diff(np.concatenate((starts, [flat.size])))
    rle = np.empty(2 * len(starts), dtype=np.int64)
    rle[0::2] = flat[starts]
    rle[1::2] = counts
    return rle.tolist()


def rotate_box_180(b, w, h):
    return [w - b[2], h - b[3], w - b[0], h - b[1]]


def draw_detections(img, boxes, hud):
    """Same HUD style as the backend's FC/Jetson stream (vision_service.py), so the simulated
    feed looks like the real one: green box, corner brackets, confidence badge, header bar."""
    color = (65, 255, 0)
    for b in boxes:
        x1, y1, x2, y2 = [int(round(v)) for v in b['xyxy']]
        bw, bh = max(1, x2 - x1), max(1, y2 - y1)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 1)
        c = max(2, min(15, bw // 4, bh // 4))
        for (px, py, dx, dy) in ((x1, y1, 1, 1), (x2, y1, -1, 1), (x1, y2, 1, -1), (x2, y2, -1, -1)):
            cv2.line(img, (px, py), (px + dx * c, py), color, 3)
            cv2.line(img, (px, py), (px, py + dy * c), color, 3)
        label = 'SURVIVOR %.1f%%' % (100.0 * b['conf'])
        cv2.rectangle(img, (x1, max(0, y1 - 22)), (x1 + 140, max(22, y1)), (0, 0, 0), -1)
        cv2.rectangle(img, (x1, max(0, y1 - 22)), (x1 + 140, max(22, y1)), color, 1)
        cv2.putText(img, label, (x1 + 5, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
                    cv2.LINE_AA)
    w = img.shape[1]
    cv2.rectangle(img, (0, 0), (w, 24), (10, 10, 12), -1)
    cv2.line(img, (0, 24), (w, 24), (39, 39, 42), 1)
    cv2.putText(img, hud, (10, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 240, 255), 1, cv2.LINE_AA)


class Bridge(object):
    def __init__(self):
        rospy.init_node('nidar_gcs_bridge', anonymous=True, disable_signals=True)
        self.lock = threading.Lock()
        self.last = {}                    # topic key -> wall time of last message
        self.mav = None                   # mavros_msgs/State
        self.local_pose = None            # PoseStamped in camera_init
        self.local_vel = None             # TwistStamped in camera_init
        self.battery = None
        self.edm_state = None
        self.coverage = None
        self.explore_done = False
        self.goal_local = None            # last FUEL B-spline end point, camera_init
        self.T_world_local = None         # 4x4, static
        self.paused = None
        self.cam_enabled = False
        self.cam_period = 1.0 / float(os.environ.get('NIDAR_GCS_CAMERA_FPS', '10'))
        self.cam_flip = os.environ.get('NIDAR_GCS_CAMERA_FLIP', '1') == '1'
        self.cam_last_emit = 0.0
        self.cam_times = []
        self.det = None                   # last detector JSON dict
        self.det_times = []
        self.map_last_emit = 0.0
        self.map_pending = None

        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        def seen(key):
            return lambda _msg: self.last.__setitem__(key, time.time())

        S = rospy.Subscriber
        S('/mavros/state', State, self._mav_cb, queue_size=5)
        S('/mavros/local_position/pose', PoseStamped, self._pose_cb, queue_size=1)
        S('/mavros/local_position/velocity_local', TwistStamped, self._vel_cb, queue_size=1)
        S('/mavros/battery', BatteryState, self._bat_cb, queue_size=1)
        S('/edm/mission_state', String, self._edm_cb, queue_size=5)
        S('/sdf_map/coverage', Float64MultiArray, self._cov_cb, queue_size=1)
        S('/exploration_completed', Bool, self._done_cb, queue_size=1)
        S('/map_2d', OccupancyGrid, self._map_cb, queue_size=1)
        S('/rosout_agg', Log, self._log_cb, queue_size=200)
        S('/survivor_detector/detections', String, self._det_cb, queue_size=1)
        S('/camera/image_raw', Image, self._img_cb, queue_size=1, buff_size=2 ** 24)
        if SurvivorArray is not None:
            S('/survivors', SurvivorArray, self._surv_cb, queue_size=1)
        if Bspline is not None:
            S('/planning/bspline', Bspline, self._bspline_cb, queue_size=1)
        # Liveness only: AnyMsg skips deserialisation, so watching the heavy topics is cheap.
        for key, topic in (('imu', '/mavros/imu/data'), ('lidar', '/velodyne_points'),
                           ('fastlio', '/Fast_LIO/odometry'), ('planner', '/planning/pos_cmd'),
                           ('clock', '/clock'), ('guard', '/flight_envelope_guard/status')):
            S(topic, rospy.AnyMsg, seen(key), queue_size=1)

        threading.Thread(target=self._stdin_loop, daemon=True).start()
        threading.Thread(target=self._master_watch, daemon=True).start()
        emit('bridge', status='connected', master=rosgraph.get_master_uri())

    # -- callbacks ------------------------------------------------------------------------------

    def _mav_cb(self, msg):
        self.last['mavros'] = time.time()
        self.mav = msg

    def _pose_cb(self, msg):
        self.last['pose'] = time.time()
        self.local_pose = msg

    def _vel_cb(self, msg):
        self.local_vel = msg

    def _bat_cb(self, msg):
        self.battery = msg

    def _edm_cb(self, msg):
        self.last['edm'] = time.time()
        self.edm_state = msg.data

    def _cov_cb(self, msg):
        if len(msg.data) > 7:
            self.coverage = float(msg.data[7])

    def _done_cb(self, msg):
        self.explore_done = bool(msg.data)

    def _bspline_cb(self, msg):
        if msg.pos_pts:
            p = msg.pos_pts[-1]
            self.goal_local = (p.x, p.y, p.z)

    def _det_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        now = time.time()
        self.last['yolo'] = now
        self.det = d
        self.det_times = [t for t in self.det_times if now - t < 3.0] + [now]

    def _map_cb(self, msg):
        info = msg.info
        q = info.origin.orientation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.map_pending = {
            'frame': msg.header.frame_id,
            'meta': {'width': info.width, 'height': info.height,
                     'resolution': round(info.resolution, 4),
                     'originX': round(info.origin.position.x, 3),
                     'originY': round(info.origin.position.y, 3),
                     'originYaw': round(yaw, 4)},
            'encoding': 'rle',
            'rle': frontier_and_rle(msg.data, info.width, info.height, MARK_FRONTIER),
        }

    def _surv_cb(self, msg):
        out = []
        for s in msg.survivors:
            out.append({'id': int(s.id), 'grid_x': int(s.grid_x), 'grid_y': int(s.grid_y),
                        'x': round(s.position.x, 3), 'y': round(s.position.y, 3),
                        'z': round(s.position.z, 3), 'confidence': round(float(s.confidence), 3),
                        'n_obs': int(s.n_obs), 'first_seen': s.first_seen.to_sec(),
                        'frame': s.header.frame_id or msg.header.frame_id})
        emit('survivors', survivors=out)

    def _log_cb(self, msg):
        if msg.name in QUIET_NODES or msg.name.startswith('/nidar_gcs_bridge'):
            return
        text = msg.msg
        if msg.level >= Log.WARN or any(p in text for p in INFO_PATTERNS):
            level = {Log.DEBUG: 'DEBUG', Log.INFO: 'INFO', Log.WARN: 'WARN', Log.ERROR: 'ERROR',
                     Log.FATAL: 'FATAL'}.get(msg.level, 'INFO')
            emit('log', level=level, node=msg.name, msg=text[:400], stamp=msg.header.stamp.to_sec())

    def _img_cb(self, msg):
        now = time.time()
        self.last['camera'] = now
        self.cam_times = [t for t in self.cam_times if now - t < 2.0] + [now]
        if not self.cam_enabled or now - self.cam_last_emit < self.cam_period:
            return
        self.cam_last_emit = now
        try:
            if msg.encoding not in ('rgb8', 'bgr8'):
                return
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 3, 3)
            img = img[:, :msg.width]
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()
            w, h = msg.width, msg.height
            boxes = []
            d = self.det
            if d and now - self.last.get('yolo', 0) < 1.0:
                boxes = [dict(b) for b in d.get('boxes', [])]
            if self.cam_flip:
                # The Gazebo sensor is mounted with a 180 deg roll, so the raw image is upside
                # down (PLANNING_DOCS/survivor_localisation_4_3_2026-10-02.md). Show it upright.
                img = cv2.rotate(img, cv2.ROTATE_180)
                for b in boxes:
                    b['xyxy'] = rotate_box_180(b['xyxy'], w, h)
            fps = self._rate(self.cam_times)
            hud = 'SRC: GAZEBO SITL | INFER: ONBOARD (YOLOv8) | FPS: %.1f | LATENCY: %.1fms | ' \
                  'PERSONS: %d' % (fps, (d or {}).get('latency_ms', 0.0), len(boxes))
            draw_detections(img, boxes, hud)
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                emit('frame', width=w, height=h, jpeg=base64.b64encode(jpg.tobytes()).decode('ascii'),
                     boxes=len(boxes))
        except Exception as e:  # never let a bad frame kill the bridge
            rospy.logwarn_throttle(10.0, '[gcs_bridge] frame encode failed: %s', e)

    # -- commands -------------------------------------------------------------------------------

    def _stdin_loop(self):
        for line in sys.stdin:
            try:
                cmd = json.loads(line)
            except ValueError:
                continue
            name = cmd.get('cmd')
            ok, detail = True, ''
            try:
                if name == 'camera':
                    self.cam_enabled = bool(cmd.get('enable'))
                elif name in ('pause', 'unpause'):
                    if Empty is None:
                        raise RuntimeError('gazebo_msgs/std_srvs not available')
                    srv = '/gazebo/%s_physics' % name
                    rospy.wait_for_service(srv, timeout=3.0)
                    rospy.ServiceProxy(srv, Empty)()
                    self.paused = (name == 'pause')
                elif name == 'land':
                    rospy.wait_for_service('/mavros/set_mode', timeout=3.0)
                    res = rospy.ServiceProxy('/mavros/set_mode', SetMode)(custom_mode='AUTO.LAND')
                    ok = bool(res.mode_sent)
                else:
                    ok, detail = False, 'unknown command'
            except Exception as e:
                ok, detail = False, str(e)
            emit('ack', cmd=name, ok=ok, detail=detail)
        os._exit(0)  # stdin closed: the backend is gone

    def _master_watch(self):
        misses = 0
        while True:
            time.sleep(1.0)
            if parent_gone():
                os._exit(0)
            misses = 0 if rosgraph.is_master_online() else misses + 1
            if misses >= 3:
                emit('bridge', status='master_lost')
                os._exit(0)

    # -- periodic snapshot ----------------------------------------------------------------------

    @staticmethod
    def _rate(times):
        if len(times) < 2:
            return 0.0
        return (len(times) - 1) / max(1e-3, times[-1] - times[0])

    def _world_from_local(self):
        if self.T_world_local is None:
            try:
                tr = self.tf_buf.lookup_transform(WORLD, LOCAL, rospy.Time(0))
                q = tr.transform.rotation
                T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
                T[:3, 3] = [tr.transform.translation.x, tr.transform.translation.y,
                            tr.transform.translation.z]
                self.T_world_local = T   # static chain (world->map->camera_init): cache it
            except Exception:
                return None
        return self.T_world_local

    def _poll_paused(self):
        if GetPhysicsProperties is None:
            return
        try:
            rospy.wait_for_service('/gazebo/get_physics_properties', timeout=0.5)
            self.paused = bool(rospy.ServiceProxy('/gazebo/get_physics_properties',
                                                  GetPhysicsProperties)().pause)
        except Exception:
            pass

    def snapshot(self):
        now = time.time()
        T = self._world_from_local()
        pos = vel = att = goal = None
        lp = self.local_pose
        if lp is not None and T is not None:
            p = lp.pose.position
            q = lp.pose.orientation
            pw = T.dot([p.x, p.y, p.z, 1.0])
            Rw = T[:3, :3].dot(tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3])
            M = np.eye(4)
            M[:3, :3] = Rw
            roll, pitch, yaw = tft.euler_from_matrix(M, 'sxyz')
            pos = [round(pw[0], 3), round(pw[1], 3), round(pw[2], 3)]
            att = [round(roll, 4), round(pitch, 4), round(yaw, 4)]
            lv = self.local_vel
            if lv is not None:
                v = T[:3, :3].dot([lv.twist.linear.x, lv.twist.linear.y, lv.twist.linear.z])
                vel = [round(v[0], 3), round(v[1], 3), round(v[2], 3)]
            if self.goal_local is not None:
                g = T.dot(list(self.goal_local) + [1.0])
                goal = [round(g[0], 2), round(g[1], 2), round(g[2], 2)]
        elif lp is not None:
            # world->camera_init is published by the planner launch; until then only the
            # vehicle's local estimate exists. Report it separately rather than mislabel it.
            att = None
        mav = self.mav
        bat = self.battery
        age = {k: round(now - v, 2) for k, v in self.last.items()}
        emit('state',
             position=pos, velocity=vel, attitude=att, goal=goal,
             local_position=None if lp is None else [round(lp.pose.position.x, 3),
                                                     round(lp.pose.position.y, 3),
                                                     round(lp.pose.position.z, 3)],
             mavros=None if mav is None else {'connected': mav.connected, 'armed': mav.armed,
                                              'mode': mav.mode, 'system_status': mav.system_status},
             battery=None if bat is None else {
                 'voltage': None if math.isnan(bat.voltage) else round(bat.voltage, 2),
                 'current': None if math.isnan(bat.current) else round(bat.current, 2),
                 'percentage': None if math.isnan(bat.percentage) else round(bat.percentage, 3)},
             edm_state=self.edm_state, coverage=self.coverage, exploration_done=self.explore_done,
             age=age, camera_fps=round(self._rate(self.cam_times), 1),
             detect_fps=round(self._rate(self.det_times), 1),
             detect_latency_ms=(self.det or {}).get('latency_ms'),
             detect_count=len((self.det or {}).get('boxes', [])) if age.get('yolo', 99) < 1.0 else 0,
             paused=self.paused, sim_time=rospy.Time.now().to_sec())

    def spin(self):
        last_pause_poll = 0.0
        while True:
            t0 = time.time()
            self.snapshot()
            if self.map_pending is not None and t0 - self.map_last_emit >= 1.0:
                m, self.map_pending = self.map_pending, None
                self.map_last_emit = t0
                emit('map', **m)
            if t0 - last_pause_poll > 2.0:
                last_pause_poll = t0
                self._poll_paused()
            time.sleep(max(0.0, 0.1 - (time.time() - t0)))


if __name__ == '__main__':
    Bridge().spin()
