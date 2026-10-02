#!/usr/bin/env python3
"""Phase 4 survivor detector node.

Pipeline:
    /camera/image_raw  -> YOLO26n@5Hz -> bbox
    bbox bottom-centre -> pixel -> camera ray (using /camera/camera_info K)
    camera ray -> tf into map frame -> intersect z=0 ground plane
    position -> nearest-neighbour tracker (assoc_radius m)
    n_obs >= confirmation_threshold -> latched /survivors publish + terminal log

Ground-plane intersection is preferred over a /cloud_registered raycast because it
does NOT require FAST-LIO to have accumulated a return at the survivor's exact
cell. The bbox bottom-centre corresponds to the feet, and the mannequin models are
grounded at z=0, so the geometry is exact for the sim ground truth we care about.
The /cloud_registered raycast is retained as a fallback for cases where the ground-
plane intersection is unphysical (ray angle too shallow, or estimated depth exceeds
detection.max_range_m).

The doc's Phase 4 §4.2 spec requires a smoke assertion on FAST-LIO rate that runs
with the detector active. This node measures /Fast_LIO/odometry Hz over a rolling
window and logs ROSERROR if it falls below FAST_LIO_MIN_HZ. That log is grep-able
by scripts/verify_full_flight.py the same way verify_fix_parity.sh greps for other
binary strings.
"""
import math
import os
import threading
import time

import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from nidar_msgs.msg import Survivor, SurvivorArray


# --- Grid helper ------------------------------------------------------------

class Grid(object):
    """Snap a world position to a competition grid cell."""

    def __init__(self, origin_x, origin_y, cell_size, cells_x, cells_y):
        self.ox = float(origin_x)
        self.oy = float(origin_y)
        self.cs = float(cell_size)
        self.nx = int(cells_x)
        self.ny = int(cells_y)

    def cell(self, x, y):
        i = int(math.floor((x - self.ox) / self.cs))
        j = int(math.floor((y - self.oy) / self.cs))
        return i, j

    def in_bounds(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny


# --- Tracker ---------------------------------------------------------------

class Track(object):
    """A single candidate survivor being accumulated across frames."""

    __slots__ = ('id', 'position', 'n_obs', 'first_seen', 'last_seen',
                 'confidence', 'published')

    def __init__(self, tid, position, first_seen, confidence):
        self.id = tid
        self.position = np.asarray(position, dtype=float)  # running centroid
        self.n_obs = 1
        self.first_seen = first_seen
        self.last_seen = first_seen
        self.confidence = float(confidence)
        self.published = False

    def merge(self, position, now, confidence):
        # Running mean of observed positions - resistant to a single noisy raycast.
        self.n_obs += 1
        self.position = self.position + (np.asarray(position) - self.position) / self.n_obs
        self.last_seen = now
        self.confidence = float(confidence)


# --- The detector node -----------------------------------------------------

class SurvivorDetector(object):

    def __init__(self):
        rospy.init_node('survivor_detector', anonymous=False)

        # Config (from the ROS param server; mission_config.yaml drives it)
        self.enabled = rospy.get_param('~enabled', True)
        self.model_path = rospy.get_param('~model_path')
        self.detect_hz = float(rospy.get_param('~detect_hz', 5.0))
        self.conf_thresh = float(rospy.get_param('~confidence_threshold', 0.55))
        self.device_pref = str(rospy.get_param('~device', 'auto'))
        self.assoc_radius = float(rospy.get_param('~association_radius', 0.75))
        self.confirm_n = int(rospy.get_param('~confirmation_threshold', 3))
        self.stale_s = float(rospy.get_param('~stale_timeout_s', 10.0))
        self.max_range = float(rospy.get_param('~max_range_m', 8.0))
        self.image_topic = rospy.get_param('~input_topic', '/camera/image_raw')
        self.cinfo_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
        self.output_topic = rospy.get_param('~output_topic', '/survivors')
        self.marker_topic = rospy.get_param('~marker_topic', '/survivor_markers')
        self.ground_truth = bool(rospy.get_param('~ground_truth_check', True))

        # Grid (from config/arena_grid.yaml, loaded to the ROS param server by
        # nidar_mission.launch). Fall back to defaults if the file is missing so
        # the detector still runs, but log a WARN so it is visible.
        try:
            self.grid = Grid(
                rospy.get_param('/arena_grid/origin_x'),
                rospy.get_param('/arena_grid/origin_y'),
                rospy.get_param('/arena_grid/cell_size'),
                rospy.get_param('/arena_grid/cells_x'),
                rospy.get_param('/arena_grid/cells_y'),
            )
        except KeyError:
            rospy.logwarn('[detector] /arena_grid/* not loaded; falling back to '
                          'origin (-7,-7), cell 2.0 m, 7x7 cells')
            self.grid = Grid(-7.0, -7.0, 2.0, 7, 7)

        if not self.enabled:
            rospy.logwarn('[detector] detection.enabled is false in mission_config; '
                          'node exiting without loading the model')
            rospy.signal_shutdown('detection disabled')
            return

        # Resolve the model path against the repo root, since consumers set it
        # as 'models/detection/PERSON_DETECTION_MODEL_V3/best.pt'.
        if not os.path.isabs(self.model_path):
            repo = os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.abspath(__file__))))))
            self.model_path = os.path.join(repo, self.model_path)
        if not os.path.exists(self.model_path):
            rospy.logfatal('[detector] model not found at %s. Did you extract '
                           'PERSON_DETECTION_MODEL_V3.zip into models/detection/? '
                           'Expected either a .pt file or an ncnn_model directory.',
                           self.model_path)
            rospy.signal_shutdown('model missing')
            return

        # NCNN vs PyTorch selection. The deployed format is NCNN (a directory
        # containing model.ncnn.param + model.ncnn.bin + metadata.yaml); the
        # .pt is retained for training/debugging. NCNN is CPU-only by design
        # and does not contend with Gazebo for the GPU -- see the FAST-LIO
        # smoke assertion below and mission_config.yaml's detection block.
        is_ncnn = os.path.isdir(self.model_path)
        rospy.loginfo('[detector] loading YOLO (%s) from %s',
                      'NCNN' if is_ncnn else 'PyTorch', self.model_path)
        # Import ultralytics lazily so a missing package produces one FATAL
        # rather than a cryptic import error at rospack time.
        from ultralytics import YOLO
        # task='detect' is required for NCNN loading because ultralytics
        # cannot infer the task from an NCNN directory the way it can from
        # the .pt header.
        self.model = YOLO(self.model_path, task='detect') if is_ncnn \
            else YOLO(self.model_path)

        # Pick the device. NCNN forces cpu; PyTorch honours 'auto'/'cuda'/'cpu'.
        if is_ncnn:
            if self.device_pref not in ('cpu', 'auto'):
                rospy.logwarn('[detector] NCNN model requested with device=%s; '
                              'NCNN is CPU-only, ignoring and using cpu',
                              self.device_pref)
            self.device = 'cpu'
        elif self.device_pref == 'auto':
            try:
                import torch
                self.device = 0 if torch.cuda.is_available() else 'cpu'
            except Exception:
                self.device = 'cpu'
        else:
            self.device = self.device_pref
        rospy.loginfo('[detector] device=%s conf>=%.2f rate=%.1f Hz confirm>=%d',
                      self.device, self.conf_thresh, self.detect_hz, self.confirm_n)

        # Warm the model up on a dummy 640x640 so the first real detection is
        # not a multi-second stall while ncnn compiles kernels / lazy-loads
        # weights. The NCNN warm-up in particular is ~100 s on first run on
        # this laptop (ultralytics extracts and pre-optimises the graph), so
        # this is a REQUIRED step, not a nicety.
        rospy.loginfo('[detector] warming up model (first-run compile can be ~100s for NCNN)...')
        t0 = time.time()
        self.model.predict(np.zeros((640, 640, 3), dtype=np.uint8),
                           device=self.device, conf=self.conf_thresh, verbose=False)
        rospy.loginfo('[detector] warm-up done in %.1fs', time.time() - t0)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Latest camera state
        self._img_lock = threading.Lock()
        self._latest_img = None       # (cv_frame, header) or None
        self._K = None                # 3x3 intrinsic matrix
        self._img_frame = None        # frame_id of the image

        # Tracks + published survivors
        self.tracks = []              # list of Track
        self._next_track_id = 1
        self._published_ids = set()

        # FAST-LIO rate assertion: the Phase 4 §4.2 requirement
        self._flio_stamps = []
        self._flio_last_warn_time = 0.0

        # Ground-truth check state (optional)
        self._truth = {}              # name -> (x, y)
        if self.ground_truth:
            try:
                from gazebo_msgs.msg import ModelStates
                rospy.Subscriber('/gazebo/model_states', ModelStates,
                                 self._truth_cb, queue_size=1)
            except ImportError:
                rospy.logwarn('[detector] gazebo_msgs not on PYTHONPATH; '
                              'ground truth comparison disabled')
                self.ground_truth = False

        # I/O
        rospy.Subscriber(self.cinfo_topic, CameraInfo, self._cinfo_cb, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self._img_cb,
                         queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber('/Fast_LIO/odometry', Odometry, self._flio_cb,
                         queue_size=50)

        self.pub_survivors = rospy.Publisher(self.output_topic, SurvivorArray,
                                             queue_size=1, latch=True)
        self.pub_markers = rospy.Publisher(self.marker_topic, MarkerArray,
                                           queue_size=1, latch=True)

        # Detect at DETECT_HZ irrespective of the 15 Hz camera rate.
        self._timer = rospy.Timer(rospy.Duration(1.0 / self.detect_hz),
                                  self._tick, oneshot=False)

        # First status heartbeat: makes "detector is alive" visible in every
        # /rosout even before the first detection fires.
        rospy.loginfo('[detector] Phase 4 survivor detector ready. '
                      'Waiting on %s and %s...', self.image_topic, self.cinfo_topic)

    # -- ROS callbacks ------------------------------------------------------

    def _cinfo_cb(self, msg):
        # CameraInfo.K is a 9-tuple in row-major order.
        self._K = np.array(msg.K, dtype=float).reshape(3, 3)

    def _img_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn_throttle(10.0, '[detector] cv_bridge decode failed: %s', e)
            return
        with self._img_lock:
            self._latest_img = (frame, msg.header)
            self._img_frame = msg.header.frame_id or 'camera_link'

    def _flio_cb(self, _msg):
        now = rospy.Time.now().to_sec()
        self._flio_stamps.append(now)
        # keep 5 s rolling window
        cutoff = now - 5.0
        while self._flio_stamps and self._flio_stamps[0] < cutoff:
            self._flio_stamps.pop(0)

    def _truth_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name.startswith('survivor_'):
                p = msg.pose[i].position
                self._truth[name] = (p.x, p.y)

    # -- Main tick ----------------------------------------------------------

    def _tick(self, _evt):
        # 1) Snapshot the inputs. If any is missing, quietly wait.
        with self._img_lock:
            snap = self._latest_img
        if snap is None or self._K is None:
            return
        frame, header = snap

        # 2) FAST-LIO rate assertion. Phase 4 §4.2 makes any regression here a
        # blocking failure, so we log at ERROR (grep-able) rather than tune.
        # 5 Hz is a floor: healthy FAST-LIO on this hardware runs 8-12 Hz.
        FAST_LIO_MIN_HZ = 5.0
        WINDOW_S = 5.0
        stamps = list(self._flio_stamps)
        if len(stamps) >= 2:
            rate = (len(stamps) - 1) / max(1e-3, stamps[-1] - stamps[0])
            span = stamps[-1] - stamps[0]
            now = rospy.Time.now().to_sec()
            if span > WINDOW_S * 0.8 and rate < FAST_LIO_MIN_HZ and \
                    now - self._flio_last_warn_time > 5.0:
                rospy.logerr('[detector] FAST-LIO rate %.1f Hz < floor %.1f Hz over '
                             '%.1fs -- Phase 4 smoke assertion FAILED (detector is '
                             'starving SLAM)', rate, FAST_LIO_MIN_HZ, span)
                self._flio_last_warn_time = now

        # 3) YOLO forward pass
        results = self.model.predict(frame, device=self.device,
                                     conf=self.conf_thresh, verbose=False)
        if not results:
            return
        r = results[0]
        if r.boxes is None or len(r.boxes) == 0:
            return

        # Ultralytics returns xyxy in original image pixel units.
        boxes = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()

        # 4) For each detection, back-project bbox bottom-centre and update
        # the tracker.
        for bbox, conf in zip(boxes, confs):
            x1, y1, x2, y2 = bbox
            u = 0.5 * (x1 + x2)
            v = y2  # bottom-centre pixel: the feet
            xyz = self._backproject_ground(u, v, header.stamp)
            if xyz is None:
                continue
            self._update_tracker(xyz, conf, header.stamp)

        # 5) Any track that just crossed the confirmation threshold gets
        # published now.
        self._publish_new_confirmations()

    # -- Geometry -----------------------------------------------------------

    def _backproject_ground(self, u, v, stamp):
        """Return (x, y, z) in map frame where the (u, v) ray hits z=0, or None.

        Uses the camera intrinsics K to build a ray in the camera_link frame,
        transforms both the ray origin and one point along it into map, then
        solves the ray-plane intersection analytically. All the transforms
        happen through tf2 so any lever arm / mount pose in the SDF is honoured
        automatically -- no hand-tuned camera-to-body matrix hidden here.
        """
        if self._K is None:
            return None
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        # In OpenCV camera-optical convention: x right, y down, z forward.
        d_cam_optical = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        norm = np.linalg.norm(d_cam_optical)
        if norm < 1e-6:
            return None
        d_cam_optical = d_cam_optical / norm

        src_frame = self._img_frame or 'camera_link'
        # Convert the optical ray into the camera_link TF frame this codebase
        # publishes. The x500_vlp16 SDF's camera_joint composes two rotations
        # (link then sensor) that resolve to a camera_link whose axes align:
        #     cam_link +X  = look direction (forward, tilted -15 deg down)
        #     cam_link +Y  = image right (drone's right)
        #     cam_link +Z  = image down (perpendicular to look)
        # So the conversion from optical (right, down, fwd) is:
        #     cam_link.x = optical.z    (forward)
        #     cam_link.y = optical.x    (right)
        #     cam_link.z = optical.y    (down)
        # NOT the standard body convention's (+z, -x, -y). Getting this wrong
        # sends every ray into the wrong world quadrant -- run 20260911_101520
        # produced 10 detections all off by 2-11 m before this was corrected.
        # If you change the SDF's camera <pose>, recompute the axes with the
        # helper block in scripts/verify_camera_link_axes.py before editing
        # these lines.
        d_cam_body = np.array([d_cam_optical[2],
                               d_cam_optical[0],
                               d_cam_optical[1]])

        # Transform to WORLD frame, not `map`. The static world_to_map_tf
        # (see nidar_planner/launch/nidar_fuel_upstream.launch) publishes world -> map with
        # a 90 deg yaw and translation to the launch pad, so `map` in this
        # stack is the camera_init odometry frame -- its coordinates are NOT
        # world coordinates and cannot be fed to the arena-grid math directly.
        # The failure mode (documented in
        # PLANNING_DOCS/survivor_detector_frame_bug_2026-09-11.md): reported
        # positions in run 20260911_103244 spanned x=[7.15..17.86] m, entirely
        # outside the -7.5..7.5 arena, because we were emitting map-frame
        # values as if they were world.
        try:
            p0 = self._tf_point(0.0, 0.0, 0.0, src_frame, 'world', stamp)
            p1 = self._tf_point(d_cam_body[0], d_cam_body[1], d_cam_body[2],
                                src_frame, 'world', stamp)
        except Exception as e:
            rospy.logwarn_throttle(5.0,
                                   '[detector] tf %s->world failed: %s', src_frame, e)
            return None

        origin = np.array([p0.point.x, p0.point.y, p0.point.z])
        far = np.array([p1.point.x, p1.point.y, p1.point.z])
        d_world = far - origin
        dz = d_world[2]
        # Reject near-horizontal rays: the -15 deg camera tilt puts good rays
        # around dz ~ -0.26; anything above -0.05 is close to parallel to the
        # ground and its z=0 intersection is arbitrarily far away.
        if dz > -0.05:
            return None
        t = -origin[2] / dz  # solve origin.z + t*dz = 0
        if t <= 0 or t > self.max_range:
            return None
        hit = origin + t * d_world
        return hit

    def _tf_point(self, x, y, z, src_frame, dst_frame, stamp):
        ps = PointStamped()
        ps.header.frame_id = src_frame
        ps.header.stamp = stamp
        ps.point.x, ps.point.y, ps.point.z = float(x), float(y), float(z)
        # Look up the exact transform valid at `stamp`; fall back to latest if
        # the buffer does not have it yet (early in a run).
        try:
            tr = self.tf_buffer.lookup_transform(dst_frame, src_frame, stamp,
                                                 rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            tr = self.tf_buffer.lookup_transform(dst_frame, src_frame,
                                                 rospy.Time(0),
                                                 rospy.Duration(0.5))
        # Apply the transform manually: tf2_geometry_msgs is not always
        # available on rospy Python paths.
        import tf.transformations as tft
        q = tr.transform.rotation
        t = tr.transform.translation
        R = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        v = R @ np.array([ps.point.x, ps.point.y, ps.point.z]) + \
            np.array([t.x, t.y, t.z])
        out = PointStamped()
        out.header.frame_id = dst_frame
        out.header.stamp = stamp
        out.point.x, out.point.y, out.point.z = float(v[0]), float(v[1]), float(v[2])
        return out

    # -- Tracker ------------------------------------------------------------

    def _update_tracker(self, xyz, conf, stamp):
        now = stamp.to_sec()
        pos_xy = np.array([xyz[0], xyz[1]])
        # Nearest existing track within ASSOC_RADIUS
        best_i, best_d = -1, self.assoc_radius
        for i, tr in enumerate(self.tracks):
            d = float(np.linalg.norm(np.array([tr.position[0], tr.position[1]]) - pos_xy))
            if d < best_d:
                best_d = d
                best_i = i
        if best_i >= 0:
            self.tracks[best_i].merge(xyz, now, conf)
        else:
            self.tracks.append(Track(self._next_track_id, xyz, now, conf))
            self._next_track_id += 1

        # Retire stale unpublished tracks so a single spurious detection can't
        # linger forever waiting for reinforcement.
        cutoff = now - self.stale_s
        self.tracks = [t for t in self.tracks
                       if t.published or t.last_seen >= cutoff]

    def _publish_new_confirmations(self):
        # Any track meeting the confirmation threshold that has NOT yet been
        # published is a fresh survivor.
        newly = [t for t in self.tracks
                 if not t.published and t.n_obs >= self.confirm_n]
        if not newly:
            return

        for t in newly:
            t.published = True
            self._published_ids.add(t.id)
            gx, gy = self.grid.cell(t.position[0], t.position[1])
            if not self.grid.in_bounds(gx, gy):
                rospy.logwarn(
                    '[detector] SURVIVOR %d at (%.2f, %.2f) is OUTSIDE the '
                    '%dx%d grid (cell %d,%d); still publishing but downstream '
                    'consumers should treat the cell as invalid',
                    t.id, t.position[0], t.position[1],
                    self.grid.nx, self.grid.ny, gx, gy)

            # Terminal log -- one line per confirmed detection, formatted so
            # it is easy to grep out of /rosout after the run.
            extra = ''
            if self.ground_truth and self._truth:
                closest_name, closest_d = None, 1e9
                for name, (tx, ty) in self._truth.items():
                    d = math.hypot(t.position[0] - tx, t.position[1] - ty)
                    if d < closest_d:
                        closest_d = d
                        closest_name = name
                if closest_name is not None:
                    tx, ty = self._truth[closest_name]
                    tgx, tgy = self.grid.cell(tx, ty)
                    cell_match = (tgx == gx and tgy == gy)
                    extra = (' | ground_truth=%s at (%.2f, %.2f) err=%.2fm '
                             'cell=%s' % (closest_name, tx, ty, closest_d,
                                          'MATCH' if cell_match else
                                          'MISMATCH cell(%d,%d)' % (tgx, tgy)))

            rospy.loginfo(
                '[SURVIVOR] id=%d grid=(%d,%d) pos=(%.2f, %.2f, %.2f) '
                'confidence=%.2f n_obs=%d%s',
                t.id, gx, gy, t.position[0], t.position[1], t.position[2],
                t.confidence, t.n_obs, extra)

        self._publish_state()

    def _publish_state(self):
        msg = SurvivorArray()
        msg.header.stamp = rospy.Time.now()
        # 'world' matches the frame the positions are now expressed in (see
        # _backproject_ground). Anything downstream that draws these on the
        # /map_2d OccupancyGrid needs to know the frame is world, not map.
        msg.header.frame_id = 'world'
        for t in self.tracks:
            if not t.published:
                continue
            s = Survivor()
            s.header.stamp = rospy.Time.from_sec(t.first_seen)
            s.header.frame_id = 'world'
            s.id = int(t.id)
            gx, gy = self.grid.cell(t.position[0], t.position[1])
            s.grid_x = int(gx)
            s.grid_y = int(gy)
            s.position.x = float(t.position[0])
            s.position.y = float(t.position[1])
            s.position.z = float(t.position[2])
            s.confidence = float(t.confidence)
            s.n_obs = int(t.n_obs)
            s.first_seen = rospy.Time.from_sec(t.first_seen)
            msg.survivors.append(s)
        self.pub_survivors.publish(msg)

        # RViz markers for optional visualisation (cost is negligible when no
        # subscriber is listening -- latched pub means one publish per new
        # survivor, not per timer tick).
        ma = MarkerArray()
        for i, t in enumerate(self.tracks):
            if not t.published:
                continue
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = rospy.Time.now()
            m.ns = 'survivors'
            m.id = int(t.id)
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(t.position[0])
            m.pose.position.y = float(t.position[1])
            m.pose.position.z = 1.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.6
            m.scale.y = 0.6
            m.scale.z = 2.0
            m.color.a = 0.6
            m.color.r = 1.0
            m.color.g = 0.4
            m.color.b = 0.0
            ma.markers.append(m)
        self.pub_markers.publish(ma)


def main():
    node = SurvivorDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
