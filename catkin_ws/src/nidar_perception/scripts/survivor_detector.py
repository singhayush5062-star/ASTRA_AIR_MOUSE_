#!/usr/bin/env python3
"""Phase 4 survivor detector node (phase plan 4.2 detection + 4.3 3D localisation).

Pipeline:
    /camera/image_raw  -> YOLO@5Hz -> bbox
    gate: drop boxes cut off by the image border, or too small to trust
    bbox CENTRE pixel -> ray in camera_link (x fwd, y LEFT, z UP -- Gazebo's sensor frame)
    ray -> tf into `world` -> intersect the horizontal plane z = target_height_m
    position -> nearest-neighbour tracker (association_radius m)
    n_obs >= confirmation_threshold -> latched /survivors publish + terminal log

Why the bbox CENTRE on a body-height plane, and not the bbox bottom on the ground. The first
version aimed the bottom-centre pixel at z=0 ("the feet"). That is wrong here for two reasons
found by ground-truth experiments (PLANNING_DOCS/survivor_localisation_4_3_2026-10-02.md):
  1. The x500_vlp16 SDF rolls the camera by 180 deg, so the RAW IMAGE IS UPSIDE-DOWN (rotate a
     frame by 180 deg and the standing T-pose mannequin is upright). The bottom of a box is
     therefore the person's HEAD, and intersecting a head ray with the floor overshoots by
     metres. The box centre is the body centre in either orientation.
  2. The pixel->ray mapping assumed an upright image (see pixel_ray_camera_link).
Aiming the box centre at a plane through the body's centre cut the median error from ~5 m to
~0.4 m. A real dummy needs its own detection.target_height_m.

Why the frame is de-rotated before YOLO (rotate_180, on in the sim). The first model
(PERSON_DETECTION_MODEL_V3) detected about as well on the upside-down frame as on a de-rotated
one (15 vs 14 of 40 poses). YOLO26S_DRONE_PERSON_V1 was trained on upright drone imagery
without vertical-flip augmentation and does not: on 158 sim frames of a survivor it found the
person in 146 upright frames and 0 upside-down ones at confidence >= 0.55. The boxes are rotated
back into raw-image pixels straight away, so everything downstream (localisation, gates, the
GCS overlay) still works in raw-image coordinates. A camera mounted upright sets it false.

Why no /cloud_registered raycast (the phase plan suggests one). Measured in the same experiment:
for 25 of 37 detections the lidar returned NO point within 0.9 m of the survivor (a VLP-16 fan of
+-15 deg passes over a body this low until ~4.5 m), and where points did exist they were mostly
wall points, which made the position worse (median 0.91 m vs 0.59 m for the plane estimate). A
line-of-sight check against the cloud rejected nothing the image-border and range gates did not.

The doc's Phase 4 section 4.2 spec requires a smoke assertion on FAST-LIO rate that runs
with the detector active. This node measures /Fast_LIO/odometry Hz over a rolling
window and logs ROSERROR if it falls below FAST_LIO_MIN_HZ. That log is grep-able
by scripts/verify_full_flight.py the same way verify_fix_parity.sh greps for other
binary strings.

In simulation (detection.ground_truth_check) it also compares every confirmed tag against the
Gazebo survivor models and logs a [LOC-CHECK] line (cell match count, position error).
"""
import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String
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


# --- Geometry (pure functions; unit-tested in nidar_perception/test) ----------

def pixel_ray_camera_link(u, v, K):
    """Unit ray through pixel (u, v), expressed in the camera_link frame.

    camera_link is the frame the x500_vlp16 SDF and the static TF chain in detector.launch
    publish, i.e. Gazebo's SENSOR frame, and Gazebo renders pixels against that frame with its
    standard camera model: the optical axis is +X, a pixel to the RIGHT of the principal point
    lies toward -Y and a pixel BELOW it toward -Z. That holds whatever the SDF rolls the
    sensor by, so this mapping does not need to know the camera is mounted upside-down.

    (Until 2026-10-02 this was [oz, +ox, +oy], which assumes an upright camera with +Y right and
    +Z down. It agrees with the true ray at the image centre and nowhere else: up to 90 deg off,
    median 37 deg, which is why tags drifted toward the arena centre.)
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    d = np.array([1.0, -(u - cx) / fx, -(v - cy) / fy])
    return d / np.linalg.norm(d)


def intersect_plane(origin, direction, z_plane, max_range, min_down=0.05):
    """Where the ray origin + t*direction (t>0) crosses the horizontal plane z = z_plane.

    Returns the 3-vector hit, or None when the ray is too close to horizontal to give a
    meaningful intersection (direction z above -min_down), points away from the plane, or the
    horizontal distance to the hit exceeds max_range.
    """
    dz = direction[2]
    if dz > -min_down:
        return None
    t = (z_plane - origin[2]) / dz
    if t <= 0:
        return None
    hit = origin + t * direction
    if math.hypot(hit[0] - origin[0], hit[1] - origin[1]) > max_range:
        return None
    return hit


def rotate_boxes_180(boxes, width, height):
    """xyxy boxes found on a frame rotated by 180 deg -> the same boxes in the raw frame."""
    b = np.asarray(boxes, dtype=float).reshape(-1, 4)
    return np.stack([width - b[:, 2], height - b[:, 3], width - b[:, 0], height - b[:, 1]], axis=1)


def box_is_usable(bbox, width, height, border_px, min_side_px):
    """Whether a YOLO box is trustworthy enough to localise from.

    A box touching the image border is truncated, so its centre is biased by an unknown
    amount: in the ground-truth experiment 10 of the 11 detections that were more than a grid
    cell off were border-truncated. A box with a tiny side is a speck (typically texture on a
    wall), not a body.
    """
    x1, y1, x2, y2 = bbox
    if x1 <= border_px or y1 <= border_px or x2 >= width - border_px or y2 >= height - border_px:
        return False, 'border'
    if (x2 - x1) < min_side_px or (y2 - y1) < min_side_px:
        return False, 'small'
    return True, None


# --- Tracker ---------------------------------------------------------------

def merge_into_published(tracks, new, radius):
    """Fold a just-confirmed track into an already-published one that lies within `radius`.

    Two observation clusters of the SAME person can sit just beyond the association radius of
    each other (0.77 m vs 0.75 m in the 2026-10-02 flight), which tags one survivor two or
    three times. Every extra tag is a false positive. Returns the surviving published track
    (position and n_obs updated, observation-weighted), or None when `new` is a distinct
    survivor. The caller removes `new` from its list.
    """
    best, best_d = None, radius
    for p in tracks:
        if p is new or not p.published:
            continue
        d = math.hypot(p.position[0] - new.position[0], p.position[1] - new.position[1])
        if d < best_d:
            best, best_d = p, d
    if best is None:
        return None
    n = best.n_obs + new.n_obs
    best.position = (best.position * best.n_obs + new.position * new.n_obs) / n
    best.n_obs = n
    best.last_seen = max(best.last_seen, new.last_seen)
    best.confidence = max(best.confidence, new.confidence)
    return best


class Track(object):
    """A single candidate survivor being accumulated across frames."""

    __slots__ = ('id', 'position', 'n_obs', 'first_seen', 'last_seen',
                 'confidence', 'published', 'cell')

    def __init__(self, tid, position, first_seen, confidence):
        self.id = tid
        self.position = np.asarray(position, dtype=float)  # running centroid
        self.n_obs = 1
        self.first_seen = first_seen
        self.last_seen = first_seen
        self.confidence = float(confidence)
        self.published = False
        self.cell = None          # grid cell last published for this track

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
        # The sim camera is mounted rolled 180 deg; run YOLO on the upright frame (module docstring)
        self.rotate_180 = bool(rospy.get_param('~rotate_180', False))
        # Localisation (see the module docstring for how these were chosen)
        self.target_height = float(rospy.get_param('~target_height_m', 0.35))
        self.border_px = int(rospy.get_param('~border_margin_px', 6))
        self.min_side_px = int(rospy.get_param('~min_box_px', 20))
        self.arena_margin = float(rospy.get_param('~arena_margin_m', 0.3))
        # A newly confirmed track within this distance of an already-published tag is the same
        # person, not a new survivor. Must stay below the closest spacing of two real survivors.
        self.merge_radius = float(rospy.get_param('~merge_radius_m', 1.5))

        # Grid (from nidar_config/config/arena_grid.yaml, loaded to the ROS param server by
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
        # repo-relative (nidar_perception/launch/detector.launch passes an absolute $(find) path).
        if not os.path.isabs(self.model_path):
            repo = os.path.dirname(os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.abspath(__file__))))))
            self.model_path = os.path.join(repo, self.model_path)
        if not os.path.exists(self.model_path):
            rospy.logfatal('[detector] model not found at %s. The weights ship under '
                           'nidar_perception/models/detection/. '
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
        rospy.loginfo('[detector] device=%s conf>=%.2f rate=%.1f Hz confirm>=%d rotate_180=%s',
                      self.device, self.conf_thresh, self.detect_hz, self.confirm_n,
                      self.rotate_180)

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
        self._dirty = False           # a published track moved: republish /survivors
        self._rejected = {'border': 0, 'small': 0, 'ray': 0, 'arena': 0}
        self._last_loc_report = 0.0

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
        # Raw per-frame boxes for display (the GCS camera overlay). JSON in a std_msgs/String:
        # {"stamp", "width", "height", "latency_ms", "boxes": [{"xyxy", "conf", "usable"}]}.
        # Published every inference tick, empty frames included, so a consumer can clear stale
        # boxes. Nothing in the mission consumes it; /survivors stays the authoritative output.
        self.pub_detections = rospy.Publisher(
            rospy.get_param('~detections_topic', '/survivor_detector/detections'), String,
            queue_size=1)

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
        t_infer = time.time()
        results = self.model.predict(cv2.rotate(frame, cv2.ROTATE_180) if self.rotate_180
                                     else frame, device=self.device,
                                     conf=self.conf_thresh, verbose=False)
        latency_ms = 1000.0 * (time.time() - t_infer)
        height, width = frame.shape[:2]
        r = results[0] if results else None
        if r is None or r.boxes is None or len(r.boxes) == 0:
            self._publish_detections(header, width, height, latency_ms, [], [])
            return

        # Ultralytics returns xyxy in original image pixel units.
        boxes = r.boxes.xyxy.cpu().numpy()
        if self.rotate_180:
            boxes = rotate_boxes_180(boxes, width, height)
        confs = r.boxes.conf.cpu().numpy()
        self._publish_detections(header, width, height, latency_ms, boxes, confs)

        # 4) For each usable detection, localise the bbox centre and update the tracker.
        for bbox, conf in zip(boxes, confs):
            ok, why = box_is_usable(bbox, width, height, self.border_px, self.min_side_px)
            if not ok:
                self._rejected[why] += 1
                continue
            xyz = self._localise(bbox, header.stamp)
            if xyz is None:
                self._rejected['ray'] += 1
                continue
            if not self._inside_arena(xyz[0], xyz[1]):
                self._rejected['arena'] += 1
                rospy.logdebug('[detector] dropped detection at (%.2f, %.2f): outside the arena',
                               xyz[0], xyz[1])
                continue
            self._update_tracker(xyz, conf, header.stamp)

        # 5) Any track that just crossed the confirmation threshold gets published now; a
        # published track whose cell changed (or that gained 5 more observations) republishes.
        self._publish_new_confirmations()
        if self._dirty:
            self._dirty = False
            self._publish_state()
        self._loc_check_report()

    def _publish_detections(self, header, width, height, latency_ms, boxes, confs):
        if self.pub_detections.get_num_connections() == 0:
            return
        self.pub_detections.publish(String(data=json.dumps({
            'stamp': header.stamp.to_sec(),
            'width': int(width),
            'height': int(height),
            'latency_ms': round(latency_ms, 1),
            'boxes': [{'xyxy': [round(float(c), 1) for c in b],
                       'conf': round(float(cf), 3),
                       'usable': bool(box_is_usable(b, width, height, self.border_px,
                                                    self.min_side_px)[0])}
                      for b, cf in zip(boxes, confs)],
        })))

    # -- Geometry -----------------------------------------------------------

    def _localise(self, bbox, stamp):
        """World-frame (x, y, 0) of the survivor behind `bbox`, or None.

        Aims the bbox CENTRE pixel through the camera and intersects the plane z =
        target_height_m (the height of the body's centre), then reports the point at ground
        level (z = 0) so /survivors keeps meaning "where on the floor". Every transform goes
        through tf2, so the SDF mount pose and FAST-LIO's pose are honoured automatically.

        Positions are expressed in `world`, not `map`: the static world_to_map_tf
        (nidar_planner/launch/nidar_fuel_upstream.launch) rotates and translates, so `map` is
        the camera_init odometry frame and its coordinates are NOT arena coordinates. Feeding
        map-frame values to the grid math put reported positions at x=[7.15..17.86] m, entirely
        outside the arena (PLANNING_DOCS/survivor_detector_frame_bug_2026-09-11.md).
        """
        if self._K is None:
            return None
        u = 0.5 * (bbox[0] + bbox[2])
        v = 0.5 * (bbox[1] + bbox[3])
        d_link = pixel_ray_camera_link(u, v, self._K)

        src_frame = self._img_frame or 'camera_link'
        try:
            R, t = self._world_from(src_frame, stamp)
        except Exception as e:
            rospy.logwarn_throttle(5.0, '[detector] tf %s->world failed: %s', src_frame, e)
            return None
        origin = t
        direction = R @ d_link
        hit = intersect_plane(origin, direction, self.target_height, self.max_range)
        if hit is None:
            return None
        return np.array([hit[0], hit[1], 0.0])

    def _world_from(self, src_frame, stamp):
        """(R, t) taking a point in src_frame to the `world` frame at `stamp`."""
        # Look up the exact transform valid at `stamp`; fall back to latest if the buffer does
        # not have it yet (early in a run).
        try:
            tr = self.tf_buffer.lookup_transform('world', src_frame, stamp,
                                                 rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            tr = self.tf_buffer.lookup_transform('world', src_frame,
                                                 rospy.Time(0), rospy.Duration(0.5))
        # Apply the transform manually: tf2_geometry_msgs is not always available on rospy
        # Python paths.
        import tf.transformations as tft
        q = tr.transform.rotation
        t = tr.transform.translation
        R = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        return R, np.array([t.x, t.y, t.z])

    def _inside_arena(self, x, y):
        """Within the competition grid, plus a small margin for localisation error."""
        m = self.arena_margin
        return (self.grid.ox - m <= x <= self.grid.ox + self.grid.nx * self.grid.cs + m and
                self.grid.oy - m <= y <= self.grid.oy + self.grid.ny * self.grid.cs + m)

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
            tr = self.tracks[best_i]
            tr.merge(xyz, now, conf)
            if tr.published:
                # The position keeps refining after confirmation (running mean over more
                # viewpoints). Republish when the cell changes or every 5th observation.
                cell = self.grid.cell(tr.position[0], tr.position[1])
                if cell != tr.cell or tr.n_obs % 5 == 0:
                    if cell != tr.cell:
                        rospy.loginfo('[SURVIVOR] id=%d moved to grid=(%d,%d) pos=(%.2f, %.2f) '
                                      'n_obs=%d', tr.id, cell[0], cell[1], tr.position[0],
                                      tr.position[1], tr.n_obs)
                    tr.cell = cell
                    self._dirty = True
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
            dup = merge_into_published(self.tracks, t, self.merge_radius)
            if dup is not None:
                self.tracks.remove(t)
                dup.cell = self.grid.cell(dup.position[0], dup.position[1])
                self._dirty = True
                rospy.loginfo('[SURVIVOR] track %d merged into id=%d (same person, %.2f m apart): '
                              'pos=(%.2f, %.2f) n_obs=%d', t.id, dup.id,
                              math.hypot(t.position[0] - dup.position[0],
                                         t.position[1] - dup.position[1]),
                              dup.position[0], dup.position[1], dup.n_obs)
                continue
            t.published = True
            self._published_ids.add(t.id)
            gx, gy = self.grid.cell(t.position[0], t.position[1])
            t.cell = (gx, gy)
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

    def _loc_check_report(self):
        """Sim only: compare every confirmed tag with the Gazebo survivor it is nearest to.

        Phase plan 4.3 asks for a sim-only assertion of the tagged cell against ground truth.
        Logs one [LOC-CHECK] line every 20 s while any tag exists; WARNs (grep-able) when a tag
        sits in a different cell from its nearest survivor.
        """
        if not (self.ground_truth and self._truth):
            return
        now = time.time()
        if now - self._last_loc_report < 20.0:
            return
        pub = [t for t in self.tracks if t.published]
        if not pub:
            return
        self._last_loc_report = now
        matched, errs, covered, bad = 0, [], set(), []
        for t in pub:
            name, (tx, ty) = min(self._truth.items(),
                                 key=lambda kv: math.hypot(t.position[0] - kv[1][0],
                                                           t.position[1] - kv[1][1]))
            err = math.hypot(t.position[0] - tx, t.position[1] - ty)
            errs.append(err)
            if self.grid.cell(t.position[0], t.position[1]) == self.grid.cell(tx, ty):
                matched += 1
                covered.add(name)
            else:
                bad.append('id%d~%s(%.1fm)' % (t.id, name, err))
        line = ('[LOC-CHECK] tags=%d cell_match=%d/%d survivors_covered=%d/%d '
                'median_err=%.2fm max_err=%.2fm rejected=%s'
                % (len(pub), matched, len(pub), len(covered), len(self._truth),
                   float(np.median(errs)), max(errs), dict(self._rejected)))
        if bad:
            rospy.logwarn('%s | assertion FAILED, wrong cell: %s', line, ', '.join(bad))
        else:
            rospy.loginfo(line)

    def _publish_state(self):
        msg = SurvivorArray()
        msg.header.stamp = rospy.Time.now()
        # 'world' matches the frame the positions are now expressed in (see
        # _localise). Anything downstream that draws these on the
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
