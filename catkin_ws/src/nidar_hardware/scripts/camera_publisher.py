#!/usr/bin/env python3
"""Real camera -> the topics the simulation's Gazebo camera provides, plus a GCS stream.

  /camera/image_raw            sensor_msgs/Image bgr8, frame camera_link, hardware.yaml
                               camera.publish_fps (the survivor detector samples it)
  /camera/camera_info          sensor_msgs/CameraInfo (calibration file, or derived from the FOV)
  /nidar/gcs/camera/compressed sensor_msgs/CompressedImage JPEG, scaled and throttled
                               (camera.gcs_stream) -- what the GCS shows over Wi-Fi

camera.source picks the device: /dev/videoN (USB/UVC camera or HDMI capture dongle; asks for
MJPG, which UVC cameras need for 720p30), rtsp://... (IP camera such as the SIYI A8 mini), or
gst:<pipeline> (CSI camera through nvarguscamerasrc; needs an L4T image with GStreamer).

With a calibration file (camera_calibration's ost.yaml) frames are undistorted before
publishing and CameraInfo carries the undistorted pinhole model: the detector back-projects
with K alone, so lens distortion would otherwise bend every survivor position near the edges.
`camera_link` follows the detector's convention (x along the optical axis, y left, z up); its
pose comes from hw_static_tf.py (camera.mount).
"""
import math
import os
import threading
import time

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def fov_intrinsics(width, height, hfov_deg):
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return np.array([[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0], [0.0, 0.0, 1.0]])


def load_calibration(path, width, height):
    """(K, D) from a ROS camera_calibration yaml, rescaled if calibrated at another resolution."""
    with open(path) as f:
        c = yaml.safe_load(f)
    K = np.array(c['camera_matrix']['data'], dtype=float).reshape(3, 3)
    D = np.array(c['distortion_coefficients']['data'], dtype=float)
    sx, sy = width / float(c['image_width']), height / float(c['image_height'])
    K[0, :] *= sx
    K[1, :] *= sy
    return K, D


def open_capture(source, width, height, fps):
    if source.startswith('gst:'):
        return cv2.VideoCapture(source[4:], cv2.CAP_GSTREAMER)
    if source.startswith('/dev/video'):
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap
    os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'rtsp_transport;tcp|stimeout;3000000')
    cap = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


class CameraPublisher(object):
    def __init__(self):
        p = lambda k, d: rospy.get_param('/hardware/camera/' + k, d)  # noqa: E731
        self.source = str(p('source', '/dev/video0'))
        self.width, self.height = int(p('width', 1280)), int(p('height', 720))
        self.fps = float(p('fps', 30))
        self.publish_period = 1.0 / max(0.5, float(p('publish_fps', 10)))
        gcs = p('gcs_stream', {}) or {}
        self.gcs_period = 1.0 / max(0.2, float(gcs.get('fps', 5)))
        self.gcs_width = int(gcs.get('width', 640))
        self.gcs_quality = int(gcs.get('jpeg_quality', 60))
        self.hfov = float(p('horizontal_fov_deg', 81.0))
        self.calib = str(p('calibration_file', '') or '')
        self.frame_id = 'camera_link'

        self.pub_img = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.pub_info = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1)
        self.pub_gcs = rospy.Publisher('/nidar/gcs/camera/compressed', CompressedImage, queue_size=1)
        self.lock = threading.Lock()
        self.frame = None
        self.frame_stamp = None
        self.maps = None
        self.K = None

    def _setup_model(self, w, h):
        """Intrinsics for the frames actually delivered (a device may ignore the request)."""
        if self.calib:
            K, D = load_calibration(self.calib, w, h)
            newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0.0)
            self.maps = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_16SC2)
            self.K = newK
            rospy.loginfo('[camera] calibration %s: undistorting, fx=%.1f', self.calib, newK[0, 0])
        else:
            self.K = fov_intrinsics(w, h, self.hfov)
            rospy.logwarn('[camera] no calibration_file: intrinsics from %.0f deg FOV (fx=%.1f). '
                          'Calibrate for accurate survivor positions.', self.hfov, self.K[0, 0])
        info = CameraInfo(width=w, height=h, distortion_model='plumb_bob', D=[0.0] * 5)
        info.K = self.K.flatten().tolist()
        info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        info.P = [self.K[0, 0], 0, self.K[0, 2], 0, 0, self.K[1, 1], self.K[1, 2], 0, 0, 0, 1, 0]
        info.header.frame_id = self.frame_id
        self.info = info

    def capture_loop(self):
        while not rospy.is_shutdown():
            cap = open_capture(self.source, self.width, self.height, self.fps)
            if not cap.isOpened():
                rospy.logerr_throttle(10.0, '[camera] cannot open %s -- retrying' % self.source)
                time.sleep(2.0)
                continue
            rospy.loginfo('[camera] %s open', self.source)
            while not rospy.is_shutdown():
                ok, frame = cap.read()
                if not ok or frame is None:
                    rospy.logwarn('[camera] %s stopped delivering frames -- reopening', self.source)
                    break
                with self.lock:
                    self.frame, self.frame_stamp = frame, rospy.Time.now()
            cap.release()
            time.sleep(1.0)

    def publish_loop(self):
        last_pub = last_gcs = 0.0
        last_seen = None
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
            with self.lock:
                frame, stamp = self.frame, self.frame_stamp
            if frame is None or stamp is last_seen:
                continue
            now = time.time()
            due_pub = now - last_pub >= self.publish_period
            due_gcs = now - last_gcs >= self.gcs_period and self.pub_gcs.get_num_connections() > 0
            if not (due_pub or due_gcs):
                continue
            last_seen = stamp
            h, w = frame.shape[:2]
            if self.K is None or self.info.width != w or self.info.height != h:
                self._setup_model(w, h)
            if self.maps is not None:
                frame = cv2.remap(frame, self.maps[0], self.maps[1], cv2.INTER_LINEAR)
            if due_pub:
                last_pub = now
                img = Image(height=h, width=w, encoding='bgr8', is_bigendian=0, step=w * 3,
                            data=frame.tobytes())
                img.header.stamp = stamp
                img.header.frame_id = self.frame_id
                self.info.header.stamp = stamp
                self.pub_img.publish(img)
                self.pub_info.publish(self.info)
            if due_gcs:
                last_gcs = now
                small = frame
                if w > self.gcs_width:
                    small = cv2.resize(frame, (self.gcs_width, int(h * self.gcs_width / float(w))),
                                       interpolation=cv2.INTER_AREA)
                ok, jpg = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, self.gcs_quality])
                if ok:
                    msg = CompressedImage(format='jpeg', data=jpg.tobytes())
                    msg.header.stamp = stamp
                    msg.header.frame_id = self.frame_id
                    self.pub_gcs.publish(msg)


if __name__ == '__main__':
    rospy.init_node('camera')
    node = CameraPublisher()
    threading.Thread(target=node.capture_loop, daemon=True).start()
    node.publish_loop()
