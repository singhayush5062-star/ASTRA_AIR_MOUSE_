"""
VisionService — Real-Time FC/Jetson Camera Ingestion & Laptop YOLO Inference
=============================================================================
Architecture:
  FC / Jetson Camera ──(Network: RTSP/HTTP/UDP)──> Laptop VisionService
                                                          │
                                                Frame Decoder & Preprocess
                                                          │
                                                YOLO26s (Laptop Inference)
                                                          │
                                                Annotated MJPEG & WebSockets
                                                          │
                                                          ▼
                                                  Dashboard HUD

IMPORTANT:
  - The laptop webcam is NEVER used. A local capture device is opened only when the operator
    sets the stream URL to it explicitly (/dev/videoN or v4l2:///dev/videoN): that is how the
    video output of an RC ground unit (e.g. the T12) or an HDMI capture dongle arrives.
  - The FC / Jetson stream is the sole source of camera truth.
  - If FC camera is offline, the service enters OFFLINE/RECONNECTING state
    without feeding stale frames or falling back to local devices.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Any, Generator

import cv2
import numpy as np

from app.core.config import settings

try:
    import onnxruntime as ort
except ImportError:
    ort = None


class VisionService:
    def __init__(self, weights_path: str = "weights/best.onnx") -> None:
        self.weights_path = weights_path
        self.session: ort.InferenceSession | None = None
        self.model_loaded = False
        self.model_name = "YOLO26s-Person (Laptop)"
        self.input_size = (960, 960)
        self.conf_threshold = 0.35

        # FC / Jetson Camera Network Stream Configuration
        self.fc_host: str = getattr(settings, "FC_HOST", "192.168.1.100")
        self.stream_url: str = getattr(settings, "FC_CAMERA_URL", "rtsp://192.168.1.100:8554/live")
        self.reconnect_interval: float = getattr(settings, "FC_RECONNECT_INTERVAL", 3.0)

        # Video Capture Client (Network Stream only — NEVER local webcam)
        self.cap: cv2.VideoCapture | None = None
        self.is_running = False
        self.camera_status: str = "OFFLINE"  # OFFLINE | CONNECTING | CONNECTED | RECONNECTING
        self.receiving_frames = False
        self.lock = threading.Lock()

        # Telemetry & Performance
        self.latest_annotated_jpeg: bytes | None = None
        self.latest_detections: list[dict[str, Any]] = []
        self.fps: float = 0.0
        self.latest_latency_ms: float = 0.0
        self.frame_count: int = 0
        self.last_frame_time: float = 0.0
        self.detected_person_count: int = 0

        self._load_model()

    def _load_model(self) -> None:
        """Load YOLO26s ONNX model for execution on the laptop."""
        if ort is None:
            print("[Vision] onnxruntime not installed; AI model inference disabled.")
            return

        if not os.path.exists(self.weights_path):
            print(f"[Vision] Model weights not found at {self.weights_path}")
            return

        try:
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = 4
            opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.session = ort.InferenceSession(self.weights_path, sess_options=opts)

            inputs = self.session.get_inputs()
            if inputs:
                shape = inputs[0].shape
                if len(shape) == 4 and isinstance(shape[2], int) and isinstance(shape[3], int):
                    self.input_size = (shape[3], shape[2])

            meta = self.session.get_modelmeta().custom_metadata_map
            self.model_name = meta.get("description", "YOLO26s-Person (Laptop)")
            self.model_loaded = True
            print(f"[Vision] YOLO26s model loaded on Laptop ({self.input_size}) from {self.weights_path}")
        except Exception as e:
            print(f"[Vision] Failed to initialize model on laptop: {e}")
            self.model_loaded = False

    def update_config(self, stream_url: str | None = None, fc_host: str | None = None) -> None:
        """Dynamically update FC camera stream URL or host from GCS dashboard."""
        with self.lock:
            restart_needed = False
            if stream_url and stream_url != self.stream_url:
                self.stream_url = stream_url.strip()
                restart_needed = True
            if fc_host and fc_host != self.fc_host:
                self.fc_host = fc_host.strip()

            if restart_needed and self.is_running:
                print(f"[Vision] Reconnecting to updated FC camera stream: {self.stream_url}")
                if self.cap:
                    self.cap.release()
                    self.cap = None
                self.camera_status = "CONNECTING"
                self.receiving_frames = False
                self.latest_annotated_jpeg = None

    def start_camera(self, stream_url: str | None = None) -> bool:
        """Start background receiver thread for the FC/Jetson stream."""
        with self.lock:
            if stream_url:
                self.stream_url = stream_url.strip()

            if self.is_running:
                return True

            self.is_running = True
            self.camera_status = "CONNECTING"
            self.receiving_frames = False
            self.latest_annotated_jpeg = None

            thread = threading.Thread(target=self._fc_stream_receiver_loop, daemon=True)
            thread.start()
            print(f"[Vision] FC Camera receiver started. Connecting to {self.stream_url}...")
            return True

    def stop_camera(self) -> None:
        """Stop receiver and disconnect from FC/Jetson stream."""
        with self.lock:
            self.is_running = False
            self.camera_status = "OFFLINE"
            self.receiving_frames = False
            self.latest_annotated_jpeg = None
            if self.cap:
                self.cap.release()
                self.cap = None
            print("[Vision] FC Camera receiver stopped.")

    def _local_device(self) -> str | None:
        """/dev/videoN path when the configured stream is an explicit local capture device."""
        url = (self.stream_url or "").strip()
        if url.startswith("v4l2://"):
            url = url[len("v4l2://"):]
        return url if url.startswith("/dev/video") else None

    def _is_stream_reachable(self) -> bool:
        """Fast non-blocking socket probe to verify FC/Jetson stream port is open."""
        dev = self._local_device()
        if dev is not None:
            return os.path.exists(dev)
        try:
            import socket
            from urllib.parse import urlparse
            parsed = urlparse(self.stream_url)
            host = parsed.hostname or self.fc_host
            port = parsed.port or (8554 if parsed.scheme == 'rtsp' else 8080 if parsed.scheme == 'http' else 554)
            if not host:
                return False
            with socket.create_connection((host, port), timeout=0.4):
                return True
        except Exception:
            return False

    def _open_fc_stream(self) -> bool:
        """Attempt connection to FC/Jetson network camera stream (RTSP/HTTP/UDP)."""
        # Fast reachability check to prevent socket hanging when FC is offline
        if not self._is_stream_reachable():
            self.camera_status = "OFFLINE"
            return False

        # Set low FFmpeg timeout (2 seconds = 2000000 us) and TCP transport
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|stimeout;2000000"

        print(f"[Vision] Connecting to FC/Jetson stream: {self.stream_url}")
        dev = self._local_device()
        if dev is not None:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        else:
            cap = cv2.VideoCapture(self.stream_url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if cap.isOpened():
            self.cap = cap
            self.camera_status = "CONNECTED"
            print(f"[Vision] FC Camera link ESTABLISHED ({self.stream_url})")
            return True
        else:
            cap.release()
            self.cap = None
            self.camera_status = "OFFLINE"
            return False

    def _fc_stream_receiver_loop(self) -> None:
        """
        Continuous loop receiving frames exclusively from FC/Jetson stream,
        running laptop-side YOLO26s inference, and rendering dashboard HUD.
        """
        prev_time = time.time()

        while self.is_running:
            # 1. Connect or Reconnect if stream is closed
            if self.cap is None or not self.cap.isOpened():
                self.camera_status = "RECONNECTING"
                self.receiving_frames = False
                self.latest_annotated_jpeg = None
                self.latest_detections = []
                connected = self._open_fc_stream()
                if not connected:
                    # Wait before retry without blocking or spiking CPU
                    time.sleep(self.reconnect_interval)
                    continue

            # 2. Grab frame from FC Camera
            ret, frame = self.cap.read()
            if not ret or frame is None:
                # Stream broken or packet lost
                print(f"[Vision] Lost frame from FC stream: {self.stream_url}. Reconnecting...")
                if self.cap:
                    self.cap.release()
                    self.cap = None
                self.camera_status = "RECONNECTING"
                self.receiving_frames = False
                self.latest_annotated_jpeg = None
                self.latest_detections = []
                time.sleep(self.reconnect_interval)
                continue

            self.camera_status = "CONNECTED"
            self.receiving_frames = True
            self.last_frame_time = time.time()
            orig_h, orig_w = frame.shape[:2]

            # 3. Laptop-Side YOLO26s Model Inference
            detections: list[dict[str, Any]] = []
            latency_ms = 0.0

            if self.model_loaded and self.session is not None:
                try:
                    t_start = time.perf_counter()

                    in_w, in_h = self.input_size
                    resized = cv2.resize(frame, (in_w, in_h))
                    blob = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)
                    blob = np.expand_dims(blob.astype(np.float32) / 255.0, 0)

                    input_name = self.session.get_inputs()[0].name
                    preds = self.session.run(None, {input_name: blob})[0]

                    t_end = time.perf_counter()
                    latency_ms = (t_end - t_start) * 1000.0

                    scale_x = orig_w / in_w
                    scale_y = orig_h / in_h

                    for row in preds[0]:
                        x1, y1, x2, y2, score, cls_id = row
                        if score >= self.conf_threshold:
                            bx1 = max(0, int(x1 * scale_x))
                            by1 = max(0, int(y1 * scale_y))
                            bx2 = min(orig_w, int(x2 * scale_x))
                            by2 = min(orig_h, int(y2 * scale_y))
                            bw = bx2 - bx1
                            bh = by2 - by1

                            detections.append({
                                "label": "SURVIVOR",
                                "class_id": int(cls_id),
                                "confidence": float(score),
                                "box": [bx1, by1, bw, bh],
                                "normalized_box": [
                                    bx1 / orig_w,
                                    by1 / orig_h,
                                    bw / orig_w,
                                    bh / orig_h,
                                ],
                            })
                except Exception as err:
                    print(f"[Vision] Laptop model inference error: {err}")

            self.latest_latency_ms = latency_ms

            # 4. Draw Tactical HUD Annotations
            annotated = frame.copy()
            for d in detections:
                bx, by, bw, bh = d["box"]
                conf = d["confidence"]
                color = (65, 255, 0)  # High-visibility Green

                # Bounding box
                cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), color, 1)

                # Corner brackets
                c_len = min(15, bw // 4, bh // 4)
                cv2.line(annotated, (bx, by), (bx + c_len, by), color, 3)
                cv2.line(annotated, (bx, by), (bx, by + c_len), color, 3)
                cv2.line(annotated, (bx + bw, by), (bx + bw - c_len, by), color, 3)
                cv2.line(annotated, (bx + bw, by), (bx + bw, by + c_len), color, 3)
                cv2.line(annotated, (bx, by + bh), (bx + c_len, by + bh), color, 3)
                cv2.line(annotated, (bx, by + bh), (bx, by + bh - c_len), color, 3)
                cv2.line(annotated, (bx + bw, by + bh), (bx + bw - c_len, by + bh), color, 3)
                cv2.line(annotated, (bx + bw, by + bh), (bx + bw, by + bh - c_len), color, 3)

                # Detection badge tag
                label_text = f"SURVIVOR {conf * 100:.1f}%"
                cv2.rectangle(annotated, (bx, max(0, by - 22)), (bx + 140, max(22, by)), (0, 0, 0), -1)
                cv2.rectangle(annotated, (bx, max(0, by - 22)), (bx + 140, max(22, by)), color, 1)
                cv2.putText(annotated, label_text, (bx + 5, max(15, by - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

            # Top Telemetry Header
            cv2.rectangle(annotated, (0, 0), (orig_w, 24), (10, 10, 12), -1)
            cv2.line(annotated, (0, 24), (orig_w, 24), (39, 39, 42), 1)

            hud_status = (
                f"SRC: FC/JETSON | INFER: LAPTOP (YOLO26s) | "
                f"FPS: {self.fps:.1f} | LATENCY: {latency_ms:.1f}ms | PERSONS: {len(detections)}"
            )
            cv2.putText(annotated, hud_status, (10, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 240, 255), 1, cv2.LINE_AA)

            # 5. Compute Frame Rate
            curr_time = time.time()
            dt = curr_time - prev_time
            prev_time = curr_time
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

            # 6. Encode to JPEG for Dashboard Stream
            ret_encode, jpeg_buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret_encode:
                self.latest_annotated_jpeg = jpeg_buf.tobytes()

            self.latest_detections = detections
            self.detected_person_count = len(detections)
            self.frame_count += 1

    def get_latest_jpeg(self) -> bytes | None:
        """Returns the latest annotated JPEG frame received from FC/Jetson."""
        return self.latest_annotated_jpeg

    def generate_mjpeg_stream(self) -> Generator[bytes, None, None]:
        """Generator yielding MJPEG multipart chunks from FC/Jetson camera."""
        if not self.is_running:
            self.start_camera()

        while self.is_running:
            frame_bytes = self.get_latest_jpeg()
            if frame_bytes is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
                )
                time.sleep(0.033)  # ~30 FPS max
            else:
                # FC camera offline: wait without yielding stale frames
                time.sleep(0.1)

    def get_status(self) -> dict[str, Any]:
        """Comprehensive debug status for the FC Camera and Laptop Model."""
        return {
            "camera_source": "FC / Jetson",
            "fc_host": self.fc_host,
            "stream_url": self.stream_url,
            "camera_status": self.camera_status,
            "receiving_frames": self.receiving_frames,
            "frame_fps": round(self.fps, 1),
            "model_status": "RUNNING ON LAPTOP" if self.model_loaded else "MODEL OFFLINE",
            "model_name": self.model_name,
            "model_fps": round(self.fps, 1),
            "inference_latency_ms": round(self.latest_latency_ms, 1),
            "last_frame_timestamp": self.last_frame_time,
            "detected_persons": self.detected_person_count,
            "detections": self.latest_detections,
        }


# Global Vision Service Singleton
vision_service = VisionService()
