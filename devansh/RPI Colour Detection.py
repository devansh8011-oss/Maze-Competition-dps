#!/usr/bin/env python3
"""
Steel Circuits — Tile Color Detector
=====================================

Threaded, class-based rewrite of the original single-function tile
detector. Behaviour (HSV / LAB / BGR voting, confidence scoring, history
buffer, debouncing, serial protocol, overlays) is IDENTICAL to the
original script — only the structure and threading model changed.

Pipeline:
    CameraManager   (thread) -> always holds the single newest raw frame
    FrameProcessor  (thread) -> blur/ROI/classify/vote/debounce/serial/overlay
    StreamServer    (Flask, its own request thread per client) -> MJPEG out
    SerialManager                  -> thread-safe UART writer
    ColorDetector                  -> stateless classification logic
    OverlayRenderer                -> stateless drawing logic

Design note on "frame queue" vs. "drop old frames":
    The brief asks for both a frame queue *and* always-process-newest /
    drop-old-frames low latency behaviour. Those two goals conflict for a
    live camera feed, so this implementation deliberately uses a
    queue of depth 1 with overwrite (a "mailbox") instead of an
    unbounded/FIFO queue: producers always overwrite the single slot,
    consumers always read the latest value. This gives the lowest
    possible latency, which is the explicit, harder requirement.

Run:
    python3 app.py
"""

import time
import threading
from collections import deque

import cv2
import numpy as np
import serial
from flask import Flask, Response
from picamera2 import Picamera2


# ======================================================================
# CONFIGURATION CONSTANTS (the only "globals" in this file)
# ======================================================================

SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 9600

FRAME_SIZE = (640, 480)          # (width, height)
JPEG_QUALITY = 70                # 65-75 recommended: visual quality vs. CPU/bandwidth

DEBOUNCE_FRAMES = 5
MIN_CONFIDENCE = 50
HISTORY_LEN = 9

STREAM_HOST = "0.0.0.0"
STREAM_PORT = 5000

# Camera tuning. ExposureTime is the dominant FPS ceiling here (see
# "Performance tips" in SETUP.md) — it is kept identical to the original
# script so detection behaviour does not change.
CAMERA_CONTROLS = {
    "AeEnable": False,
    "AwbEnable": False,
    "ExposureTime": 20000,
}

THRESHOLDS = {
    "BLACK": {"v_max": 35, "l_max": 35},
    "WHITE": {"v_min": 180, "s_max": 50, "l_min": 170},
    "BLUE": {"h_min": 90, "h_max": 140, "s_min": 55, "v_min": 12},
}

DRAW_COLORS = {
    "BLACK": (50, 50, 50),
    "WHITE": (255, 255, 255),
    "BLUE": (255, 80, 0),
    "UNKNOWN": (0, 220, 0),
}


# ======================================================================
# CameraManager — owns Picamera2, runs a dedicated capture thread,
# always exposes only the single newest frame.
# ======================================================================

class CameraManager:
    """Dedicated capture thread. Never blocks a consumer and never builds
    up a backlog: the latest frame simply overwrites the previous one."""

    def __init__(self, size=FRAME_SIZE):
        self._picam2 = Picamera2()
        # NOTE: deliberately using create_preview_configuration (not
        # create_video_configuration) with no explicit pixel format.
        # This matches the exact configuration the original, working
        # detection code used. Picamera2/libcamera format naming for
        # "RGB888" vs "BGR888" is notoriously easy to get backwards, and
        # getting it wrong silently breaks every threshold in
        # ColorDetector. buffer_count is a safe, format-independent
        # optimisation that increases the internal capture buffer pool
        # (less stalling under load) without touching pixel layout.
        config = self._picam2.create_preview_configuration(
            main={"size": size}, buffer_count=4
        )
        self._picam2.configure(config)
        self._picam2.start()
        time.sleep(2)
        self._picam2.set_controls(CAMERA_CONTROLS)

        self._lock = threading.Lock()
        self._frame = None
        self._frame_id = 0
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="CameraCapture"
        )

    def start(self):
        self._thread.start()
        return self

    def _capture_loop(self):
        capture = self._picam2.capture_array
        while not self._stop_event.is_set():
            try:
                frame = capture()
            except Exception as exc:  # noqa: BLE001 - keep the thread alive
                print(f"[CameraManager] capture error: {exc}")
                time.sleep(0.05)
                continue
            with self._lock:
                self._frame = frame
                self._frame_id += 1

    def read(self):
        """Returns (frame_id, frame). frame may briefly be None at startup."""
        with self._lock:
            return self._frame_id, self._frame

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        try:
            self._picam2.stop()
        except Exception:  # noqa: BLE001
            pass


# ======================================================================
# ColorDetector — stateless HSV + LAB + BGR voting classifier.
# Logic is byte-for-byte identical to the original script.
# ======================================================================

class ColorDetector:

    @staticmethod
    def classify_hsv(h, s, v):
        t = THRESHOLDS
        if t["BLUE"]["h_min"] < h < t["BLUE"]["h_max"] and s > t["BLUE"]["s_min"] and v > t["BLUE"]["v_min"]:
            return "BLUE"
        if v < t["BLACK"]["v_max"]:
            return "BLACK"
        if s < t["WHITE"]["s_max"] and v > t["WHITE"]["v_min"]:
            return "WHITE"
        return "UNKNOWN"

    @staticmethod
    def classify_lab(l, a, b_ch):
        t = THRESHOLDS
        if b_ch < 118 and 98 < a < 152 and l > 8:
            return "BLUE"
        if l < t["BLACK"]["l_max"]:
            return "BLACK"
        if l > t["WHITE"]["l_min"]:
            return "WHITE"
        return "UNKNOWN"

    @staticmethod
    def classify_bgr(b_med, g_med, r_med):
        total = float(b_med + g_med + r_med) + 1e-6
        r_ratio = r_med / total
        g_ratio = g_med / total
        b_ratio = b_med / total
        brightness = total / 3.0
        if b_ratio > 0.36 and b_med > r_med + 15 and b_med > g_med + 10 and b_med > 20:
            return "BLUE"
        if brightness < 35:
            return "BLACK"
        if brightness > 170 and abs(r_ratio - g_ratio) < 0.08:
            return "WHITE"
        return "UNKNOWN"

    @staticmethod
    def vote(hsv_r, lab_r, bgr_r):
        # HSV counts double for BLUE — hue is the most reliable blue signal
        votes = [hsv_r, hsv_r, lab_r, bgr_r]
        for candidate in ("BLUE", "WHITE", "BLACK"):
            if votes.count(candidate) >= 2:
                return candidate
        return hsv_r

    @staticmethod
    def get_confidence(hsv_roi, lab_roi, color_name):
        h = hsv_roi[:, :, 0]
        s = hsv_roi[:, :, 1]
        v = hsv_roi[:, :, 2]
        L = lab_roi[:, :, 0]
        t = THRESHOLDS
        if color_name == "BLACK":
            mask = (v < t["BLACK"]["v_max"]) & (L < t["BLACK"]["l_max"] + 10)
        elif color_name == "WHITE":
            mask = (s < t["WHITE"]["s_max"] + 10) & (v > t["WHITE"]["v_min"] - 10)
        elif color_name == "BLUE":
            mask = (
                (h > t["BLUE"]["h_min"] - 5)
                & (h < t["BLUE"]["h_max"] + 5)
                & (s > t["BLUE"]["s_min"] - 15)
            )
        else:
            return 0.0
        total = hsv_roi.shape[0] * hsv_roi.shape[1]
        return round((np.sum(mask) / total) * 100, 1)


# ======================================================================
# OverlayRenderer — stateless drawing logic, mutates the frame in place.
# ======================================================================

class OverlayRenderer:

    @staticmethod
    def draw(frame, roi_box, color_name, confidence, draw_color,
             stable_count, last_sent, history,
             hsv_vote, lab_vote, bgr_vote,
             med_h, med_s, med_v, med_l, med_a, med_b):

        h_f, w_f = frame.shape[:2]
        cx1, cy1, cx2, cy2 = roi_box

        cv2.rectangle(frame, (0, 0), (w_f - 1, h_f - 1), draw_color, 25)
        cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), draw_color, 2)

        label = f"{color_name}  {confidence}%"
        for offset, col, thick in [((2, 2), (0, 0, 0), 5), ((0, 0), draw_color, 3)]:
            cv2.putText(frame, label, (20 + offset[0], 56 + offset[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.4, col, thick)

        vote_text = f"HSV:{hsv_vote[:3]}  LAB:{lab_vote[:3]}  BGR:{bgr_vote[:3]}"
        for offset, col, thick in [((1, 1), (0, 0, 0), 3), ((0, 0), (180, 180, 180), 1)]:
            cv2.putText(frame, vote_text, (20 + offset[0], 95 + offset[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, thick)

        raw_text = f"H:{med_h} S:{med_s} V:{med_v}  L:{med_l} a:{med_a} b:{med_b}"
        for offset, col, thick in [((1, 1), (0, 0, 0), 3), ((0, 0), (160, 160, 160), 1)]:
            cv2.putText(frame, raw_text, (20 + offset[0], 120 + offset[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, thick)

        bx, by, bw, bh = 20, 132, 200, 10
        fill = int((min(stable_count, DEBOUNCE_FRAMES) / DEBOUNCE_FRAMES) * bw)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (30, 30, 30), -1)
        cv2.rectangle(frame, (bx, by), (bx + fill, by + bh), draw_color, -1)

        dot_r = 7
        for i, past_color in enumerate(history):
            dot_col = DRAW_COLORS.get(past_color, (0, 220, 0))
            cx_dot = 20 + i * (dot_r * 2 + 4)
            cy_dot = h_f - 30
            cv2.circle(frame, (cx_dot, cy_dot), dot_r, dot_col, -1)
            cv2.circle(frame, (cx_dot, cy_dot), dot_r, (80, 80, 80), 1)

        sent_col = (0, 200, 0) if last_sent == color_name else (80, 80, 80)
        sent_txt = f"SENT: {last_sent if last_sent else '---'}"
        cv2.putText(frame, sent_txt, (w_f - 220, h_f - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, sent_col, 2)

        return frame


# ======================================================================
# SerialManager — thread-safe UART writer. Protocol is unchanged:
# "<COLOR>\n" at 9600 baud on /dev/serial0.
# ======================================================================

class SerialManager:

    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        self._lock = threading.Lock()
        try:
            self._ser = serial.Serial(port, baud)
        except serial.SerialException as exc:
            print(f"[SerialManager] WARNING: could not open {port}: {exc}")
            self._ser = None

    def send(self, color_name):
        if self._ser is None:
            return
        with self._lock:
            try:
                self._ser.write((color_name + "\n").encode())
            except serial.SerialException as exc:
                print(f"[SerialManager] write failed: {exc}")

    def close(self):
        if self._ser is not None:
            with self._lock:
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass


# ======================================================================
# StreamServer — Flask app + MJPEG generator. Holds only the single
# newest processed frame, so the browser never sees a backlog either.
# ======================================================================

class StreamServer:

    def __init__(self, host=STREAM_HOST, port=STREAM_PORT, jpeg_quality=JPEG_QUALITY):
        self.host = host
        self.port = port
        self._encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
        self._lock = threading.Lock()
        self._frame = None
        self.app = Flask(__name__)
        self._register_routes()

    def update_frame(self, frame):
        with self._lock:
            self._frame = frame

    def _get_frame(self):
        with self._lock:
            return self._frame

    def _generate(self):
        boundary_header = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
        while True:
            frame = self._get_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            ok, buf = cv2.imencode(".jpg", frame, self._encode_params)
            if not ok:
                continue
            yield boundary_header + buf.tobytes() + b"\r\n"

    def _register_routes(self):
        @self.app.route("/")
        def video_feed():
            return Response(
                self._generate(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

    def run(self):
        # threaded=True lets each MJPEG client get its own request thread,
        # so the long-lived streaming connection never blocks new clients.
        self.app.run(host=self.host, port=self.port, threaded=True,
                     debug=False, use_reloader=False)


# ======================================================================
# FrameProcessor — the per-frame pipeline, on its own thread.
# blur -> ROI -> HSV/LAB/BGR -> vote -> confidence -> history/debounce
# -> serial -> overlay -> hand off to StreamServer.
# ======================================================================

class FrameProcessor:

    def __init__(self, camera, serial_manager, stream_server, color_detector=ColorDetector):
        self._camera = camera
        self._serial = serial_manager
        self._stream = stream_server
        self._detector = color_detector

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="FrameProcessor")

        self._last_sent = ""
        self._stable_color = ""
        self._stable_count = 0
        self._frame_num = 0
        self._history = deque(maxlen=HISTORY_LEN)
        self._last_frame_id = -1

        # Reusable ROI-sized buffers for the two colour-space conversions
        # that run every single frame. These never escape this thread
        # (fully consumed before the next frame arrives), so reusing them
        # is safe and removes two allocations per frame. The full-frame
        # BGR/blur buffers are deliberately NOT reused: that frame is
        # handed off to the streaming thread, and reusing it would race
        # with JPEG-encoding the previous frame.
        self._hsv_buf = None
        self._lab_buf = None

    def start(self):
        self._thread.start()
        return self

    @staticmethod
    def _roi_box(h, w):
        return w // 4, h // 4, 3 * w // 4, 3 * h // 4

    def _ensure_roi_buffers(self, roi):
        if self._hsv_buf is None or self._hsv_buf.shape != roi.shape:
            self._hsv_buf = np.empty_like(roi)
            self._lab_buf = np.empty_like(roi)

    def _loop(self):
        while not self._stop_event.is_set():
            frame_id, raw = self._camera.read()
            if raw is None or frame_id == self._last_frame_id:
                time.sleep(0.001)
                continue
            self._last_frame_id = frame_id

            try:
                self._process_one(raw)
            except Exception as exc:  # noqa: BLE001 - keep the thread alive
                print(f"[FrameProcessor] error: {exc}")

    def _process_one(self, raw):
        frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        frame = cv2.medianBlur(frame, 5)

        h_f, w_f = frame.shape[:2]
        cx1, cy1, cx2, cy2 = self._roi_box(h_f, w_f)
        roi = frame[cy1:cy2, cx1:cx2]

        self._ensure_roi_buffers(roi)
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV, self._hsv_buf)
        lab_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB, self._lab_buf)

        med_h = int(np.median(hsv_roi[:, :, 0]))
        med_s = int(np.median(hsv_roi[:, :, 1]))
        med_v = int(np.median(hsv_roi[:, :, 2]))

        med_l = int(np.median(lab_roi[:, :, 0]))
        med_a = int(np.median(lab_roi[:, :, 1]))
        med_b = int(np.median(lab_roi[:, :, 2]))

        med_bgr_b = int(np.median(roi[:, :, 0]))
        med_bgr_g = int(np.median(roi[:, :, 1]))
        med_bgr_r = int(np.median(roi[:, :, 2]))

        hsv_vote = self._detector.classify_hsv(med_h, med_s, med_v)
        lab_vote = self._detector.classify_lab(med_l, med_a, med_b)
        bgr_vote = self._detector.classify_bgr(med_bgr_b, med_bgr_g, med_bgr_r)

        frame_color = self._detector.vote(hsv_vote, lab_vote, bgr_vote)

        self._history.append(frame_color)
        color_counts = {}
        for c in self._history:
            color_counts[c] = color_counts.get(c, 0) + 1
        color_name = max(color_counts, key=color_counts.get)

        confidence = self._detector.get_confidence(hsv_roi, lab_roi, color_name)
        draw_color = DRAW_COLORS.get(color_name, (0, 220, 0))

        if color_name == self._stable_color:
            self._stable_count += 1
        else:
            self._stable_color = color_name
            self._stable_count = 1

        ready = (
            self._stable_count >= DEBOUNCE_FRAMES
            and color_name != self._last_sent
            and confidence >= MIN_CONFIDENCE
            and color_name != "UNKNOWN"
        )

        if ready:
            self._serial.send(color_name)
            print(
                f"[SEND] {color_name:7s} | "
                f"HSV:{med_h},{med_s},{med_v} | "
                f"LAB:{med_l},{med_a},{med_b} | "
                f"BGR:{med_bgr_r},{med_bgr_g},{med_bgr_b} | "
                f"votes HSV={hsv_vote} LAB={lab_vote} BGR={bgr_vote} | "
                f"conf:{confidence}%"
            )
            self._last_sent = color_name

        self._frame_num += 1
        if self._frame_num % 60 == 0:
            print(
                f"[CAL]  {color_name:7s} | "
                f"HSV:{med_h:3d},{med_s:3d},{med_v:3d} | "
                f"LAB:{med_l:3d},{med_a:3d},{med_b:3d} | "
                f"votes HSV={hsv_vote} LAB={lab_vote} BGR={bgr_vote} | "
                f"conf:{confidence}% stable:{self._stable_count}"
            )

        OverlayRenderer.draw(
            frame, (cx1, cy1, cx2, cy2), color_name, confidence, draw_color,
            self._stable_count, self._last_sent, self._history,
            hsv_vote, lab_vote, bgr_vote,
            med_h, med_s, med_v, med_l, med_a, med_b,
        )

        self._stream.update_frame(frame)

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)


# ======================================================================
# Entry point
# ======================================================================

def main():
    print("=" * 50)
    print("TILE DETECTOR — BLACK / WHITE / BLUE")
    print("Analysing centre 50% of frame")
    print("=" * 50)

    camera = CameraManager(FRAME_SIZE).start()
    serial_manager = SerialManager(SERIAL_PORT, BAUD_RATE)
    stream_server = StreamServer(STREAM_HOST, STREAM_PORT, JPEG_QUALITY)
    processor = FrameProcessor(camera, serial_manager, stream_server).start()

    try:
        stream_server.run()
    except KeyboardInterrupt:
        pass
    finally:
        processor.stop()
        camera.stop()
        serial_manager.close()


if __name__ == "__main__":
    main()
