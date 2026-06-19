from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import numpy as np
import time
import serial
from collections import deque

ser = serial.Serial('/dev/serial0', 9600)
app = Flask(__name__)

picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(main={"size": (640, 480)})
)
picam2.start()
time.sleep(2)
picam2.set_controls({
    "AeEnable":     False,
    "AwbEnable":    False,
    "ExposureTime": 20000
})

THRESHOLDS = {
    "BLACK": {"v_max": 35,  "l_max": 35},
    "WHITE": {"v_min": 180, "s_max": 50, "l_min": 170},
    "BLUE":  {"h_min": 90,  "h_max": 140, "s_min": 55, "v_min": 12},
}

DEBOUNCE_FRAMES = 5
MIN_CONFIDENCE  = 50
HISTORY_LEN     = 9

DRAW_COLORS = {
    "BLACK":   (50,  50,  50),
    "WHITE":   (255, 255, 255),
    "BLUE":    (255, 80,  0),
    "UNKNOWN": (0,   220, 0),
}

def get_roi(frame):
    h, w = frame.shape[:2]
    return frame[h // 4: 3 * h // 4, w // 4: 3 * w // 4]

def classify_hsv(h, s, v):
    t = THRESHOLDS
    if t["BLUE"]["h_min"] < h < t["BLUE"]["h_max"] and s > t["BLUE"]["s_min"] and v > t["BLUE"]["v_min"]:
        return "BLUE"
    if v < t["BLACK"]["v_max"]:
        return "BLACK"
    if s < t["WHITE"]["s_max"] and v > t["WHITE"]["v_min"]:
        return "WHITE"
    return "UNKNOWN"

def classify_lab(l, a, b_ch):
    t = THRESHOLDS
    if b_ch < 118 and 98 < a < 152 and l > 8:
        return "BLUE"
    if l < t["BLACK"]["l_max"]:
        return "BLACK"
    if l > t["WHITE"]["l_min"]:
        return "WHITE"
    return "UNKNOWN"

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

def vote(hsv_r, lab_r, bgr_r):
    # HSV counts double for BLUE — hue is the most reliable blue signal
    votes = [hsv_r, hsv_r, lab_r, bgr_r]
    for candidate in ["BLUE", "WHITE", "BLACK"]:
        if votes.count(candidate) >= 2:
            return candidate
    return hsv_r

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

def generate_frames():
    last_sent    = ""
    stable_color = ""
    stable_count = 0
    frame_num    = 0
    history      = deque(maxlen=HISTORY_LEN)

    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.medianBlur(frame, 5)

        roi = get_roi(frame)

        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lab_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

        med_h = int(np.median(hsv_roi[:, :, 0]))
        med_s = int(np.median(hsv_roi[:, :, 1]))
        med_v = int(np.median(hsv_roi[:, :, 2]))

        med_l = int(np.median(lab_roi[:, :, 0]))
        med_a = int(np.median(lab_roi[:, :, 1]))
        med_b = int(np.median(lab_roi[:, :, 2]))

        med_bgr_b = int(np.median(roi[:, :, 0]))
        med_bgr_g = int(np.median(roi[:, :, 1]))
        med_bgr_r = int(np.median(roi[:, :, 2]))

        hsv_vote = classify_hsv(med_h, med_s, med_v)
        lab_vote = classify_lab(med_l, med_a, med_b)
        bgr_vote = classify_bgr(med_bgr_b, med_bgr_g, med_bgr_r)

        frame_color = vote(hsv_vote, lab_vote, bgr_vote)

        history.append(frame_color)
        color_counts = {}
        for c in history:
            color_counts[c] = color_counts.get(c, 0) + 1
        color_name = max(color_counts, key=color_counts.get)

        confidence = get_confidence(hsv_roi, lab_roi, color_name)
        draw_color = DRAW_COLORS.get(color_name, (0, 220, 0))

        if color_name == stable_color:
            stable_count += 1
        else:
            stable_color = color_name
            stable_count = 1

        ready = (
            stable_count >= DEBOUNCE_FRAMES
            and color_name != last_sent
            and confidence >= MIN_CONFIDENCE
            and color_name != "UNKNOWN"
        )

        if ready:
            ser.write((color_name + "\n").encode())
            print(
                f"[SEND] {color_name:7s} | "
                f"HSV:{med_h},{med_s},{med_v} | "
                f"LAB:{med_l},{med_a},{med_b} | "
                f"BGR:{med_bgr_r},{med_bgr_g},{med_bgr_b} | "
                f"votes HSV={hsv_vote} LAB={lab_vote} BGR={bgr_vote} | "
                f"conf:{confidence}%"
            )
            last_sent = color_name

        frame_num += 1
        if frame_num % 60 == 0:
            print(
                f"[CAL]  {color_name:7s} | "
                f"HSV:{med_h:3d},{med_s:3d},{med_v:3d} | "
                f"LAB:{med_l:3d},{med_a:3d},{med_b:3d} | "
                f"votes HSV={hsv_vote} LAB={lab_vote} BGR={bgr_vote} | "
                f"conf:{confidence}% stable:{stable_count}"
            )

        h_f, w_f = frame.shape[:2]
        cx1, cy1 = w_f // 4, h_f // 4
        cx2, cy2 = 3 * w_f // 4, 3 * h_f // 4

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
            cx_dot  = 20 + i * (dot_r * 2 + 4)
            cy_dot  = h_f - 30
            cv2.circle(frame, (cx_dot, cy_dot), dot_r, dot_col, -1)
            cv2.circle(frame, (cx_dot, cy_dot), dot_r, (80, 80, 80), 1)

        sent_col = (0, 200, 0) if last_sent == color_name else (80, 80, 80)
        sent_txt = f"SENT: {last_sent if last_sent else '---'}"
        cv2.putText(frame, sent_txt, (w_f - 220, h_f - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, sent_col, 2)

        _, buffer = cv2.imencode('.jpg', frame)
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            buffer.tobytes() + b'\r\n'
        )

@app.route('/')
def video_feed():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == '__main__':
    print("=" * 50)
    print("TILE DETECTOR — BLACK / WHITE / BLUE")
    print("Analysing centre 50% of frame")
    print("=" * 50)
    app.run(host='0.0.0.0', port=5000)
