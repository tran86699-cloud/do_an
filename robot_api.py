#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'

from flask import Flask, jsonify, request, Response
from flask_cors import CORS

import cv2
import numpy as np
import time
import math
import threading
import json
from collections import deque
from threading import Lock

# ============================================================================
# MODE 3 - PC COMMUNICATION (AI-ASSISTED)
# ============================================================================
try:
    from config.pc_config import PC_IP, PC_PORT, PC_UPDATE_INTERVAL, PC_POLL_INTERVAL
    from config.pc_communication import PCCommunication
    PC_AVAILABLE = True
    print("[MODE3] PC communication module loaded")
except ImportError as e:
    PC_AVAILABLE = False
    print(f"[MODE3] PC not available: {e}")
    print("[MODE3] MODE 3 disabled - install pc_communication.py to enable")

print("[OK] Loading system...")

# ============================================================================
# THREAD-SAFE STATE MANAGEMENT (ENHANCED)
# ============================================================================
state_lock = Lock()
robot_state = {
    'distance': 0,
    'pan_angle': 90,
    'tilt_angle': 45,
    'mode': 'IDLE',
    'line_sensors': [0, 0, 0, 0],
    'speed': 0,
    'last_command': 'none',
    'timestamp': time.time()
}

def update_robot_state(key, value):
    """Update robot state thread-safe"""
    global robot_state
    with state_lock:
        robot_state[key] = value
        robot_state['timestamp'] = time.time()

def get_robot_state():
    global robot_state
    with state_lock:
        return robot_state.copy()

# ============================================================================
# MODE 3 - GLOBAL STATE
# ============================================================================
current_mode = 'manual'  # 'manual', 'autonomous', 'ai'
pc_comm = None
pc_update_thread = None
pc_poll_thread = None
pc_running = False

# ============================================================================
# CONFIGURATION IMPORTS
# ============================================================================
try:
    from config.vision_config import (
        VISION_CAMERA_INDEX, VISION_FRAME_WIDTH, VISION_FRAME_HEIGHT, VISION_FPS,
        VISION_EXPOSURE, VISION_BRIGHTNESS, VISION_CONTRAST, VISION_SATURATION,
        VISION_FACE_SCALE_FACTOR, VISION_FACE_MIN_NEIGHBORS,
        VISION_FACE_MIN_SIZE, VISION_FACE_MAX_SIZE, VISION_FACE_TRACKING_ALPHA,
        VISION_FACE_TRACKING_OUTLIER_THRESHOLD, VISION_FACE_TRACKING_STREAK,
        VISION_SERVO_PAN_MIN, VISION_SERVO_PAN_MAX, VISION_SERVO_TILT_MIN,
        VISION_SERVO_TILT_MAX, VISION_SERVO_PAN_CENTER, VISION_SERVO_TILT_CENTER,
        VISION_SERVO_SEARCH_SPEED, VISION_SERVO_SEARCH_RANGE,
        VISION_ULTRASONIC_DANGER_DISTANCE, VISION_FACE_MISSING_THRESHOLD,
        VISION_FACE_CONFIDENCE_STREAK_THRESHOLD, VISION_PATROL_SPEED,
        VISION_SERVO_PAN_ADJUST_STEP, VISION_SERVO_TILT_ADJUST_STEP,
        VISION_SERVO_DEADZONE, VISION_SEARCH_PAN_MIN, VISION_SEARCH_PAN_MAX,
        VISION_SEARCH_TILT_MIN, VISION_SEARCH_TILT_MAX,
        VISION_FACE_CENTER_SMOOTHING_ALPHA, VISION_OBSTACLE_AVOID_SPEED,
        VISION_OBSTACLE_AVOID_DURATION, VISION_OBSTACLE_AVOID_ROTATE_SPEED,
        VISION_OBSTACLE_AVOID_ROTATE_DURATION,
    )
    print("[OK] Config loaded")
except ImportError as e:
    print(f"[WARN] Config error: {e}")
    print("[WARN] Using fallback values")
    VISION_CAMERA_INDEX = 0
    VISION_FRAME_WIDTH = 640
    VISION_FRAME_HEIGHT = 480
    VISION_FPS = 30
    VISION_EXPOSURE = -5
    VISION_BRIGHTNESS = 0
    VISION_CONTRAST = 0
    VISION_SATURATION = 0
    VISION_FACE_SCALE_FACTOR = 1.05
    VISION_FACE_MIN_NEIGHBORS = 8
    VISION_FACE_MIN_SIZE = (50, 50)
    VISION_FACE_MAX_SIZE = (400, 400)
    VISION_FACE_TRACKING_ALPHA = 0.4
    VISION_FACE_TRACKING_OUTLIER_THRESHOLD = 120
    VISION_FACE_TRACKING_STREAK = 5
    VISION_SERVO_PAN_MIN = 0
    VISION_SERVO_PAN_MAX = 180
    VISION_SERVO_TILT_MIN = 0
    VISION_SERVO_TILT_MAX = 180
    VISION_SERVO_PAN_CENTER = 90
    VISION_SERVO_TILT_CENTER = 45
    VISION_SERVO_SEARCH_SPEED = 2
    VISION_SERVO_SEARCH_RANGE = 45
    VISION_ULTRASONIC_DANGER_DISTANCE = 20
    VISION_FACE_MISSING_THRESHOLD = 30
    VISION_FACE_CONFIDENCE_STREAK_THRESHOLD = 5
    VISION_PATROL_SPEED = 80
    VISION_SERVO_PAN_ADJUST_STEP = 50
    VISION_SERVO_TILT_ADJUST_STEP = 50
    VISION_SERVO_DEADZONE = 120
    VISION_SEARCH_PAN_MIN = 20
    VISION_SEARCH_PAN_MAX = 160
    VISION_SEARCH_TILT_MIN = 10
    VISION_SEARCH_TILT_MAX = 90
    VISION_FACE_CENTER_SMOOTHING_ALPHA = 0.4
    VISION_OBSTACLE_AVOID_SPEED = 90
    VISION_OBSTACLE_AVOID_DURATION = 0.8
    VISION_OBSTACLE_AVOID_ROTATE_SPEED = 90
    VISION_OBSTACLE_AVOID_ROTATE_DURATION = 1.0

# ============================================================================
# HARDWARE
# ============================================================================

class RaspbotHardware:
    def __init__(self):
        try:
            from Raspbot_Lib import Raspbot
            self.bot = Raspbot()
            self.available = True
            print("[OK] Raspbot_Lib initialized")
        except ImportError:
            print("[WARN] Simulation mode")
            self.bot = None
            self.available = False
   
    def read_ultrasonic(self):
        if not self.available or not self.bot:
            return -1
        try:
            self.bot.Ctrl_Ulatist_Switch(1)
            time.sleep(0.05)
            dis_H_arr = self.bot.read_data_array(0x1b, 1)
            dis_L_arr = self.bot.read_data_array(0x1a, 1)
            if not dis_H_arr or not dis_L_arr:
                return -1
            dis_H = dis_H_arr[0]
            dis_L = dis_L_arr[0]
            distance_mm = (dis_H << 8) | dis_L
            distance_cm = distance_mm / 10
            self.bot.Ctrl_Ulatist_Switch(0)
            return distance_cm if distance_cm > 0 else -1
        except:
            return -1
   
    def read_line_sensors(self):
        """Read 4 line tracking sensors - FIXED ORDER"""
        if not self.available or not self.bot:
            return [0, 0, 0, 0]
        try:
            arr = self.bot.read_data_array(0x0a, 1)
            if not arr:
                return [0, 0, 0, 0]
            data = arr[0]
            # Extract bits: [SW2, SW1, SW3, SW4] from hardware
            sw2 = 1 if (data & 0b00000100) else 0  # Hardware L1
            sw1 = 1 if (data & 0b00000010) else 0  # Hardware L2
            sw3 = 1 if (data & 0b00001000) else 0  # Hardware R1
            sw4 = 1 if (data & 0b00000001) else 0  # Hardware R2
           
            # FIXED: Return in correct order [L1, L2, R1, R2]
            # Swap R1 and R2 because hardware wiring is reversed
            return [sw2, sw3, sw1, sw4]
        except:
            return [0, 0, 0, 0]
   
    def set_servo(self, servo_id, angle):
        if not self.available or not self.bot:
            return
        try:
            angle = max(0, min(180, angle))
            self.bot.Ctrl_Servo(servo_id, angle)
        except:
            pass
   
    def set_motors(self, l1, l2, r1, r2):
        if not self.available or not self.bot:
            return
        try:
            self.bot.Ctrl_Muto(0, l1)
            self.bot.Ctrl_Muto(1, l2)
            self.bot.Ctrl_Muto(2, r1)
            self.bot.Ctrl_Muto(3, r2)
        except:
            pass

hardware = RaspbotHardware()

# ============================================================================
# SERVO CONTROLLER
# ============================================================================

class ServoController:
    def __init__(self):
        self.pan = VISION_SERVO_PAN_CENTER
        self.tilt = VISION_SERVO_TILT_CENTER
        self.hardware = hardware
   
    def set_pan(self, angle):
        self.pan = max(VISION_SERVO_PAN_MIN, min(VISION_SERVO_PAN_MAX, angle))
        # FIXED: Invert pan because servo is mounted backwards
        inverted_angle = 180 - self.pan
        self.hardware.set_servo(1, inverted_angle)
        update_robot_state('pan_angle', self.pan)
       
    def set_tilt(self, angle):
        self.tilt = max(VISION_SERVO_TILT_MIN, min(VISION_SERVO_TILT_MAX, angle))
        self.hardware.set_servo(2, self.tilt)
        update_robot_state('tilt_angle', self.tilt)
   
    def center(self):
        self.set_pan(VISION_SERVO_PAN_CENTER)
        self.set_tilt(VISION_SERVO_TILT_CENTER)
   
    def print_status(self):
        return f"Pan: {self.pan} deg, Tilt: {self.tilt} deg"

servo_controller = ServoController()
servo_controller.center()
print("[OK] ServoController ready")

# ============================================================================
# ULTRASONIC MONITOR
# ============================================================================

class UltrasonicMonitor:
    def __init__(self, danger_distance=VISION_ULTRASONIC_DANGER_DISTANCE):
        self.danger_distance = danger_distance
        self.current_distance = -1
        self.distance_history = deque(maxlen=10)
        self.hardware = hardware
   
    def update(self):
        distance = self.hardware.read_ultrasonic()
        if distance > 0:
            self.current_distance = distance
            self.distance_history.append(distance)
            update_robot_state('distance', round(distance, 2))
        return self.current_distance
   
    def is_obstacle(self):
        return 0 < self.current_distance < self.danger_distance
   
    def get_status(self):
        if self.current_distance < 0:
            return "[WARN] N/A"
        elif self.is_obstacle():
            return f"[DANGER] {self.current_distance:.1f}cm DANGER"
        elif self.current_distance < 50:
            return f"[WARN] {self.current_distance:.1f}cm CLOSE"
        else:
            return f"[OK] {self.current_distance:.1f}cm SAFE"

ultrasonic = UltrasonicMonitor(danger_distance=VISION_ULTRASONIC_DANGER_DISTANCE)
print("[OK] UltrasonicMonitor ready")

# ============================================================================
# ALWAYS-ON ULTRASONIC THREAD
# ============================================================================

def ultrasonic_update_loop():
    """Background thread - always read ultrasonic"""
    while True:
        try:
            ultrasonic.update()
            time.sleep(0.1)
        except:
            time.sleep(0.5)

ult_thread = threading.Thread(target=ultrasonic_update_loop, daemon=True)
ult_thread.start()
print("[OK] Ultrasonic thread started")

# ============================================================================
# FACE TRACKER
# ============================================================================

class FaceTracker:
    def __init__(self):
        self.cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.bbox_history = deque(maxlen=5)
        self.consecutive_count = 0
        self.min_frames_for_detection = 2
   
    def detect(self, frame):
        """Face detection with larger bounding box"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.cascade.detectMultiScale(
            gray,
            scaleFactor=VISION_FACE_SCALE_FACTOR,
            minNeighbors=VISION_FACE_MIN_NEIGHBORS,
            minSize=VISION_FACE_MIN_SIZE,
            maxSize=VISION_FACE_MAX_SIZE,
            flags=cv2.CASCADE_SCALE_IMAGE
        )
       
        if len(faces) == 0:
            self.consecutive_count = 0
            return None
       
        largest = max(faces, key=lambda f: f[2] * f[3])
        x, y, w, h = largest
       
        aspect_ratio = w / h
        if aspect_ratio < 0.6 or aspect_ratio > 1.6:
            self.consecutive_count = 0
            return None
       
        expand_x = int(w * 0.60)
        expand_y = int(h * 0.60)
       
        x = max(0, x - expand_x)
        y = max(0, y - expand_y)
        w_new = w + expand_x * 2
        h_new = h + expand_y * 2
       
        if x + w_new > frame.shape[1]:
            w_new = frame.shape[1] - x
        if y + h_new > frame.shape[0]:
            h_new = frame.shape[0] - y
       
        detection = {
            'bbox': (x, y, w_new, h_new),
            'center': (x + w_new//2, y + h_new//2),
            'area': w_new * h_new
        }
       
        self.bbox_history.append(detection)
       
        if len(self.bbox_history) >= self.min_frames_for_detection:
            self.consecutive_count += 1
           
            bboxes = [d['bbox'] for d in self.bbox_history]
            xs = [b[0] for b in bboxes]
            ys = [b[1] for b in bboxes]
            ws = [b[2] for b in bboxes]
            hs = [b[3] for b in bboxes]
           
            smooth_x = int(np.mean(xs))
            smooth_y = int(np.mean(ys))
            smooth_w = int(np.mean(ws))
            smooth_h = int(np.mean(hs))
           
            smoothed = {
                'bbox': (smooth_x, smooth_y, smooth_w, smooth_h),
                'center': (smooth_x + smooth_w//2, smooth_y + smooth_h//2),
                'area': smooth_w * smooth_h,
                'confidence': min(self.consecutive_count, 5)
            }
            return smoothed
        else:
            self.consecutive_count += 1
            return None
   
    def draw(self, frame, detection):
        if not detection:
            return frame
        x, y, w, h = detection['bbox']
        confidence = detection.get('confidence', 0)
        color = (0, 255, 0) if confidence >= 4 else (0, 255, 255) if confidence >= 2 else (0, 165, 255)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 3)
        label_text = f"FACE (L{confidence})"
        label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
        cv2.rectangle(frame, (x, y - 30), (x + label_size[0] + 10, y), color, -1)
        cv2.putText(frame, label_text, (x + 5, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
        cx, cy = detection['center']
        cv2.circle(frame, (cx, cy), 8, color, -1)
        cv2.circle(frame, (cx, cy), 10, color, 2)
        return frame

face_tracker = FaceTracker()
print("[OK] FaceTracker ready")

# ============================================================================
# VISION SYSTEM
# ============================================================================

class VisionSystem:
    def __init__(self, camera_index=VISION_CAMERA_INDEX):
        self.camera = cv2.VideoCapture(camera_index)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, VISION_FRAME_WIDTH)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, VISION_FRAME_HEIGHT)
        self.camera.set(cv2.CAP_PROP_FPS, VISION_FPS)
       
        if VISION_EXPOSURE != 0:
            self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            self.camera.set(cv2.CAP_PROP_EXPOSURE, VISION_EXPOSURE)
        if VISION_BRIGHTNESS != 0:
            self.camera.set(cv2.CAP_PROP_BRIGHTNESS, VISION_BRIGHTNESS)
        if VISION_CONTRAST != 0:
            self.camera.set(cv2.CAP_PROP_CONTRAST, VISION_CONTRAST)
        if VISION_SATURATION != 0:
            self.camera.set(cv2.CAP_PROP_SATURATION, VISION_SATURATION)
       
        self.face_tracker = face_tracker
        self.current_frame = None
        self.detections = {}
        self.detect_mode = 'none'
        self.servo = servo_controller
        self.ultrasonic = ultrasonic
   
    def capture(self):
        ret, frame = self.camera.read()
        if ret:
            self.current_frame = frame
            return frame
        return None
   
    def process_detections(self):
        if self.current_frame is None:
            self.detections = {}
            return
        if self.detect_mode == 'face':
            detection = self.face_tracker.detect(self.current_frame)
            self.detections['face'] = detection
        else:
            self.detections = {}
   
    def draw_all(self, frame):
        if frame is None:
            return frame
        frame = self.face_tracker.draw(frame, self.detections.get('face'))
        return frame
   
    def release(self):
        self.camera.release()

vision = VisionSystem()
print("[OK] VisionSystem ready")

# ============================================================================
# MOTOR CONTROLLER
# ============================================================================

class RobotController:
    def __init__(self):
        self.hardware = hardware
        self.moving = False
        self.last_command_time = 0
        self.command_debounce = 0.05
   
    def _send_motors(self, l1, l2, r1, r2):
        now = time.time()
        if now - self.last_command_time < self.command_debounce:
            return
        self.last_command_time = now
        self._set_motors(l1, l2, r1, r2)
   
    def set_deflection(self, speed, deflection):
        if speed > 255:
            speed = 255
        if speed < 0:
            speed = 0
       
        rad2deg = math.pi / 180
        vx = speed * math.cos(deflection * rad2deg)
        vy = speed * math.sin(deflection * rad2deg)
       
        l1 = int(vy + vx)
        l2 = int(vy - vx)
        r1 = int(vy - vx)
        r2 = int(vy + vx)
       
        return l1, l2, r1, r2
   
    def move_forward(self, speed=150):
        speed = max(0, min(255, speed))
        l1, l2, r1, r2 = self.set_deflection(speed, 90)
        self._send_motors(l1, l2, r1, r2)
        self.moving = True
        update_robot_state('speed', speed)
        print(f"[MOTOR] FORWARD: {speed}")
   
    def move_backward(self, speed=150):
        speed = max(0, min(255, speed))
        l1, l2, r1, r2 = self.set_deflection(speed, 270)
        self._send_motors(l1, l2, r1, r2)
        self.moving = True
        update_robot_state('speed', speed)
        print(f"[MOTOR] BACKWARD: {speed}")
   
    def move_left(self, speed=150):
        speed = max(0, min(255, speed))
        l1, l2, r1, r2 = self.set_deflection(speed, 180)
        self._send_motors(l1, l2, r1, r2)
        self.moving = True
        update_robot_state('speed', speed)
        print(f"[MOTOR] LEFT: {speed}")
   
    def move_right(self, speed=150):
        speed = max(0, min(255, speed))
        l1, l2, r1, r2 = self.set_deflection(speed, 0)
        self._send_motors(l1, l2, r1, r2)
        self.moving = True
        update_robot_state('speed', speed)
        print(f"[MOTOR] RIGHT: {speed}")
   
    def rotate_left(self, speed=150):
        speed = max(0, min(255, speed))
        l1, l2, r1, r2 = self.set_deflection(speed, 180)
        self._send_motors(l1, -l2, r1, abs(r2))
        self.moving = True
        update_robot_state('speed', speed)
        print(f"[MOTOR] ROTATE_LEFT: {speed}")
   
    def rotate_right(self, speed=150):
        speed = max(0, min(255, speed))
        l1, l2, r1, r2 = self.set_deflection(speed, 0)
        self._send_motors(l1, abs(l2), r1, -r2)
        self.moving = True
        update_robot_state('speed', speed)
        print(f"[MOTOR] ROTATE_RIGHT: {speed}")
   
    def turn_left(self, speed=150):
        """Turn left (same as rotate_left)"""
        self.rotate_left(speed)
    
    def turn_right(self, speed=150):
        """Turn right (same as rotate_right)"""
        self.rotate_right(speed)
   
    def go_angle(self, angle, speed=150):
        speed = max(0, min(255, speed))
        l1, l2, r1, r2 = self.set_deflection(speed, angle)
        self._send_motors(l1, l2, r1, r2)
        self.moving = True
        update_robot_state('speed', speed)
        print(f"[MOTOR] ANGLE: {angle} @ {speed}")
   
    def stop(self):
        self._set_motors(0, 0, 0, 0)
        self.moving = False
        update_robot_state('speed', 0)
        print("[MOTOR] STOP")
   
    def _set_motors(self, l1, l2, r1, r2):
        self.hardware.set_motors(int(l1), int(l2), int(r1), int(r2))

robot_controller = RobotController()
print("[OK] RobotController ready")

# ============================================================================
# AUTONOMOUS CONTROLLER
# ============================================================================

class AutonomousController:
    def __init__(self, vision, motor, servo, ultrasonic):
        self.vision = vision
        self.motor = motor
        self.servo = servo
        self.ultrasonic = ultrasonic
       
        self.state = 'PATROL'
        self.is_running = False
        self.frame_count = 0
       
        self.face_missing_count = 0
        self.face_confidence_streak = 0
        self.last_valid_center = (VISION_FRAME_WIDTH // 2, VISION_FRAME_HEIGHT // 2)
        self.smooth_cx = VISION_FRAME_WIDTH // 2
        self.smooth_cy = VISION_FRAME_HEIGHT // 2
        self.alpha = VISION_FACE_CENTER_SMOOTHING_ALPHA
       
        self.search_pan = VISION_SERVO_PAN_CENTER
        self.search_direction = 1
        self.search_speed = VISION_SERVO_SEARCH_SPEED
       
        self.search_tilt = VISION_SERVO_TILT_CENTER
        self.search_tilt_direction = 1
        self.search_tilt_speed = 1
       
        self.face_missing_threshold = VISION_FACE_MISSING_THRESHOLD
       
        print("[OK] AutonomousController ready")
   
    def start(self):
        self.is_running = True
        self.servo.center()
        self.vision.detect_mode = 'face'
        self.state = 'PATROL'
        self.frame_count = 0
        self.face_missing_count = 0
        self.face_confidence_streak = 0
        self.search_pan = VISION_SERVO_PAN_CENTER
        self.search_direction = 1
        self.search_tilt = VISION_SERVO_TILT_CENTER
        self.search_tilt_direction = 1
        self.smooth_cx = VISION_FRAME_WIDTH // 2
        self.smooth_cy = VISION_FRAME_HEIGHT // 2
        update_robot_state('mode', 'PATROL')
        print("[OK] Autonomous STARTED")
   
    def stop(self):
        self.is_running = False
        self.motor.stop()
        self.servo.center()
        self.vision.detect_mode = 'none'
        update_robot_state('mode', 'IDLE')
        print("[OK] Autonomous STOPPED")
   
    def update(self):
        if not self.is_running:
            return
       
        self.frame_count += 1
        self.vision.detect_mode = 'face'
        self.vision.process_detections()
       
        if self.frame_count % 5 == 0:
            self.ultrasonic.update()
            if self.ultrasonic.is_obstacle():
                self._handle_obstacle()
                return
       
        face = self.vision.detections.get('face')
       
        if face and face.get('confidence', 0) >= 4:
            if self.state == 'SEARCH':
                self.face_confidence_streak = VISION_FACE_CONFIDENCE_STREAK_THRESHOLD
                self.face_missing_count = 0
           
            cx, cy = face['center']
            dist = math.sqrt((cx - self.last_valid_center[0])**2 +
                           (cy - self.last_valid_center[1])**2)
           
            if dist > VISION_FACE_TRACKING_OUTLIER_THRESHOLD:
                self.face_confidence_streak = 0
            else:
                self.smooth_cx = int(self.alpha * cx + (1 - self.alpha) * self.smooth_cx)
                self.smooth_cy = int(self.alpha * cy + (1 - self.alpha) * self.smooth_cy)
                self.last_valid_center = (cx, cy)
                self.face_confidence_streak += 1
                self.face_missing_count = 0
               
                if self.face_confidence_streak >= VISION_FACE_CONFIDENCE_STREAK_THRESHOLD:
                    self.state = 'FACE_TRACKING'
                    update_robot_state('mode', 'FACE_TRACKING')
                    self._handle_face_tracking()
                    return
        else:
            self.face_confidence_streak = 0
            self.face_missing_count += 1
       
        if self.face_missing_count >= self.face_missing_threshold:
            self.state = 'SEARCH'
            update_robot_state('mode', 'SEARCH')
            self._handle_search()
            return
       
        self.state = 'PATROL'
        update_robot_state('mode', 'PATROL')
        self._handle_patrol()
   
    def _handle_face_tracking(self):
        center_x = VISION_FRAME_WIDTH // 2
        center_y = VISION_FRAME_HEIGHT // 2
       
        error_x = self.smooth_cx - center_x
        error_y = self.smooth_cy - center_y
       
        if abs(error_x) > VISION_SERVO_DEADZONE:
            pan_adjust = -int(error_x / VISION_SERVO_PAN_ADJUST_STEP)
            self.servo.set_pan(self.servo.pan + pan_adjust)
       
        if abs(error_y) > VISION_SERVO_DEADZONE:
            tilt_adjust = -int(error_y / VISION_SERVO_TILT_ADJUST_STEP)
            self.servo.set_tilt(self.servo.tilt + tilt_adjust)
       
        self.motor.stop()
   
    def _handle_search(self):
        self.search_pan += self.search_direction * self.search_speed
       
        if self.search_pan >= VISION_SEARCH_PAN_MAX:
            self.search_direction = -1
            self.search_pan = VISION_SEARCH_PAN_MAX
       
        if self.search_pan <= VISION_SEARCH_PAN_MIN:
            self.search_direction = 1
            self.search_pan = VISION_SEARCH_PAN_MIN
       
        self.search_tilt += self.search_tilt_direction * self.search_tilt_speed
       
        if self.search_tilt >= VISION_SEARCH_TILT_MAX:
            self.search_tilt_direction = -1
            self.search_tilt = VISION_SEARCH_TILT_MAX
       
        if self.search_tilt <= VISION_SEARCH_TILT_MIN:
            self.search_tilt_direction = 1
            self.search_tilt = VISION_SEARCH_TILT_MIN
       
        self.servo.set_pan(self.search_pan)
        self.servo.set_tilt(self.search_tilt)
        self.motor.stop()
       
        if self.frame_count % 60 == 0:
            if self.search_direction == 1:
                self.motor.rotate_left(40)
            else:
                self.motor.rotate_right(40)
            time.sleep(0.5)
            self.motor.stop()
   
    def _handle_patrol(self):
        self.motor.move_forward(VISION_PATROL_SPEED)
   
    def _handle_obstacle(self):
        self.motor.stop()
        time.sleep(0.2)
        self.motor.move_backward(VISION_OBSTACLE_AVOID_SPEED)
        time.sleep(VISION_OBSTACLE_AVOID_DURATION)
        self.motor.stop()
        time.sleep(0.2)
        self.motor.rotate_left(VISION_OBSTACLE_AVOID_ROTATE_SPEED)
        time.sleep(VISION_OBSTACLE_AVOID_ROTATE_DURATION)
        self.motor.stop()
        self.state = 'PATROL'
        update_robot_state('mode', 'PATROL')
   
    def get_status(self):
        return {
            'state': self.state,
            'is_running': self.is_running,
            'distance': round(self.ultrasonic.current_distance, 1) if self.ultrasonic.current_distance > 0 else -1,
            'pan': self.servo.pan,
            'tilt': self.servo.tilt,
            'face_missing': self.face_missing_count,
            'frame_count': self.frame_count,
            'timestamp': time.time()
        }

autonomous = AutonomousController(vision, robot_controller, servo_controller, ultrasonic)
print("[OK] AutonomousController loaded")

# ============================================================================
# AUTONOMOUS LOOP
# ============================================================================

def autonomous_loop():
    while True:
        try:
            if autonomous.is_running and vision.current_frame is not None:
                autonomous.update()
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"[ERROR] Loop: {e}")
            time.sleep(0.1)

auto_thread = threading.Thread(target=autonomous_loop, daemon=True)
auto_thread.start()

print("[OK] Autonomous loop started\n")

# ============================================================================
# FLASK APP
# ============================================================================

app = Flask(__name__)
CORS(app, supports_credentials=True)

@app.before_request
def handle_preflight():
    """Handle CORS preflight (OPTIONS) requests"""
    if request.method == "OPTIONS":
        response = app.make_default_options_response()
        response.headers.add("Access-Control-Allow-Origin", request.headers.get("Origin", "*"))
        response.headers.add("Access-Control-Allow-Headers", "Content-Type")
        response.headers.add("Access-Control-Allow-Methods", "GET, POST, OPTIONS, PUT")
        return response, 200

print("="*70)
print("WEB API ENDPOINTS:")
print("="*70)

@app.route('/api/autonomous/start', methods=['POST', 'OPTIONS'])
def start_autonomous():
    """Start autonomous face tracking"""
    if request.method == 'OPTIONS':
        return '', 204
   
    try:
        autonomous.start()
        return jsonify({
            'status': 'ok',
            'message': 'Face tracking started',
            'mode': 'PATROL - TRACKING - SEARCH'
        }), 200
    except Exception as e:
        print(f"[ERROR] Start: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/api/autonomous/stop', methods=['POST', 'OPTIONS'])
def stop_autonomous():
    """Stop autonomous face tracking"""
    if request.method == 'OPTIONS':
        return '', 204
   
    try:
        autonomous.stop()
        return jsonify({
            'status': 'ok',
            'message': 'Face tracking stopped'
        }), 200
    except Exception as e:
        print(f"[ERROR] Stop: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/api/autonomous/status', methods=['GET'])
def get_status_once():
    return jsonify(autonomous.get_status()), 200

# ============================================================================
# SSE STREAMING - 10Hz
# ============================================================================

def status_stream():
    """Enhanced SSE stream - 10Hz with line sensors"""
    while True:
        try:
            state = get_robot_state()
           
            try:
                sensors = hardware.read_line_sensors()
            except:
                sensors = [0, 0, 0, 0]
           
            status = {
                'state': autonomous.state if autonomous.is_running else 'IDLE',
                'is_running': autonomous.is_running,
                'distance': state['distance'],
                'pan': state['pan_angle'],
                'tilt': state['tilt_angle'],
                'mode': state['mode'],
                'speed': state['speed'],
                'last_command': state['last_command'],
                'line_sensors': sensors,
                'frame_count': autonomous.frame_count if autonomous.is_running else 0,
                'timestamp': time.time()
            }
           
            yield f"data: {json.dumps(status)}\n\n"
            time.sleep(0.1)
           
        except Exception as e:
            print(f"[ERROR] Stream: {e}")
            time.sleep(1)

@app.route('/api/autonomous/stream')
def autonomous_stream():
    return Response(
        status_stream(),
        mimetype='text/event-stream',
        headers={
            'Cache-Control': 'no-cache',
            'X-Accel-Buffering': 'no',
            'Connection': 'keep-alive'
        }
    )

print("  POST /api/autonomous/start   - Start face tracking")
print("  POST /api/autonomous/stop    - Stop face tracking")
print("  GET  /api/autonomous/stream  - Real-time SSE (10Hz)")

def generate_frames():
    """Generate MJPEG frames"""
    while True:
        frame = vision.capture()
        if frame is None:
            time.sleep(0.01)
            continue
       
        if autonomous.is_running:
            vision.process_detections()
            frame = vision.draw_all(frame)
       
        try:
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_bytes = buffer.tobytes() if ret else None
        except:
            frame_bytes = None
       
        if frame_bytes is None:
            time.sleep(0.01)
            continue
       
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n'
               b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n' +
               frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )
@app.route('/api/camera/snapshot', methods=['GET'])
def camera_snapshot():
    """
    Get camera frame for AI vision analysis
    OPTIMIZED: Resize to 320x240, Quality 60% for faster processing
    """
    try:
        import base64
        
        # Get current frame from vision system
        frame = vision.current_frame
        
        if frame is None:
            # Try to capture new frame
            frame = vision.capture()
        
        if frame is None:
            return jsonify({
                'status': 'error',
                'message': 'No camera frame available'
            }), 500
        
        # ? OPTIMIZE: Resize image for faster AI processing
        # 640x480 ? 320x240 (4x less data, 3-5x faster!)
        resized_frame = cv2.resize(frame, (320, 240))
        
        # ? OPTIMIZE: Lower JPEG quality (60% is good enough for AI)
        ret, buffer = cv2.imencode('.jpg', resized_frame, [
            cv2.IMWRITE_JPEG_QUALITY, 60  # Lower = faster
        ])
        
        if not ret:
            return jsonify({
                'status': 'error',
                'message': 'Failed to encode frame'
            }), 500
        
        # Convert to base64
        jpg_base64 = base64.b64encode(buffer).decode('utf-8')
        
        # Log size for debugging
        size_kb = len(jpg_base64) / 1024
        print(f"[SNAPSHOT] ? Captured: {size_kb:.1f}KB (320x240, Q60)")
        
        return jsonify({
            'status': 'ok',
            'image': jpg_base64,
            'width': 320,
            'height': 240,
            'original_size': f"{size_kb:.1f}KB",
            'timestamp': time.time()
        }), 200
        
    except Exception as e:
        print(f"[SNAPSHOT] ? Error: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500


@app.route('/api/camera/snapshot', methods=['OPTIONS'])
def camera_snapshot_options():
    """Handle CORS preflight for snapshot endpoint"""
    response = jsonify({'status': 'ok'})
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return response
# ============================================================================
# MOTOR CONTROL
# ============================================================================

@app.route('/api/command', methods=['POST', 'OPTIONS'])
def command():
    """Motor control"""
    if request.method == 'OPTIONS':
        return '', 204
   
    data = request.json or {}
    action = data.get('action', 'stop')
    speed = data.get('speed', 150)
    angle = data.get('angle', 0)
   
    update_robot_state('last_command', action)
    update_robot_state('mode', 'MANUAL')
   
    if action == 'forward':
        robot_controller.move_forward(speed)
    elif action == 'backward':
        robot_controller.move_backward(speed)
    elif action == 'left':
        robot_controller.move_left(speed)
    elif action == 'right':
        robot_controller.move_right(speed)
    elif action == 'rotate_left':
        robot_controller.rotate_left(speed)
    elif action == 'rotate_right':
        robot_controller.rotate_right(speed)
    elif action == 'go_angle':
        robot_controller.go_angle(angle, speed)
    elif action == 'stop':
        robot_controller.stop()
    else:
        robot_controller.stop()
   
    return jsonify({'status': 'ok', 'action': action}), 200

print("  POST /api/command            - Motor control")

@app.route('/api/servo', methods=['POST', 'OPTIONS'])
def servo_control():
    """Manual servo control - pan and tilt"""
    if request.method == 'OPTIONS':
        return '', 204
   
    try:
        data = request.json or {}
        pan = data.get('pan')
        tilt = data.get('tilt')
       
        if pan is not None:
            servo_controller.set_pan(pan)
            print(f"[SERVO] PAN: {pan}")
       
        if tilt is not None:
            servo_controller.set_tilt(tilt)
            print(f"[SERVO] TILT: {tilt}")
       
        return jsonify({
            'status': 'ok',
            'pan': servo_controller.pan,
            'tilt': servo_controller.tilt
        }), 200
    except Exception as e:
        print(f"[ERROR] Servo: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

print("  POST /api/servo              - Servo control (pan/tilt)")

@app.route('/api/status', methods=['GET'])
def api_status():
    state = get_robot_state()
    return jsonify({
        'server': 'online',
        'camera_ready': vision.current_frame is not None,
        'autonomous_running': autonomous.is_running,
        'distance': state['distance'],
        'mode': state['mode'],
        'speed': state['speed']
    }), 200

print("  GET  /api/status             - Server status")
print("="*70 + "\n")

# ============================================================================
# MODE 3 - AI-ASSISTED ENDPOINTS
# ============================================================================

@app.route('/api/mode/status', methods=['GET'])
def get_mode_status():
    """Get current mode"""
    return jsonify({
        'mode': current_mode,
        'pc_available': PC_AVAILABLE and (pc_comm is not None),
        'pc_ip': PC_IP if PC_AVAILABLE else None
    }), 200

@app.route('/api/mode/set', methods=['POST'])
def set_mode():
    """Set robot mode"""
    global current_mode
    
    data = request.json
    new_mode = data.get('mode', 'manual')
    
    if new_mode not in ['manual', 'autonomous', 'ai']:
        return jsonify({'error': 'Invalid mode'}), 400
    
    # Stop autonomous if switching away
    if current_mode == 'autonomous' and new_mode != 'autonomous':
        try:
            autonomous.stop()
        except:
            pass
    
    current_mode = new_mode
    update_robot_state('mode', current_mode.upper())
    
    print(f"[MODE3] Switched to: {current_mode}")
    
    return jsonify({
        'mode': current_mode,
        'success': True
    }), 200


@app.route('/api/ai/chat', methods=['POST'])
def ai_chat_forward():
    """Forward chat to PC AI"""
    if not PC_AVAILABLE or not pc_comm:
        return jsonify({'status': 'error', 'message': 'PC not connected'}), 503
    
    data = request.json
    message = data.get('message', '')
    image = data.get('image', None)  # ? GET IMAGE FROM REQUEST
    
    if not message:
        return jsonify({'error': 'No message'}), 400
    
    # ? LOG FOR DEBUGGING
    print(f"[AI-FORWARD] Message: {message}")
    if image:
        print(f"[AI-FORWARD] With image: {len(image)} chars")
    else:
        print(f"[AI-FORWARD] No image in request")
    
    # ? FORWARD BOTH MESSAGE AND IMAGE
    result = pc_comm.send_chat(message, image)
    
    return jsonify(result), 200

print("  GET  /api/mode/status        - Get current mode (MODE 3)")
print("  POST /api/mode/set           - Set mode (MODE 3)")
print("  POST /api/ai/chat            - AI chat (MODE 3)")

if __name__ == '__main__':
    # ====================================================================
    # MODE 3 - INIT PC COMMUNICATION
    # ====================================================================
    if PC_AVAILABLE:
        print(f"[PC-COMM] Configured for PC at http://{PC_IP}:{PC_PORT}")
        pc_comm = PCCommunication(PC_IP, PC_PORT)
        print(f"[MODE3] Testing PC connection to {PC_IP}:{PC_PORT}...")
        
        if pc_comm.test_connection():
            pc_running = True
            
            # Thread 1: Send sensor data to PC
            def pc_update_loop():
                global pc_running
                while pc_running:
                    try:
                        state = get_robot_state()
                        data = {
                            'distance': state['distance'],
                            'pan': state['pan_angle'],
                            'tilt': state['tilt_angle'],
                            'mode': current_mode,
                            'line_sensors': state['line_sensors'],
                            'speed': state['speed'],
                            'pc_control_enabled': current_mode == 'ai',
                            'timestamp': state['timestamp']
                        }
                        pc_comm.send_sensor_data(data)
                    except Exception as e:
                        pass
                    time.sleep(PC_UPDATE_INTERVAL)
            
            # FIXED: Thread 2 - Poll commands from PC
            def pc_poll_loop():
                global pc_running
                print("[MODE3-POLL]  Command polling thread started")
                
                while pc_running:
                    try:
                        if current_mode == 'ai':
                            # FIX: Get result dict first
                            result = pc_comm.get_commands()
                            
                            # FIX: Check status and get commands list
                            if result and result.get('status') == 'ok':
                                commands = result.get('commands', [])
                                
                                #  FIX: Now loop the actual commands list
                                for cmd in commands:
                                    action = cmd.get('action')
                                    speed = cmd.get('speed', 150)
                                    
                                    print(f"[MODE3-POLL] Executing: {action}")
                                    
                                    # Execute command
                                    if action == 'move_forward':
                                        robot_controller.move_forward(speed)
                                        time.sleep(1)
                                        robot_controller.stop()
                                    
                                    elif action == 'move_backward':
                                        robot_controller.move_backward(speed)
                                        time.sleep(1)
                                        robot_controller.stop()
                                    
                                    elif action == 'turn_left' or action == 'rotate_left':
                                        robot_controller.rotate_left(speed)
                                        time.sleep(0.5)
                                        robot_controller.stop()
                                    
                                    elif action == 'turn_right' or action == 'rotate_right':
                                        robot_controller.rotate_right(speed)
                                        time.sleep(0.5)
                                        robot_controller.stop()
                                    
                                    elif action == 'stop':
                                        robot_controller.stop()
                                    
                                    elif action == 'start_face_tracking':
                                        autonomous.start()
                                    
                                    elif action == 'pan_left':
                                        servo_controller.set_pan(servo_controller.pan + 10)
                                    
                                    elif action == 'pan_right':
                                        servo_controller.set_pan(servo_controller.pan - 10)
                                    
                                    elif action == 'tilt_up':
                                        servo_controller.set_tilt(servo_controller.tilt - 10)
                                    
                                    elif action == 'tilt_down':
                                        servo_controller.set_tilt(servo_controller.tilt + 10)
                                    
                                    elif action == 'camera_center':
                                        servo_controller.center()
                                    
                                    print(f"[MODE3-POLL]  Done: {action}")
                    
                    except Exception as e:
                        if current_mode == 'ai':
                            print(f"[MODE3-POLL]  Error: {e}")
                    
                    time.sleep(PC_POLL_INTERVAL)
            
            # Start threads
            from threading import Thread
            pc_update_thread = Thread(target=pc_update_loop, daemon=True)
            pc_poll_thread = Thread(target=pc_poll_loop, daemon=True)
            
            pc_update_thread.start()
            pc_poll_thread.start()
            
            print("[MODE3]  PC communication threads started")
            print(f"[MODE3] - Update interval: {PC_UPDATE_INTERVAL}s")
            print(f"[MODE3] - Poll interval: {PC_POLL_INTERVAL}s")
            print("[PC-COMM]  Connected to PC server")
        else:
            print("[MODE3] PC server not reachable")
            print("[MODE3] MODE 3 disabled - start PC server to enable")
    else:
        print("[MODE3] Not installed - MODE 3 disabled")
    print("")

    print("Starting Flask on http://0.0.0.0:5000\n")
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
        autonomous.stop()
        robot_controller.stop()
        vision.release()


