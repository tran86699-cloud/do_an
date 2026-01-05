#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RASPBOT V2 COMPLETE - IMPROVED with Servo Control + Ultrasonic + Trackbar + BETTER FACE DETECTION"""

import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'

import cv2
import numpy as np
import time
import math
import threading
from collections import deque

print("✓ Loading complete system...")

# ============================================================================
# RASPBOT_LIB INTEGRATION
# ============================================================================

class RaspbotHardware:
    """Raspbot_Lib hardware wrapper"""
    
    def __init__(self):
        try:
            from Raspbot_Lib import Raspbot
            self.bot = Raspbot()
            self.available = True
            print("✓ Raspbot_Lib initialized")
        except ImportError:
            print("⚠ Raspbot_Lib not available (simulation mode)")
            self.bot = None
            self.available = False
    
    def read_ultrasonic(self):
        """Read ultrasonic distance"""
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
        except Exception as e:
            print(f"⚠ Ultrasonic error: {e}")
            return -1
    
    def set_servo(self, servo_id, angle):
        """Set servo angle (0-180)"""
        if not self.available or not self.bot:
            return
        
        try:
            angle = max(0, min(180, angle))
            self.bot.Ctrl_Servo(servo_id, angle)
        except Exception as e:
            print(f"⚠ Servo error: {e}")
    
    def set_motors(self, l1, l2, r1, r2):
        """Set all motors"""
        if not self.available or not self.bot:
            return
        
        try:
            self.bot.Ctrl_Muto(0, l1)
            self.bot.Ctrl_Muto(1, l2)
            self.bot.Ctrl_Muto(2, r1)
            self.bot.Ctrl_Muto(3, r2)
        except Exception as e:
            print(f"⚠ Motor error: {e}")

hardware = RaspbotHardware()

print("✓ RaspbotHardware ready\n")

# ============================================================================
# SERVO CONTROLLER WITH TRACKBAR
# ============================================================================

class ServoController:
    """Camera servo controller with trackbar support"""
    
    def __init__(self):
        self.pan = 90
        self.tilt = 45
        self.hardware = hardware
    
    def set_pan(self, angle):
        """Set pan angle"""
        self.pan = max(0, min(180, angle))
        self.hardware.set_servo(1, self.pan)
    
    def set_tilt(self, angle):
        """Set tilt angle"""
        self.tilt = max(0, min(180, angle))
        self.hardware.set_servo(2, self.tilt)
    
    def center(self):
        """Center camera"""
        self.set_pan(90)
        self.set_tilt(45)
    
    def print_status(self):
        """Print current servo status"""
        return f"Pan: {self.pan}°, Tilt: {self.tilt}°"

servo_controller = ServoController()
servo_controller.center()

print("✓ ServoController initialized\n")

# ============================================================================
# ULTRASONIC SENSOR MONITOR
# ============================================================================

class UltrasonicMonitor:
    """Monitor distance and detect obstacles"""
    
    def __init__(self, danger_distance=20):
        self.danger_distance = danger_distance
        self.current_distance = -1
        self.distance_history = deque(maxlen=10)
        self.hardware = hardware
    
    def update(self):
        """Update distance reading"""
        distance = self.hardware.read_ultrasonic()
        if distance > 0:
            self.current_distance = distance
            self.distance_history.append(distance)
        return self.current_distance
    
    def is_obstacle(self):
        """Check if obstacle detected"""
        return 0 < self.current_distance < self.danger_distance
    
    def get_status(self):
        """Get status string"""
        if self.current_distance < 0:
            return "⚠ N/A"
        elif self.is_obstacle():
            return f"🚨 {self.current_distance:.1f}cm DANGER!"
        elif self.current_distance < 50:
            return f"⚠ {self.current_distance:.1f}cm (CLOSE)"
        else:
            return f"✓ {self.current_distance:.1f}cm (SAFE)"

ultrasonic = UltrasonicMonitor(danger_distance=20)

print("✓ UltrasonicMonitor initialized\n")

# ============================================================================
# COLOR TRACKER
# ============================================================================

class ColorTracker:
    """Detect colors"""
    
    COLOR_RANGES = {
        'red': {
            'lower1': np.array([0, 70, 72]),
            'upper1': np.array([10, 255, 255]),
            'lower2': np.array([170, 70, 72]),
            'upper2': np.array([180, 255, 255]),
            'color': (0, 0, 255),
            'name': 'RED'
        },
        'green': {
            'lower': np.array([54, 109, 78]),
            'upper': np.array([77, 255, 255]),
            'color': (0, 255, 0),
            'name': 'GREEN'
        },
        'blue': {
            'lower': np.array([92, 100, 62]),
            'upper': np.array([121, 251, 255]),
            'color': (255, 0, 0),
            'name': 'BLUE'
        },
        'yellow': {
            'lower': np.array([26, 100, 91]),
            'upper': np.array([32, 255, 255]),
            'color': (0, 255, 255),
            'name': 'YELLOW'
        }
    }
    
    def detect(self, frame, target_color='red'):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        if target_color not in self.COLOR_RANGES:
            return None
        
        color_config = self.COLOR_RANGES[target_color]
        
        if 'lower2' in color_config:
            mask1 = cv2.inRange(hsv, color_config['lower1'], color_config['upper1'])
            mask2 = cv2.inRange(hsv, color_config['lower2'], color_config['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, color_config['lower'], color_config['upper'])
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area < 500:
            return None
        
        (cx, cy), radius = cv2.minEnclosingCircle(largest)
        
        return {
            'center': (int(cx), int(cy)),
            'radius': int(radius),
            'area': int(area),
            'color': target_color
        }
    
    def draw(self, frame, detection):
        if not detection:
            return frame
        
        cx, cy = detection['center']
        radius = detection['radius']
        color_name = detection['color']
        
        if color_name not in self.COLOR_RANGES:
            return frame
        
        color = self.COLOR_RANGES[color_name]['color']
        display_name = self.COLOR_RANGES[color_name]['name']
        
        cv2.circle(frame, (cx, cy), radius, color, 2)
        cv2.putText(frame, display_name, (cx - 30, cy - radius - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        return frame

print("✓ ColorTracker loaded")

# ============================================================================
# IMPROVED FACE TRACKER
# ============================================================================

class FaceTracker:
    """Detect faces - IMPROVED: Less sensitive + Stable bbox"""
    
    def __init__(self):
        self.cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        self.bbox_history = deque(maxlen=5)
        self.consecutive_count = 0
        self.min_frames_for_detection = 2
    
    def detect(self, frame):
        """Detect faces with improved stability"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        faces = self.cascade.detectMultiScale(
            gray, 
            scaleFactor=1.08,
            minNeighbors=8,         # STRICT (was 3)
            minSize=(50, 50),       
            maxSize=(400, 400),
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
        
        # BIGGER EXPANSION: 35% (was 20%)
        expand_x = int(w * 0.35)
        expand_y = int(h * 0.35)
        
        x = max(0, x - expand_x)
        y = max(0, y - expand_y)
        w_new = w + expand_x * 2
        h_new = h + expand_y * 2
        
        detection = {
            'bbox': (x, y, w_new, h_new),
            'center': (x + w_new//2, y + h_new//2),
            'area': w_new * h_new
        }
        
        # TEMPORAL SMOOTHING
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
        """Draw face detection"""
        if not detection:
            return frame
        
        x, y, w, h = detection['bbox']
        confidence = detection.get('confidence', 0)
        
        if confidence >= 4:
            color = (0, 255, 0)
        elif confidence >= 2:
            color = (0, 255, 255)
        else:
            color = (0, 165, 255)
        
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 3)
        
        label_text = f"FACE (L{confidence})"
        label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
        cv2.rectangle(frame, (x, y - 30), (x + label_size[0] + 10, y), color, -1)
        cv2.putText(frame, label_text, (x + 5, y - 8),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
        
        cx, cy = detection['center']
        cv2.circle(frame, (cx, cy), 8, color, -1)
        cv2.circle(frame, (cx, cy), 10, color, 2)
        
        return frame

print("✓ FaceTracker loaded (IMPROVED)")

# ============================================================================
# VISION SYSTEM WITH TRACKBAR
# ============================================================================

class VisionSystem:
    """Camera and vision with servo trackbar"""
    
    def __init__(self, camera_index=0):
        self.camera = cv2.VideoCapture(camera_index)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        
        self.color_tracker = ColorTracker()
        self.face_tracker = FaceTracker()
        
        self.current_frame = None
        self.detections = {}
        
        self.detect_mode = 'none'
        self.current_color = 'red'
        
        self.servo = servo_controller
        self.ultrasonic = ultrasonic
    
    def capture(self):
        """Capture frame"""
        ret, frame = self.camera.read()
        if ret:
            self.current_frame = frame
            return frame
        return None
    
    def process_detections(self):
        """Process detections based on mode"""
        if self.current_frame is None:
            self.detections = {}
            return
        
        if self.detect_mode == 'face':
            detection = self.face_tracker.detect(self.current_frame)
            self.detections['face'] = detection
            self.detections['color'] = None
        
        elif self.detect_mode == 'color':
            detection = self.color_tracker.detect(self.current_frame, self.current_color)
            self.detections['color'] = detection
            self.detections['face'] = None
        
        else:
            self.detections = {}
    
    def draw_all(self, frame):
        """Draw all detections"""
        frame = self.color_tracker.draw(frame, self.detections.get('color'))
        frame = self.face_tracker.draw(frame, self.detections.get('face'))
        return frame
    
    def release(self):
        self.camera.release()

vision = VisionSystem()

print("✓ VisionSystem loaded")

# ============================================================================
# MOTOR CONTROLLER
# ============================================================================

class RobotController:
    """Control motors"""
    
    def __init__(self):
        self.hardware = hardware
        self.moving = False
    
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
        """Move forward with obstacle check"""
        if ultrasonic.is_obstacle():
            print("🚨 OBSTACLE DETECTED! Cannot move forward")
            self.stop()
            return False
        
        l1, l2, r1, r2 = self.set_deflection(speed, 90)
        self._set_motors(l1, l2, r1, r2)
        self.moving = True
        return True
    
    def move_backward(self, speed=150):
        l1, l2, r1, r2 = self.set_deflection(speed, 270)
        self._set_motors(l1, l2, r1, r2)
        self.moving = True
    
    def rotate_left(self, speed=150):
        l1, l2, r1, r2 = self.set_deflection(speed, 180)
        l1, l2, r1, r2 = -l1, -l2, r1, r2
        self._set_motors(l1, l2, r1, r2)
        self.moving = True
    
    def rotate_right(self, speed=150):
        l1, l2, r1, r2 = self.set_deflection(speed, 0)
        l1, l2, r1, r2 = l1, l2, -r1, -r2
        self._set_motors(l1, l2, r1, r2)
        self.moving = True
    
    def stop(self):
        self._set_motors(0, 0, 0, 0)
        self.moving = False
    
    def _set_motors(self, l1, l2, r1, r2):
        self.hardware.set_motors(l1, l2, r1, r2)

motor = RobotController()

print("✓ RobotController loaded\n")

# ============================================================================
# COMMAND PARSER
# ============================================================================

class CommandParser:
    """Parse commands"""
    
    COMMANDS = {
        'move_forward': ['forward', 'ahead', 'go', 'tiến'],
        'move_backward': ['back', 'backward', 'lùi'],
        'turn_left': ['left', 'trái', 'quay trái'],
        'turn_right': ['right', 'phải', 'quay phải'],
        'find_red': ['find red', 'red', 'đỏ'],
        'find_green': ['find green', 'green', 'xanh'],
        'find_blue': ['find blue', 'blue', 'xanh dương'],
        'find_yellow': ['find yellow', 'yellow', 'vàng'],
        'find_face': ['find face', 'face', 'mặt'],
        'stop': ['stop', 'dừng'],
        'servo': ['servo', 'camera'],
    }
    
    def parse(self, user_input):
        user_input = user_input.lower().strip()
        
        for command, keywords in self.COMMANDS.items():
            for keyword in keywords:
                if keyword in user_input:
                    return {'action': command}
        
        return {'action': 'unknown'}

print("✓ CommandParser loaded\n")

# ============================================================================
# MAIN SYSTEM
# ============================================================================

class RaspbotV2Complete:
    """Complete Raspbot V2 System"""
    
    def __init__(self):
        print("="*70)
        print("RASPBOT V2 COMPLETE - IMPROVED")
        print("="*70 + "\n")
        
        self.vision = vision
        self.motor = motor
        self.servo = servo_controller
        self.ultrasonic = ultrasonic
        self.parser = CommandParser()
        
        self.display_enabled = True
        self.window_name = 'Raspbot V2 - Camera Control'
        
        print("✓ System initialized!\n")
    
    def vision_thread_func(self):
        """Vision capture and display with trackbar"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 900, 700)
        
        # Create trackbars
        cv2.createTrackbar('Pan', self.window_name, 90, 180, 
                          lambda x: servo_controller.set_pan(x))
        cv2.createTrackbar('Tilt', self.window_name, 45, 180, 
                          lambda x: servo_controller.set_tilt(x))
        
        while self.display_enabled:
            frame = self.vision.capture()
            if frame is None:
                continue
            
            self.ultrasonic.update()
            self.vision.process_detections()
            frame = self.vision.draw_all(frame)
            
            h, w = frame.shape[:2]
            
            # Crosshair
            cx, cy = w // 2, h // 2
            cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)
            cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)
            cv2.circle(frame, (cx, cy), 8, (0, 255, 0), 2)
            
            # Info panel
            y_offset = 30
            cv2.putText(frame, f"Mode: {self.vision.detect_mode.upper()}", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            if self.vision.detect_mode == 'color':
                cv2.putText(frame, f"Color: {self.vision.current_color.upper()}", 
                           (10, y_offset + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Servo status
            servo_status = self.servo.print_status()
            cv2.putText(frame, servo_status, 
                       (10, y_offset + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            
            # Ultrasonic status
            ultrasonic_status = self.ultrasonic.get_status()
            color_ultra = (0, 0, 255) if self.ultrasonic.is_obstacle() else (0, 255, 0)
            cv2.putText(frame, ultrasonic_status, 
                       (10, y_offset + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_ultra, 2)
            
            # Motor status
            motor_status = "MOVING" if self.motor.moving else "IDLE"
            cv2.putText(frame, f"Motor: {motor_status}", 
                       (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow(self.window_name, frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.display_enabled = False
            
            time.sleep(0.03)
        
        cv2.destroyAllWindows()
    
    def interactive(self):
        """Interactive command mode"""
        print("="*70)
        print("CONTROL INTERFACE")
        print("="*70)
        print("\n📍 MOVEMENT (checked for obstacles):")
        print("   • forward / backward / left / right")
        print("\n🔍 DETECTION:")
        print("   • find face / find red/green/blue/yellow")
        print("\n📷 CAMERA:")
        print("   • servo (adjust trackbars in window)")
        print("\n⏹ CONTROL:")
        print("   • stop / quit\n")
        
        vision_thread = threading.Thread(target=self.vision_thread_func, daemon=True)
        vision_thread.start()
        
        time.sleep(1)
        
        while True:
            try:
                user_input = input("You: ").strip()
                
                if not user_input:
                    continue
                
                if user_input.lower() in ['quit', 'exit']:
                    break
                
                parsed = self.parser.parse(user_input)
                action = parsed['action']
                
                print(f"⚡ {action}\n")
                
                if action == 'move_forward':
                    self.vision.detect_mode = 'none'
                    if not self.motor.move_forward(150):
                        print("Cannot move - obstacle ahead!\n")
                    else:
                        print("→ Moving forward\n")
                
                elif action == 'move_backward':
                    self.vision.detect_mode = 'none'
                    self.motor.move_backward(150)
                    print("← Moving backward\n")
                
                elif action == 'turn_left':
                    self.vision.detect_mode = 'none'
                    self.motor.rotate_left(150)
                    print("↙ Turning left\n")
                
                elif action == 'turn_right':
                    self.vision.detect_mode = 'none'
                    self.motor.rotate_right(150)
                    print("↘ Turning right\n")
                
                elif action == 'stop':
                    self.vision.detect_mode = 'none'
                    self.motor.stop()
                    print("⏹ Stopped\n")
                
                elif action == 'find_face':
                    print("👤 Detecting face\n")
                    self.vision.detect_mode = 'face'
                
                elif action == 'find_red':
                    print("🔴 Detecting RED\n")
                    self.vision.detect_mode = 'color'
                    self.vision.current_color = 'red'
                
                elif action == 'find_green':
                    print("🟢 Detecting GREEN\n")
                    self.vision.detect_mode = 'color'
                    self.vision.current_color = 'green'
                
                elif action == 'find_blue':
                    print("🔵 Detecting BLUE\n")
                    self.vision.detect_mode = 'color'
                    self.vision.current_color = 'blue'
                
                elif action == 'find_yellow':
                    print("🟡 Detecting YELLOW\n")
                    self.vision.detect_mode = 'color'
                    self.vision.current_color = 'yellow'
                
                elif action == 'servo':
                    print("📷 Servo control: Adjust Pan/Tilt trackbars in window\n")
                
                elif action == 'unknown':
                    print("❓ Unknown command\n")
            
            except KeyboardInterrupt:
                print("\n\nInterrupted")
                break
            except Exception as e:
                print(f"✗ Error: {e}\n")
        
        print("\n" + "="*70)
        print("Shutting down...")
        print("="*70)
        
        self.display_enabled = False
        self.motor.stop()
        self.servo.center()
        self.vision.release()
        cv2.destroyAllWindows()
        
        print("\n✓ Raspbot V2 stopped\n")

# ============================================================================
# MAIN
# ============================================================================

if __name__ == '__main__':
    try:
        system = RaspbotV2Complete()
        system.interactive()
    
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RASPBOT V2 COMPLETE - IMPROVED with Servo Control + Ultrasonic + Trackbar + BETTER FACE DETECTION"""

import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'

import cv2
import numpy as np
import time
import math
import threading
from collections import deque

print("✓ Loading complete system...")

# ============================================================================
# RASPBOT_LIB INTEGRATION
# ============================================================================

class RaspbotHardware:
    """Raspbot_Lib hardware wrapper"""
    
    def __init__(self):
        try:
            from Raspbot_Lib import Raspbot
            self.bot = Raspbot()
            self.available = True
            print("✓ Raspbot_Lib initialized")
        except ImportError:
            print("⚠ Raspbot_Lib not available (simulation mode)")
            self.bot = None
            self.available = False
    
    def read_ultrasonic(self):
        """Read ultrasonic distance"""
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
        except Exception as e:
            print(f"⚠ Ultrasonic error: {e}")
            return -1
    
    def set_servo(self, servo_id, angle):
        """Set servo angle (0-180)"""
        if not self.available or not self.bot:
            return
        
        try:
            angle = max(0, min(180, angle))
            self.bot.Ctrl_Servo(servo_id, angle)
        except Exception as e:
            print(f"⚠ Servo error: {e}")
    
    def set_motors(self, l1, l2, r1, r2):
        """Set all motors"""
        if not self.available or not self.bot:
            return
        
        try:
            self.bot.Ctrl_Muto(0, l1)
            self.bot.Ctrl_Muto(1, l2)
            self.bot.Ctrl_Muto(2, r1)
            self.bot.Ctrl_Muto(3, r2)
        except Exception as e:
            print(f"⚠ Motor error: {e}")

hardware = RaspbotHardware()

print("✓ RaspbotHardware ready\n")

# ============================================================================
# SERVO CONTROLLER WITH TRACKBAR
# ============================================================================

class ServoController:
    """Camera servo controller with trackbar support"""
    
    def __init__(self):
        self.pan = 90
        self.tilt = 45
        self.hardware = hardware
    
    def set_pan(self, angle):
        """Set pan angle"""
        self.pan = max(0, min(180, angle))
        self.hardware.set_servo(1, self.pan)
    
    def set_tilt(self, angle):
        """Set tilt angle"""
        self.tilt = max(0, min(180, angle))
        self.hardware.set_servo(2, self.tilt)
    
    def center(self):
        """Center camera"""
        self.set_pan(90)
        self.set_tilt(45)
    
    def print_status(self):
        """Print current servo status"""
        return f"Pan: {self.pan}°, Tilt: {self.tilt}°"

servo_controller = ServoController()
servo_controller.center()

print("✓ ServoController initialized\n")

# ============================================================================
# ULTRASONIC SENSOR MONITOR
# ============================================================================

class UltrasonicMonitor:
    """Monitor distance and detect obstacles"""
    
    def __init__(self, danger_distance=20):
        self.danger_distance = danger_distance
        self.current_distance = -1
        self.distance_history = deque(maxlen=10)
        self.hardware = hardware
    
    def update(self):
        """Update distance reading"""
        distance = self.hardware.read_ultrasonic()
        if distance > 0:
            self.current_distance = distance
            self.distance_history.append(distance)
        return self.current_distance
    
    def is_obstacle(self):
        """Check if obstacle detected"""
        return 0 < self.current_distance < self.danger_distance
    
    def get_status(self):
        """Get status string"""
        if self.current_distance < 0:
            return "⚠ N/A"
        elif self.is_obstacle():
            return f"🚨 {self.current_distance:.1f}cm DANGER!"
        elif self.current_distance < 50:
            return f"⚠ {self.current_distance:.1f}cm (CLOSE)"
        else:
            return f"✓ {self.current_distance:.1f}cm (SAFE)"

ultrasonic = UltrasonicMonitor(danger_distance=20)

print("✓ UltrasonicMonitor initialized\n")

# ============================================================================
# COLOR TRACKER
# ============================================================================

class ColorTracker:
    """Detect colors"""
    
    COLOR_RANGES = {
        'red': {
            'lower1': np.array([0, 70, 72]),
            'upper1': np.array([10, 255, 255]),
            'lower2': np.array([170, 70, 72]),
            'upper2': np.array([180, 255, 255]),
            'color': (0, 0, 255),
            'name': 'RED'
        },
        'green': {
            'lower': np.array([54, 109, 78]),
            'upper': np.array([77, 255, 255]),
            'color': (0, 255, 0),
            'name': 'GREEN'
        },
        'blue': {
            'lower': np.array([92, 100, 62]),
            'upper': np.array([121, 251, 255]),
            'color': (255, 0, 0),
            'name': 'BLUE'
        },
        'yellow': {
            'lower': np.array([26, 100, 91]),
            'upper': np.array([32, 255, 255]),
            'color': (0, 255, 255),
            'name': 'YELLOW'
        }
    }
    
    def detect(self, frame, target_color='red'):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        if target_color not in self.COLOR_RANGES:
            return None
        
        color_config = self.COLOR_RANGES[target_color]
        
        if 'lower2' in color_config:
            mask1 = cv2.inRange(hsv, color_config['lower1'], color_config['upper1'])
            mask2 = cv2.inRange(hsv, color_config['lower2'], color_config['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, color_config['lower'], color_config['upper'])
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area < 500:
            return None
        
        (cx, cy), radius = cv2.minEnclosingCircle(largest)
        
        return {
            'center': (int(cx), int(cy)),
            'radius': int(radius),
            'area': int(area),
            'color': target_color
        }
    
    def draw(self, frame, detection):
        if not detection:
            return frame
        
        cx, cy = detection['center']
        radius = detection['radius']
        color_name = detection['color']
        
        if color_name not in self.COLOR_RANGES:
            return frame
        
        color = self.COLOR_RANGES[color_name]['color']
        display_name = self.COLOR_RANGES[color_name]['name']
        
        cv2.circle(frame, (cx, cy), radius, color, 2)
        cv2.putText(frame, display_name, (cx - 30, cy - radius - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        return frame

print("✓ ColorTracker loaded")

# ============================================================================
# IMPROVED FACE TRACKER
# ============================================================================

class FaceTracker:
    """Detect faces - IMPROVED: Less sensitive + Stable bbox"""
    
    def __init__(self):
        self.cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        self.bbox_history = deque(maxlen=5)
        self.consecutive_count = 0
        self.min_frames_for_detection = 2
    
    def detect(self, frame):
        """Detect faces with improved stability"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        faces = self.cascade.detectMultiScale(
            gray, 
            scaleFactor=1.08,
            minNeighbors=8,         # STRICT (was 3)
            minSize=(50, 50),       
            maxSize=(400, 400),
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
        
        # BIGGER EXPANSION: 35% (was 20%)
        expand_x = int(w * 0.35)
        expand_y = int(h * 0.35)
        
        x = max(0, x - expand_x)
        y = max(0, y - expand_y)
        w_new = w + expand_x * 2
        h_new = h + expand_y * 2
        
        detection = {
            'bbox': (x, y, w_new, h_new),
            'center': (x + w_new//2, y + h_new//2),
            'area': w_new * h_new
        }
        
        # TEMPORAL SMOOTHING
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
        """Draw face detection"""
        if not detection:
            return frame
        
        x, y, w, h = detection['bbox']
        confidence = detection.get('confidence', 0)
        
        if confidence >= 4:
            color = (0, 255, 0)
        elif confidence >= 2:
            color = (0, 255, 255)
        else:
            color = (0, 165, 255)
        
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 3)
        
        label_text = f"FACE (L{confidence})"
        label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
        cv2.rectangle(frame, (x, y - 30), (x + label_size[0] + 10, y), color, -1)
        cv2.putText(frame, label_text, (x + 5, y - 8),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
        
        cx, cy = detection['center']
        cv2.circle(frame, (cx, cy), 8, color, -1)
        cv2.circle(frame, (cx, cy), 10, color, 2)
        
        return frame

print("✓ FaceTracker loaded (IMPROVED)")

# ============================================================================
# VISION SYSTEM WITH TRACKBAR
# ============================================================================

class VisionSystem:
    """Camera and vision with servo trackbar"""
    
    def __init__(self, camera_index=0):
        self.camera = cv2.VideoCapture(camera_index)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        
        self.color_tracker = ColorTracker()
        self.face_tracker = FaceTracker()
        
        self.current_frame = None
        self.detections = {}
        
        self.detect_mode = 'none'
        self.current_color = 'red'
        
        self.servo = servo_controller
        self.ultrasonic = ultrasonic
    
    def capture(self):
        """Capture frame"""
        ret, frame = self.camera.read()
        if ret:
            self.current_frame = frame
            return frame
        return None
    
    def process_detections(self):
        """Process detections based on mode"""
        if self.current_frame is None:
            self.detections = {}
            return
        
        if self.detect_mode == 'face':
            detection = self.face_tracker.detect(self.current_frame)
            self.detections['face'] = detection
            self.detections['color'] = None
        
        elif self.detect_mode == 'color':
            detection = self.color_tracker.detect(self.current_frame, self.current_color)
            self.detections['color'] = detection
            self.detections['face'] = None
        
        else:
            self.detections = {}
    
    def draw_all(self, frame):
        """Draw all detections"""
        frame = self.color_tracker.draw(frame, self.detections.get('color'))
        frame = self.face_tracker.draw(frame, self.detections.get('face'))
        return frame
    
    def release(self):
        self.camera.release()

vision = VisionSystem()

print("✓ VisionSystem loaded")

# ============================================================================
# MOTOR CONTROLLER
# ============================================================================

class RobotController:
    """Control motors"""
    
    def __init__(self):
        self.hardware = hardware
        self.moving = False
    
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
        """Move forward with obstacle check"""
        if ultrasonic.is_obstacle():
            print("🚨 OBSTACLE DETECTED! Cannot move forward")
            self.stop()
            return False
        
        l1, l2, r1, r2 = self.set_deflection(speed, 90)
        self._set_motors(l1, l2, r1, r2)
        self.moving = True
        return True
    
    def move_backward(self, speed=150):
        l1, l2, r1, r2 = self.set_deflection(speed, 270)
        self._set_motors(l1, l2, r1, r2)
        self.moving = True
    
    def rotate_left(self, speed=150):
        l1, l2, r1, r2 = self.set_deflection(speed, 180)
        l1, l2, r1, r2 = -l1, -l2, r1, r2
        self._set_motors(l1, l2, r1, r2)
        self.moving = True
    
    def rotate_right(self, speed=150):
        l1, l2, r1, r2 = self.set_deflection(speed, 0)
        l1, l2, r1, r2 = l1, l2, -r1, -r2
        self._set_motors(l1, l2, r1, r2)
        self.moving = True
    
    def stop(self):
        self._set_motors(0, 0, 0, 0)
        self.moving = False
    
    def _set_motors(self, l1, l2, r1, r2):
        self.hardware.set_motors(l1, l2, r1, r2)

motor = RobotController()

print("✓ RobotController loaded\n")

# ============================================================================
# COMMAND PARSER
# ============================================================================

class CommandParser:
    """Parse commands"""
    
    COMMANDS = {
        'move_forward': ['forward', 'ahead', 'go', 'tiến'],
        'move_backward': ['back', 'backward', 'lùi'],
        'turn_left': ['left', 'trái', 'quay trái'],
        'turn_right': ['right', 'phải', 'quay phải'],
        'find_red': ['find red', 'red', 'đỏ'],
        'find_green': ['find green', 'green', 'xanh'],
        'find_blue': ['find blue', 'blue', 'xanh dương'],
        'find_yellow': ['find yellow', 'yellow', 'vàng'],
        'find_face': ['find face', 'face', 'mặt'],
        'stop': ['stop', 'dừng'],
        'servo': ['servo', 'camera'],
    }
    
    def parse(self, user_input):
        user_input = user_input.lower().strip()
        
        for command, keywords in self.COMMANDS.items():
            for keyword in keywords:
                if keyword in user_input:
                    return {'action': command}
        
        return {'action': 'unknown'}

print("✓ CommandParser loaded\n")

# ============================================================================
# MAIN SYSTEM
# ============================================================================

class RaspbotV2Complete:
    """Complete Raspbot V2 System"""
    
    def __init__(self):
        print("="*70)
        print("RASPBOT V2 COMPLETE - IMPROVED")
        print("="*70 + "\n")
        
        self.vision = vision
        self.motor = motor
        self.servo = servo_controller
        self.ultrasonic = ultrasonic
        self.parser = CommandParser()
        
        self.display_enabled = True
        self.window_name = 'Raspbot V2 - Camera Control'
        
        print("✓ System initialized!\n")
    
    def vision_thread_func(self):
        """Vision capture and display with trackbar"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 900, 700)
        
        # Create trackbars
        cv2.createTrackbar('Pan', self.window_name, 90, 180, 
                          lambda x: servo_controller.set_pan(x))
        cv2.createTrackbar('Tilt', self.window_name, 45, 180, 
                          lambda x: servo_controller.set_tilt(x))
        
        while self.display_enabled:
            frame = self.vision.capture()
            if frame is None:
                continue
            
            self.ultrasonic.update()
            self.vision.process_detections()
            frame = self.vision.draw_all(frame)
            
            h, w = frame.shape[:2]
            
            # Crosshair
            cx, cy = w // 2, h // 2
            cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)
            cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)
            cv2.circle(frame, (cx, cy), 8, (0, 255, 0), 2)
            
            # Info panel
            y_offset = 30
            cv2.putText(frame, f"Mode: {self.vision.detect_mode.upper()}", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            if self.vision.detect_mode == 'color':
                cv2.putText(frame, f"Color: {self.vision.current_color.upper()}", 
                           (10, y_offset + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Servo status
            servo_status = self.servo.print_status()
            cv2.putText(frame, servo_status, 
                       (10, y_offset + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            
            # Ultrasonic status
            ultrasonic_status = self.ultrasonic.get_status()
            color_ultra = (0, 0, 255) if self.ultrasonic.is_obstacle() else (0, 255, 0)
            cv2.putText(frame, ultrasonic_status, 
                       (10, y_offset + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_ultra, 2)
            
            # Motor status
            motor_status = "MOVING" if self.motor.moving else "IDLE"
            cv2.putText(frame, f"Motor: {motor_status}", 
                       (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow(self.window_name, frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.display_enabled = False
            
            time.sleep(0.03)
        
        cv2.destroyAllWindows()
    
    def interactive(self):
        """Interactive command mode"""
        print("="*70)
        print("CONTROL INTERFACE")
        print("="*70)
        print("\n📍 MOVEMENT (checked for obstacles):")
        print("   • forward / backward / left / right")
        print("\n🔍 DETECTION:")
        print("   • find face / find red/green/blue/yellow")
        print("\n📷 CAMERA:")
        print("   • servo (adjust trackbars in window)")
        print("\n⏹ CONTROL:")
        print("   • stop / quit\n")
        
        vision_thread = threading.Thread(target=self.vision_thread_func, daemon=True)
        vision_thread.start()
        
        time.sleep(1)
        
        while True:
            try:
                user_input = input("You: ").strip()
                
                if not user_input:
                    continue
                
                if user_input.lower() in ['quit', 'exit']:
                    break
                
                parsed = self.parser.parse(user_input)
                action = parsed['action']
                
                print(f"⚡ {action}\n")
                
                if action == 'move_forward':
                    self.vision.detect_mode = 'none'
                    if not self.motor.move_forward(150):
                        print("Cannot move - obstacle ahead!\n")
                    else:
                        print("→ Moving forward\n")
                
                elif action == 'move_backward':
                    self.vision.detect_mode = 'none'
                    self.motor.move_backward(150)
                    print("← Moving backward\n")
                
                elif action == 'turn_left':
                    self.vision.detect_mode = 'none'
                    self.motor.rotate_left(150)
                    print("↙ Turning left\n")
                
                elif action == 'turn_right':
                    self.vision.detect_mode = 'none'
                    self.motor.rotate_right(150)
                    print("↘ Turning right\n")
                
                elif action == 'stop':
                    self.vision.detect_mode = 'none'
                    self.motor.stop()
                    print("⏹ Stopped\n")
                
                elif action == 'find_face':
                    print("👤 Detecting face\n")
                    self.vision.detect_mode = 'face'
                
                elif action == 'find_red':
                    print("🔴 Detecting RED\n")
                    self.vision.detect_mode = 'color'
                    self.vision.current_color = 'red'
                
                elif action == 'find_green':
                    print("🟢 Detecting GREEN\n")
                    self.vision.detect_mode = 'color'
                    self.vision.current_color = 'green'
                
                elif action == 'find_blue':
                    print("🔵 Detecting BLUE\n")
                    self.vision.detect_mode = 'color'
                    self.vision.current_color = 'blue'
                
                elif action == 'find_yellow':
                    print("🟡 Detecting YELLOW\n")
                    self.vision.detect_mode = 'color'
                    self.vision.current_color = 'yellow'
                
                elif action == 'servo':
                    print("📷 Servo control: Adjust Pan/Tilt trackbars in window\n")
                
                elif action == 'unknown':
                    print("❓ Unknown command\n")
            
            except KeyboardInterrupt:
                print("\n\nInterrupted")
                break
            except Exception as e:
                print(f"✗ Error: {e}\n")
        
        print("\n" + "="*70)
        print("Shutting down...")
        print("="*70)
        
        self.display_enabled = False
        self.motor.stop()
        self.servo.center()
        self.vision.release()
        cv2.destroyAllWindows()
        
        print("\n✓ Raspbot V2 stopped\n")

# ============================================================================
# MAIN
# ============================================================================

if __name__ == '__main__':
    try:
        system = RaspbotV2Complete()
        system.interactive()
    
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


