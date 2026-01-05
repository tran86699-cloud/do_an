#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vision Detector - Real-time detection using OpenCV + TensorFlow + MediaPipe
Fixed version with camera stream integration
"""

import cv2
import numpy as np
import threading
import time
import requests
from io import BytesIO

try:
    import tensorflow as tf
    TF_AVAILABLE = True
except ImportError:
    TF_AVAILABLE = False
    print("⚠️ TensorFlow not available")

try:
    import mediapipe as mp
    MP_AVAILABLE = True
except ImportError:
    MP_AVAILABLE = False
    print("⚠️ MediaPipe not available")

# ============================================================================
# VISION DETECTOR CLASS
# ============================================================================

class VisionDetector:
    """Real-time vision detection"""
    
    def __init__(self, robot_api_url="http://192.168.100.13:5000"):
        self.robot_api_url = robot_api_url
        self.running = True
        self.frame = None
        self.frame_lock = threading.Lock()
        
        print("\n" + "="*60)
        print("🎥 VISION DETECTOR - INITIALIZATION")
        print("="*60)
        
        # Initialize detectors
        self.init_face_detector()
        
        if MP_AVAILABLE:
            self.init_pose_detector()
        else:
            self.pose = None
        
        if MP_AVAILABLE:
            self.init_hand_detector()
        else:
            self.hands = None
        
        # Cache results
        self.face_results = {'detected': False, 'count': 0, 'boxes': []}
        self.obstacle_results = {'detected': False, 'position': 'unknown', 'area': 0}
        self.pose_results = {'detected': False, 'landmarks': []}
        self.hand_results = {'detected': False, 'count': 0, 'gestures': []}
        
        # Start threads
        self.camera_thread = threading.Thread(
            target=self._camera_loop,
            daemon=True
        )
        self.camera_thread.start()
        
        self.process_thread = threading.Thread(
            target=self._process_loop,
            daemon=True
        )
        self.process_thread.start()
        
        print("✅ Vision Detector initialized\n")
    
    # ==================== INITIALIZATION ====================
    
    def init_face_detector(self):
        """Initialize Haar Cascade face detector"""
        try:
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            
            if self.face_cascade.empty():
                raise Exception("Failed to load cascade")
            
            print("✅ Face detector loaded (Haar Cascade)")
        except Exception as e:
            print(f"❌ Face detector error: {e}")
            self.face_cascade = None
    
    def init_pose_detector(self):
        """Initialize MediaPipe pose detector"""
        try:
            self.mp_pose = mp.solutions.pose
            self.pose = self.mp_pose.Pose(
                static_image_mode=False,
                model_complexity=1,
                smooth_landmarks=True,
                min_detection_confidence=0.5
            )
            print("✅ Pose detector loaded (MediaPipe)")
        except Exception as e:
            print(f"❌ Pose detector error: {e}")
            self.pose = None
    
    def init_hand_detector(self):
        """Initialize MediaPipe hand detector"""
        try:
            self.mp_hands = mp.solutions.hands
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=2,
                min_detection_confidence=0.5
            )
            print("✅ Hand detector loaded (MediaPipe)")
        except Exception as e:
            print(f"❌ Hand detector error: {e}")
            self.hands = None
    
    # ==================== CAMERA STREAMING ====================
    
    def _camera_loop(self):
        """Background thread to get frames from robot camera"""
        print("Camera streaming thread started")
        failed_count = 0
        frame_count = 0
        
        while self.running:
            try:
                frame = self._fetch_frame_from_robot()
                
                if frame is not None:
                    with self.frame_lock:
                        self.frame = frame
                    
                    frame_count += 1
                    failed_count = 0
                else:
                    failed_count += 1
                    if failed_count % 30 == 0:
                        print(f"⚠️ Waiting for camera feed... ({failed_count} attempts)")
                
                time.sleep(0.1)
            
            except Exception as e:
                if failed_count % 30 == 0:
                    print(f"Camera stream error: {e}")
                failed_count += 1
                time.sleep(1)
        
        print(f"Camera streaming thread stopped ({frame_count} frames)")
    
    def _fetch_frame_from_robot(self):
        """Fetch frame from robot API video stream"""
        try:
            stream_url = f"{self.robot_api_url}/video_feed"
            
            response = requests.get(
                stream_url,
                stream=True,
                timeout=3
            )
            
            if response.status_code != 200:
                return None
            
            bytes_data = b''
            
            for chunk in response.iter_content(chunk_size=1024):
                if not chunk:
                    break
                
                bytes_data += chunk
                
                # Find JPEG frame boundaries
                a = bytes_data.find(b'\xff\xd8')  # JPEG start
                b = bytes_data.find(b'\xff\xd9')  # JPEG end
                
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b+2]
                    bytes_data = bytes_data[b+2:]
                    
                    # Decode frame
                    nparr = np.frombuffer(jpg, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    return frame
        
        except Exception as e:
            return None
    
    # ==================== PROCESSING LOOP ====================
    
    def _process_loop(self):
        """Background thread for processing frames"""
        print("Vision processing thread started")
        process_count = 0
        
        while self.running:
            try:
                with self.frame_lock:
                    frame = self.frame
                
                if frame is None:
                    time.sleep(0.1)
                    continue
                
                # Run detections
                self.detect_faces(frame)
                self.detect_obstacles(frame)
                
                if self.pose:
                    self.detect_pose(frame)
                
                if self.hands:
                    self.detect_hands(frame)
                
                process_count += 1
            
            except Exception as e:
                print(f"Processing error: {e}")
            
            time.sleep(0.05)
        
        print(f"Processing thread stopped ({process_count} frames processed)")
    
    # ==================== FACE DETECTION ====================
    
    def detect_faces(self, frame):
        """Detect faces using Haar Cascade"""
        if self.face_cascade is None or frame is None:
            self.face_results = {'detected': False, 'count': 0, 'boxes': []}
            return
        
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30)
            )
            
            boxes = []
            for (x, y, w, h) in faces:
                boxes.append({
                    'x': int(x),
                    'y': int(y),
                    'w': int(w),
                    'h': int(h),
                    'confidence': 0.85
                })
            
            self.face_results = {
                'detected': len(faces) > 0,
                'count': len(faces),
                'boxes': boxes
            }
        
        except Exception as e:
            self.face_results = {'detected': False, 'count': 0, 'boxes': []}
    
    # ==================== OBSTACLE DETECTION ====================
    
    def detect_obstacles(self, frame):
        """Detect obstacles using color-based detection"""
        if frame is None:
            self.obstacle_results = {'detected': False, 'position': 'unknown', 'area': 0}
            return
        
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Detect dark colors
            lower_dark = np.array([0, 0, 0])
            upper_dark = np.array([180, 255, 60])
            
            mask = cv2.inRange(hsv, lower_dark, upper_dark)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            obstacles = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 1000:
                    x, y, w, h = cv2.boundingRect(contour)
                    position = self._get_position(x, frame.shape[1])
                    
                    obstacles.append({
                        'x': int(x),
                        'y': int(y),
                        'w': int(w),
                        'h': int(h),
                        'area': int(area),
                        'position': position
                    })
            
            if obstacles:
                closest = max(obstacles, key=lambda o: o['area'])
                self.obstacle_results = {
                    'detected': True,
                    'position': closest['position'],
                    'area': closest['area']
                }
            else:
                self.obstacle_results = {'detected': False, 'position': 'clear', 'area': 0}
        
        except Exception as e:
            self.obstacle_results = {'detected': False, 'position': 'unknown', 'area': 0}
    
    # ==================== POSE DETECTION ====================
    
    def detect_pose(self, frame):
        """Detect human pose using MediaPipe"""
        if self.pose is None or frame is None:
            self.pose_results = {'detected': False, 'landmarks': []}
            return
        
        try:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.pose.process(rgb_frame)
            
            if results.pose_landmarks:
                landmarks = []
                for landmark in results.pose_landmarks.landmark:
                    landmarks.append({
                        'x': float(landmark.x),
                        'y': float(landmark.y),
                        'z': float(landmark.z),
                        'visibility': float(landmark.visibility)
                    })
                
                self.pose_results = {
                    'detected': True,
                    'landmarks': landmarks,
                    'count': len(landmarks)
                }
            else:
                self.pose_results = {'detected': False, 'landmarks': []}
        
        except Exception as e:
            self.pose_results = {'detected': False, 'landmarks': []}
    
    # ==================== HAND DETECTION ====================
    
    def detect_hands(self, frame):
        """Detect hands and recognize gestures"""
        if self.hands is None or frame is None:
            self.hand_results = {'detected': False, 'count': 0, 'gestures': []}
            return
        
        try:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb_frame)
            
            if results.multi_hand_landmarks and results.multi_handedness:
                gestures = []
                
                for hand_landmarks, handedness in zip(
                    results.multi_hand_landmarks,
                    results.multi_handedness
                ):
                    gesture = self._recognize_gesture(hand_landmarks.landmark)
                    
                    gestures.append({
                        'hand': handedness.classification[0].label,
                        'gesture': gesture,
                        'confidence': float(handedness.classification[0].score)
                    })
                
                self.hand_results = {
                    'detected': True,
                    'count': len(gestures),
                    'gestures': gestures
                }
            else:
                self.hand_results = {'detected': False, 'count': 0, 'gestures': []}
        
        except Exception as e:
            self.hand_results = {'detected': False, 'count': 0, 'gestures': []}
    
    def _recognize_gesture(self, landmarks):
        """Recognize hand gesture"""
        try:
            wrist = landmarks[0]
            thumb_tip = landmarks[4]
            index_tip = landmarks[8]
            middle_tip = landmarks[12]
            ring_tip = landmarks[16]
            pinky_tip = landmarks[20]
            
            fingers_up = 0
            
            if thumb_tip.x > wrist.x:
                fingers_up += 1
            if index_tip.y < wrist.y:
                fingers_up += 1
            if middle_tip.y < wrist.y:
                fingers_up += 1
            if ring_tip.y < wrist.y:
                fingers_up += 1
            if pinky_tip.y < wrist.y:
                fingers_up += 1
            
            if fingers_up == 0:
                return 'closed_fist'
            elif fingers_up == 1:
                return 'pointing'
            elif fingers_up == 2:
                return 'peace'
            elif fingers_up == 5:
                return 'open_hand'
            else:
                return 'hand_gesture'
        
        except:
            return 'unknown'
    
    def _get_position(self, x, width):
        """Get position relative to frame center"""
        center = width // 2
        
        if x < center - 100:
            return 'left'
        elif x > center + 100:
            return 'right'
        else:
            return 'center'
    
    # ==================== PUBLIC METHODS ====================
    
    def get_face_detection(self):
        """Get face detection results"""
        return self.face_results
    
    def get_obstacle_detection(self):
        """Get obstacle detection results"""
        return self.obstacle_results
    
    def get_pose_detection(self):
        """Get pose detection results"""
        return self.pose_results
    
    def get_hand_detection(self):
        """Get hand detection results"""
        return self.hand_results
    
    def get_all_detections(self):
        """Get all detection results"""
        return {
            'faces': self.face_results,
            'obstacles': self.obstacle_results,
            'pose': self.pose_results,
            'hands': self.hand_results
        }
    
    def stop(self):
        """Stop detector"""
        self.running = False
        if self.pose:
            self.pose.close()
        if self.hands:
            self.hands.close()

# ============================================================================
# GLOBAL INSTANCE
# ============================================================================

_vision_detector = None

def get_vision_detector():
    """Get or create vision detector instance"""
    global _vision_detector
    if _vision_detector is None:
        _vision_detector = VisionDetector()
    return _vision_detector

if __name__ == '__main__':
    detector = get_vision_detector()
    print("\n✅ Vision Detector initialized and running")
    print("Waiting for camera frames...")
    
    try:
        while True:
            time.sleep(1)
            print(f"Faces: {detector.get_face_detection()['count']}, "
                  f"Obstacles: {detector.get_obstacle_detection()['detected']}, "
                  f"Hands: {detector.get_hand_detection()['count']}")
    except KeyboardInterrupt:
        print("\nShutdown")
        detector.stop()