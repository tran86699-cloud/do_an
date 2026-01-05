#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vision Detector - Face & Obstacle detection using OpenCV
Built-in models (no downloads needed)
"""

import cv2
import numpy as np
import time
from utils import logger

class VisionDetector:
    """Vision detection system - faces, obstacles, people"""
    
    def __init__(self):
        """Initialize detector with Haar Cascade"""
        try:
            logger.info("Initializing vision detector...")
            
            # Load Haar Cascade classifiers
            cascade_path = cv2.data.haarcascades
            
            # Face detector
            self.face_cascade = cv2.CascadeClassifier(
                cascade_path + 'haarcascade_frontalface_default.xml'
            )
            
            # Upper body detector (for person detection)
            self.body_cascade = cv2.CascadeClassifier(
                cascade_path + 'haarcascade_upperbody.xml'
            )
            
            # Camera
            self.camera = None
            self.last_frame = None
            self.last_detections = {
                'face': {'detected': False, 'count': 0, 'boxes': []},
                'obstacle': {'detected': False, 'distance': None, 'position': None},
                'person': {'detected': False, 'count': 0, 'boxes': []}
            }
            
            self.available = True
            logger.info("✓ Vision detector initialized")
            
        except Exception as e:
            logger.error(f"Vision detector init error: {e}")
            self.available = False
    
    # ========================================================================
    # CAMERA
    # ========================================================================
    
    def init_camera(self, camera_index=0):
        """Initialize camera"""
        try:
            logger.info("Initializing camera...")
            self.camera = cv2.VideoCapture(camera_index)
            
            if not self.camera.isOpened():
                logger.error("Camera failed to open")
                return False
            
            # Set resolution and FPS
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            logger.info("✓ Camera initialized")
            return True
        
        except Exception as e:
            logger.error(f"Camera init error: {e}")
            return False
    
    def capture_frame(self):
        """Capture frame from camera"""
        if not self.camera:
            return None
        
        try:
            ret, frame = self.camera.read()
            if ret:
                self.last_frame = frame
                return frame
            return None
        except Exception as e:
            logger.error(f"Camera capture error: {e}")
            return None
    
    def release_camera(self):
        """Release camera"""
        if self.camera:
            self.camera.release()
            logger.info("Camera released")
    
    # ========================================================================
    # FACE DETECTION
    # ========================================================================
    
    def detect_face(self, frame=None):
        """Detect faces in frame
        
        Returns:
        {
            'detected': bool,
            'count': int,
            'boxes': [(x, y, w, h), ...],
            'centers': [(cx, cy), ...]
        }
        """
        try:
            if frame is None:
                frame = self.last_frame
            
            if frame is None:
                return {
                    'detected': False,
                    'count': 0,
                    'boxes': [],
                    'centers': []
                }
            
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=4,
                minSize=(30, 30),
                maxSize=(500, 500)
            )
            
            result = {
                'detected': len(faces) > 0,
                'count': len(faces),
                'boxes': faces.tolist() if len(faces) > 0 else [],
                'centers': []
            }
            
            # Get centers
            for (x, y, w, h) in faces:
                cx = x + w // 2
                cy = y + h // 2
                result['centers'].append([cx, cy])
            
            self.last_detections['face'] = result
            
            if result['detected']:
                logger.debug(f"Detected {result['count']} face(s)")
            
            return result
        
        except Exception as e:
            logger.error(f"Face detection error: {e}")
            return {
                'detected': False,
                'count': 0,
                'boxes': [],
                'centers': []
            }
    
    # ========================================================================
    # OBSTACLE DETECTION (Color-based)
    # ========================================================================
    
    def detect_obstacle(self, frame=None, color='red'):
        """Detect obstacles by color (default: red)
        
        Returns:
        {
            'detected': bool,
            'position': 'left'|'center'|'right'|None,
            'center': [x, y] or None,
            'area': int,
            'distance_estimate': str
        }
        """
        try:
            if frame is None:
                frame = self.last_frame
            
            if frame is None:
                return {
                    'detected': False,
                    'position': None,
                    'center': None,
                    'area': 0,
                    'distance_estimate': None
                }
            
            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Color ranges (HSV)
            color_ranges = {
                'red': [
                    (np.array([0, 70, 72]), np.array([10, 255, 255])),
                    (np.array([170, 70, 72]), np.array([180, 255, 255]))
                ],
                'green': [
                    (np.array([54, 109, 78]), np.array([77, 255, 255]))
                ],
                'blue': [
                    (np.array([92, 100, 62]), np.array([121, 251, 255]))
                ]
            }
            
            # Create mask
            mask = None
            if color in color_ranges:
                ranges = color_ranges[color]
                for lower, upper in ranges:
                    m = cv2.inRange(hsv, lower, upper)
                    mask = m if mask is None else cv2.bitwise_or(mask, m)
            
            if mask is None:
                return {
                    'detected': False,
                    'position': None,
                    'center': None,
                    'area': 0,
                    'distance_estimate': None
                }
            
            # Morphological operations
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            
            if len(contours) == 0:
                return {
                    'detected': False,
                    'position': None,
                    'center': None,
                    'area': 0,
                    'distance_estimate': None
                }
            
            # Get largest contour
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            # Min area threshold
            if area < 500:
                return {
                    'detected': False,
                    'position': None,
                    'center': None,
                    'area': int(area),
                    'distance_estimate': None
                }
            
            # Get center
            (cx, cy), radius = cv2.minEnclosingCircle(largest)
            cx, cy = int(cx), int(cy)
            
            # Position in frame
            h, w = frame.shape[:2]
            third_w = w // 3
            
            if cx < third_w:
                position = 'left'
            elif cx > third_w * 2:
                position = 'right'
            else:
                position = 'center'
            
            # Distance estimate based on area
            if area > 50000:
                distance = 'very close'
            elif area > 20000:
                distance = 'close'
            elif area > 5000:
                distance = 'medium'
            else:
                distance = 'far'
            
            result = {
                'detected': True,
                'position': position,
                'center': [cx, cy],
                'area': int(area),
                'radius': int(radius),
                'distance_estimate': distance
            }
            
            self.last_detections['obstacle'] = result
            logger.debug(f"Obstacle detected: {position}, {distance}")
            
            return result
        
        except Exception as e:
            logger.error(f"Obstacle detection error: {e}")
            return {
                'detected': False,
                'position': None,
                'center': None,
                'area': 0,
                'distance_estimate': None
            }
    
    # ========================================================================
    # PERSON DETECTION
    # ========================================================================
    
    def detect_person(self, frame=None):
        """Detect people using upper body cascade
        
        Returns:
        {
            'detected': bool,
            'count': int,
            'boxes': [(x, y, w, h), ...],
            'centers': [(cx, cy), ...]
        }
        """
        try:
            if frame is None:
                frame = self.last_frame
            
            if frame is None:
                return {
                    'detected': False,
                    'count': 0,
                    'boxes': [],
                    'centers': []
                }
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect upper bodies
            bodies = self.body_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=3,
                minSize=(50, 50)
            )
            
            result = {
                'detected': len(bodies) > 0,
                'count': len(bodies),
                'boxes': bodies.tolist() if len(bodies) > 0 else [],
                'centers': []
            }
            
            # Get centers
            for (x, y, w, h) in bodies:
                cx = x + w // 2
                cy = y + h // 2
                result['centers'].append([cx, cy])
            
            self.last_detections['person'] = result
            
            if result['detected']:
                logger.debug(f"Detected {result['count']} person(s)")
            
            return result
        
        except Exception as e:
            logger.error(f"Person detection error: {e}")
            return {
                'detected': False,
                'count': 0,
                'boxes': [],
                'centers': []
            }
    
    # ========================================================================
    # FULL DETECTION (all at once)
    # ========================================================================
    
    def detect_all(self, frame=None):
        """Run all detections on frame"""
        try:
            if frame is None:
                frame = self.last_frame
            
            if frame is None:
                return self.last_detections
            
            # Run detections
            self.detect_face(frame)
            self.detect_obstacle(frame, color='red')
            self.detect_person(frame)
            
            return self.last_detections
        
        except Exception as e:
            logger.error(f"Full detection error: {e}")
            return self.last_detections
    
    # ========================================================================
    # UTILITIES
    # ========================================================================
    
    def get_status(self):
        """Get detector status"""
        return {
            'available': self.available,
            'camera_ready': self.camera is not None,
            'last_detections': self.last_detections,
            'detector_type': 'OpenCV Haar Cascade'
        }
    
    def draw_detections(self, frame=None):
        """Draw detection boxes on frame"""
        if frame is None:
            frame = self.last_frame
        
        if frame is None:
            return None
        
        frame = frame.copy()
        h, w = frame.shape[:2]
        
        # Draw faces
        for (x, y, box_w, box_h) in self.last_detections['face']['boxes']:
            cv2.rectangle(frame, (x, y), (x + box_w, y + box_h), (0, 255, 0), 2)
            cv2.putText(frame, 'Face', (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw obstacle
        if self.last_detections['obstacle']['detected']:
            cx, cy = self.last_detections['obstacle']['center']
            radius = self.last_detections['obstacle'].get('radius', 30)
            cv2.circle(frame, (cx, cy), radius, (0, 0, 255), 2)
            cv2.putText(frame, f"Obstacle: {self.last_detections['obstacle']['distance_estimate']}", 
                       (cx - 60, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Draw persons
        for (x, y, box_w, box_h) in self.last_detections['person']['boxes']:
            cv2.rectangle(frame, (x, y), (x + box_w, y + box_h), (255, 0, 0), 2)
            cv2.putText(frame, 'Person', (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Draw center crosshair
        cx, cy = w // 2, h // 2
        cv2.line(frame, (cx, 0), (cx, h), (0, 255, 255), 1)
        cv2.line(frame, (0, cy), (w, cy), (0, 255, 255), 1)
        cv2.circle(frame, (cx, cy), 8, (0, 255, 255), 2)
        
        return frame
    
    def cleanup(self):
        """Cleanup resources"""
        self.release_camera()

# ============================================================================
# GLOBAL INSTANCE
# ============================================================================

_detector = None

def get_detector():
    """Get or create detector instance"""
    global _detector
    if _detector is None:
        _detector = VisionDetector()
        _detector.init_camera()
    return _detector


