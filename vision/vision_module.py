#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vision Module - High-level vision interface
Manages camera, detections, and provides callbacks for movement control
"""

import cv2
import time
from typing import Callable, Dict, List, Optional
from collections import deque
from enum import Enum

try:
    from utils import logger
except ImportError:
    logger = None

from object_detector import (
    DetectionType,
    Detection,
    get_color_detector,
    get_face_detector,
    get_obstacle_detector,
    DetectionDrawer
)

# Import vision configuration
try:
    from config.vision_config import (
        VISION_CAMERA_WIDTH,
        VISION_CAMERA_HEIGHT,
        VISION_CAMERA_FPS,
        VISION_CAMERA_INDEX,
        VISION_CAPTURE_DELAY,
        VISION_DISPLAY_DELAY,
    )
    if logger:
        logger.info("✓ Vision module config loaded")
except ImportError:
    if logger:
        logger.warning("⚠ Using fallback vision defaults")
    VISION_CAMERA_WIDTH = 640
    VISION_CAMERA_HEIGHT = 480
    VISION_CAMERA_FPS = 30
    VISION_CAMERA_INDEX = 0
    VISION_CAPTURE_DELAY = 0.05
    VISION_DISPLAY_DELAY = 0.03


class VisionMode(Enum):
    """Vision detection modes"""
    IDLE = "idle"
    COLOR_TRACKING = "color_tracking"
    FACE_TRACKING = "face_tracking"
    OBSTACLE_DETECTION = "obstacle_detection"
    MULTI_DETECTION = "multi_detection"


class VisionModule:
    """High-level vision interface"""
    
    def __init__(self, camera_index=VISION_CAMERA_INDEX):
        """Initialize vision module"""
        self.camera = cv2.VideoCapture(camera_index)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, VISION_CAMERA_WIDTH)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, VISION_CAMERA_HEIGHT)
        self.camera.set(cv2.CAP_PROP_FPS, VISION_CAMERA_FPS)
        
        # Detectors
        self.color_detector = get_color_detector()
        self.face_detector = get_face_detector()
        self.obstacle_detector = get_obstacle_detector()
        
        # State
        self.current_mode = VisionMode.IDLE
        self.current_frame = None
        self.current_detections: Dict[DetectionType, Optional[Detection]] = {
            DetectionType.COLOR: None,
            DetectionType.FACE: None,
            DetectionType.OBSTACLE: None,
        }
        
        # Detection history for smoothing
        self.detection_history = deque(maxlen=5)
        
        # Callbacks
        self.on_detection_callbacks: Dict[DetectionType, List[Callable]] = {
            DetectionType.COLOR: [],
            DetectionType.FACE: [],
            DetectionType.OBSTACLE: [],
        }
        
        # Tracking state
        self.tracked_color = 'red'
        self.is_tracking = False
        self.tracking_lost_count = 0
        self.tracking_lost_threshold = 10
        
        if logger:
            logger.info(f"✓ Vision module initialized: {VISION_CAMERA_WIDTH}x{VISION_CAMERA_HEIGHT}@{VISION_CAMERA_FPS}fps")
    
    def capture(self) -> Optional[cv2.Mat]:
        """Capture frame from camera"""
        ret, frame = self.camera.read()
        if ret:
            self.current_frame = frame
            return frame
        return None
    
    def set_mode(self, mode: VisionMode, target_color: str = 'red'):
        """
        Set vision detection mode
        
        Args:
            mode: VisionMode to set
            target_color: Color to track (for COLOR_TRACKING mode)
        """
        self.current_mode = mode
        self.tracked_color = target_color
        
        if logger:
            log_msg = f"✓ Vision mode: {mode.value}"
            if mode == VisionMode.COLOR_TRACKING:
                log_msg += f" (tracking: {target_color})"
            logger.info(log_msg)
    
    def process_frame(self) -> Dict[DetectionType, Optional[Detection]]:
        """
        Process current frame based on mode
        
        Returns:
            Dictionary of detections
        """
        if self.current_frame is None:
            return self.current_detections
        
        try:
            if self.current_mode == VisionMode.IDLE:
                self.current_detections = {
                    DetectionType.COLOR: None,
                    DetectionType.FACE: None,
                    DetectionType.OBSTACLE: None,
                }
            
            elif self.current_mode == VisionMode.COLOR_TRACKING:
                detection = self.color_detector.detect(self.current_frame, self.tracked_color)
                self.current_detections[DetectionType.COLOR] = detection
                self._trigger_callback(DetectionType.COLOR, detection)
            
            elif self.current_mode == VisionMode.FACE_TRACKING:
                detection = self.face_detector.detect(self.current_frame)
                self.current_detections[DetectionType.FACE] = detection
                self._trigger_callback(DetectionType.FACE, detection)
            
            elif self.current_mode == VisionMode.OBSTACLE_DETECTION:
                detection = self.obstacle_detector.detect(self.current_frame)
                self.current_detections[DetectionType.OBSTACLE] = detection
                self._trigger_callback(DetectionType.OBSTACLE, detection)
            
            elif self.current_mode == VisionMode.MULTI_DETECTION:
                # Detect all simultaneously
                color_det = self.color_detector.detect(self.current_frame, self.tracked_color)
                face_det = self.face_detector.detect(self.current_frame)
                obstacle_det = self.obstacle_detector.detect(self.current_frame)
                
                self.current_detections[DetectionType.COLOR] = color_det
                self.current_detections[DetectionType.FACE] = face_det
                self.current_detections[DetectionType.OBSTACLE] = obstacle_det
                
                for det_type, detection in self.current_detections.items():
                    if detection:
                        self._trigger_callback(det_type, detection)
            
            # Track detection loss
            if not self.current_detections.get(DetectionType.COLOR) and \
               self.current_mode == VisionMode.COLOR_TRACKING:
                self.tracking_lost_count += 1
            else:
                self.tracking_lost_count = 0
            
            return self.current_detections
        
        except Exception as e:
            if logger:
                logger.error(f"Frame processing error: {e}")
            return self.current_detections
    
    def draw_detections(self, frame=None) -> Optional[cv2.Mat]:
        """
        Draw all detections on frame
        
        Args:
            frame: Input frame (uses current_frame if None)
        
        Returns:
            Frame with drawn detections
        """
        if frame is None:
            frame = self.current_frame
        
        if frame is None:
            return None
        
        output = frame.copy()
        
        # Draw each detection
        for detection in self.current_detections.values():
            if detection:
                output = DetectionDrawer.draw_detection(output, detection)
        
        # Draw mode indicator
        mode_text = f"Mode: {self.current_mode.value}"
        cv2.putText(output, mode_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw tracking status
        if self.current_mode in [VisionMode.COLOR_TRACKING, VisionMode.FACE_TRACKING]:
            if self.tracking_lost_count > 0:
                status = f"⚠ Lost for {self.tracking_lost_count} frames"
                cv2.putText(output, status, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                cv2.putText(output, "✓ Tracking", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return output
    
    def register_callback(self, detection_type: DetectionType, 
                         callback: Callable[[Detection], None]):
        """
        Register callback for detection
        
        Args:
            detection_type: Type of detection to listen to
            callback: Function to call when detection occurs
        """
        if detection_type in self.on_detection_callbacks:
            self.on_detection_callbacks[detection_type].append(callback)
            if logger:
                logger.debug(f"Registered callback for {detection_type.value}")
    
    def _trigger_callback(self, detection_type: DetectionType, detection: Optional[Detection]):
        """Trigger callbacks for detection"""
        if detection:
            for callback in self.on_detection_callbacks.get(detection_type, []):
                try:
                    callback(detection)
                except Exception as e:
                    if logger:
                        logger.error(f"Callback error: {e}")
    
    def get_detection(self, detection_type: DetectionType) -> Optional[Detection]:
        """Get latest detection of specific type"""
        return self.current_detections.get(detection_type)
    
    def get_tracked_position(self) -> Optional[tuple]:
        """Get current tracked object position (center)"""
        detection = self.current_detections.get(DetectionType.COLOR)
        if detection:
            return detection.center
        return None
    
    def get_face_position(self) -> Optional[tuple]:
        """Get current face position (center)"""
        detection = self.current_detections.get(DetectionType.FACE)
        if detection:
            return detection.center
        return None
    
    def get_obstacle_info(self) -> Optional[Dict]:
        """Get obstacle information"""
        detection = self.current_detections.get(DetectionType.OBSTACLE)
        if detection:
            return {
                'detected': True,
                'position': detection.metadata.get('position'),
                'center': detection.center,
                'confidence': detection.confidence
            }
        return {'detected': False}
    
    def is_tracking_lost(self) -> bool:
        """Check if tracking is lost"""
        return self.tracking_lost_count > self.tracking_lost_threshold
    
    def get_stats(self) -> Dict:
        """Get vision module statistics"""
        return {
            'mode': self.current_mode.value,
            'tracked_color': self.tracked_color,
            'tracking_lost': self.is_tracking_lost(),
            'lost_count': self.tracking_lost_count,
            'active_detections': sum(1 for d in self.current_detections.values() if d is not None),
            'frame_size': (VISION_CAMERA_WIDTH, VISION_CAMERA_HEIGHT)
        }
    
    def release(self):
        """Release camera"""
        if self.camera:
            self.camera.release()
        if logger:
            logger.info("✓ Vision module released")


# Integration helper for movement control
class VisionMovementBridge:
    """Bridge between vision and movement controller"""
    
    def __init__(self, vision_module: VisionModule, movement_controller=None):
        """
        Args:
            vision_module: VisionModule instance
            movement_controller: MovementController instance
        """
        self.vision = vision_module
        self.movement = movement_controller
    
    def follow_color(self, target_color: str, forward_speed=150, rotate_speed=120):
        """Set up color tracking with auto-movement"""
        self.vision.set_mode(VisionMode.COLOR_TRACKING, target_color)
        
        def on_color_detected(detection: Detection):
            if not self.movement:
                return
            
            h, w = self.vision.current_frame.shape[:2]
            cx, cy = detection.center
            center_x = w // 2
            error = cx - center_x
            threshold = 30
            
            if abs(error) < threshold:
                # In center - move forward
                self.movement.move_forward(speed=forward_speed, duration=0.1)
            elif error > 0:
                # Right side - rotate right
                self.movement.rotate_right(speed=rotate_speed, duration=0.1)
            else:
                # Left side - rotate left
                self.movement.rotate_left(speed=rotate_speed, duration=0.1)
        
        self.vision.register_callback(DetectionType.COLOR, on_color_detected)
    
    def follow_face(self, forward_speed=150, rotate_speed=120):
        """Set up face tracking with auto-movement"""
        self.vision.set_mode(VisionMode.FACE_TRACKING)
        
        def on_face_detected(detection: Detection):
            if not self.movement:
                return
            
            area = detection.area
            
            if 2000 < area < 5000:
                self.movement.move_forward(speed=forward_speed, duration=0.1)
            elif area < 2000:
                self.movement.move_forward(speed=forward_speed, duration=0.1)
            else:
                self.movement.move_backward(speed=forward_speed, duration=0.1)
        
        self.vision.register_callback(DetectionType.FACE, on_face_detected)
    
    def avoid_obstacles(self, rotate_speed=120):
        """Set up obstacle avoidance"""
        self.vision.set_mode(VisionMode.OBSTACLE_DETECTION)
        
        def on_obstacle_detected(detection: Detection):
            if not self.movement:
                return
            
            position = detection.metadata.get('position', 'center')
            
            if position == 'center':
                self.movement.rotate_right(speed=rotate_speed, duration=1.0)
            elif position == 'left':
                self.movement.rotate_right(speed=rotate_speed, duration=0.8)
            elif position == 'right':
                self.movement.rotate_left(speed=rotate_speed, duration=0.8)
        
        self.vision.register_callback(DetectionType.OBSTACLE, on_obstacle_detected)


# Singleton instance
_vision_module = None

def get_vision_module(camera_index=VISION_CAMERA_INDEX):
    """Get or create vision module"""
    global _vision_module
    if _vision_module is None:
        _vision_module = VisionModule(camera_index)
    return _vision_module
