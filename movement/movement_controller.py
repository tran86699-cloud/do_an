#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Movement Controller - High-level robot movement abstraction
Integrates with hardware motors and uses centralized configuration
"""

import time
import math
from enum import Enum

try:
    from hardware.motors import get_motors
    from hardware.movement import get_movement
    from utils import logger
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    logger = None

# Import movement configuration
try:
    from config.hardware_config import (
        DEFAULT_SPEED,
        MIN_SPEED,
        MAX_SPEED,
        MOTOR_LEFT_CALIBRATION,
        MOTOR_RIGHT_CALIBRATION,
    )
    logger.info("✓ Movement config loaded from hardware_config")
except ImportError:
    logger.warning("⚠ Using fallback movement defaults")
    DEFAULT_SPEED = 150
    MIN_SPEED = 0
    MAX_SPEED = 255
    MOTOR_LEFT_CALIBRATION = 1.0
    MOTOR_RIGHT_CALIBRATION = 1.0


class MovementDirection(Enum):
    """Movement directions"""
    FORWARD = 90
    BACKWARD = 270
    LEFT = 180
    RIGHT = 0
    ROTATE_LEFT = 180
    ROTATE_RIGHT = 0
    STOP = None


class MovementController:
    """High-level movement controller with trajectory support"""
    
    def __init__(self):
        """Initialize movement controller"""
        self.movement = get_movement() if HARDWARE_AVAILABLE else None
        self.motors = get_motors() if HARDWARE_AVAILABLE else None
        self.current_speed = DEFAULT_SPEED
        self.is_moving = False
        self.movement_history = []  # Track movement for trajectory
        
        if logger:
            logger.info("✓ Movement controller initialized")
    
    def set_speed(self, speed):
        """Set default movement speed (0-255)"""
        self.current_speed = max(MIN_SPEED, min(MAX_SPEED, speed))
        if self.movement:
            self.movement.set_speed(self.current_speed)
    
    def move(self, direction, speed=None, duration=None):
        """
        Move in specified direction
        
        Args:
            direction: MovementDirection enum
            speed: Optional override speed (0-255)
            duration: Optional duration in seconds
        """
        speed = speed or self.current_speed
        speed = max(MIN_SPEED, min(MAX_SPEED, speed))
        
        if not self.movement:
            logger.warning("⚠ Movement not available (mock mode)")
            return False
        
        try:
            if direction == MovementDirection.FORWARD:
                self.movement.move_forward(speed=speed, duration=duration)
                self._record_movement("forward", speed, duration)
                
            elif direction == MovementDirection.BACKWARD:
                self.movement.move_backward(speed=speed, duration=duration)
                self._record_movement("backward", speed, duration)
                
            elif direction == MovementDirection.LEFT:
                self.movement.turn_left(speed=speed, duration=duration)
                self._record_movement("left", speed, duration)
                
            elif direction == MovementDirection.RIGHT:
                self.movement.turn_right(speed=speed, duration=duration)
                self._record_movement("right", speed, duration)
                
            elif direction == MovementDirection.ROTATE_LEFT:
                self.movement.rotate_left(speed=speed, duration=duration)
                self._record_movement("rotate_left", speed, duration)
                
            elif direction == MovementDirection.ROTATE_RIGHT:
                self.movement.rotate_right(speed=speed, duration=duration)
                self._record_movement("rotate_right", speed, duration)
                
            elif direction == MovementDirection.STOP:
                self.stop()
                return True
            
            self.is_moving = True
            return True
        
        except Exception as e:
            if logger:
                logger.error(f"Movement error: {e}")
            return False
    
    def move_forward(self, speed=None, duration=None):
        """Move forward"""
        return self.move(MovementDirection.FORWARD, speed, duration)
    
    def move_backward(self, speed=None, duration=None):
        """Move backward"""
        return self.move(MovementDirection.BACKWARD, speed, duration)
    
    def turn_left(self, speed=None, duration=None):
        """Turn left"""
        return self.move(MovementDirection.LEFT, speed, duration)
    
    def turn_right(self, speed=None, duration=None):
        """Turn right"""
        return self.move(MovementDirection.RIGHT, speed, duration)
    
    def rotate_left(self, speed=None, duration=None):
        """Rotate left in place"""
        return self.move(MovementDirection.ROTATE_LEFT, speed, duration)
    
    def rotate_right(self, speed=None, duration=None):
        """Rotate right in place"""
        return self.move(MovementDirection.ROTATE_RIGHT, speed, duration)
    
    def stop(self):
        """Stop all movement"""
        if self.movement:
            self.movement.stop()
        self.is_moving = False
        self._record_movement("stop", 0, 0)
        if logger:
            logger.info("Movement stopped")
        return True
    
    def move_to_distance(self, distance_cm, speed=None):
        """
        Move forward for estimated distance
        
        Args:
            distance_cm: Distance in centimeters
            speed: Optional speed override
        
        Note: This is estimation-based. Actual distance depends on:
        - Motor speed consistency
        - Surface friction
        - Battery voltage
        - Motor calibration
        """
        speed = speed or self.current_speed
        
        # Estimation: ~15cm per second at speed=150
        # Adjust based on MOTOR_LEFT_CALIBRATION and MOTOR_RIGHT_CALIBRATION
        estimated_speed_cm_s = (speed / 255.0) * 20  # Linear approximation
        estimated_duration = distance_cm / estimated_speed_cm_s if estimated_speed_cm_s > 0 else 0
        
        if logger:
            logger.info(f"Moving ~{distance_cm}cm (est. {estimated_duration:.2f}s at speed={speed})")
        
        return self.move_forward(speed=speed, duration=estimated_duration)
    
    def _record_movement(self, direction, speed, duration):
        """Record movement for trajectory tracking"""
        self.movement_history.append({
            'timestamp': time.time(),
            'direction': direction,
            'speed': speed,
            'duration': duration or 0
        })
    
    def get_movement_history(self):
        """Get recorded movement history"""
        return self.movement_history.copy()
    
    def clear_history(self):
        """Clear movement history"""
        self.movement_history.clear()
    
    def apply_calibration(self):
        """Apply motor calibration factors"""
        if self.motors and MOTOR_LEFT_CALIBRATION != 1.0:
            logger.info(f"Left motor calibration: {MOTOR_LEFT_CALIBRATION}")
        if self.motors and MOTOR_RIGHT_CALIBRATION != 1.0:
            logger.info(f"Right motor calibration: {MOTOR_RIGHT_CALIBRATION}")


# Singleton instance
_movement_controller = None

def get_movement_controller():
    """Get or create movement controller instance"""
    global _movement_controller
    if _movement_controller is None:
        _movement_controller = MovementController()
    return _movement_controller


