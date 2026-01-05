#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Movement Module - High-level movement functions for Mecanum wheels"""

from hardware.motors import get_motors
from utils import logger
import time

class Movement:
    """High-level movement controller for Mecanum wheels
    
    Motor layout (from above):
        L1(0)  L2(1)
        R1(2)  R2(3)
    
    Where:
    - L = Left side
    - R = Right side
    - 1,2 = Front/Back wheels
    """
    
    def __init__(self):
        self.motors = get_motors()
        self.speed = 150
        self.is_moving = False
    
    def set_speed(self, speed):
        """Set default speed (0-255)"""
        self.speed = max(0, min(255, speed))
        logger.debug(f"Speed set to {self.speed}")
    
    # ========================================================================
    # BASIC MOVEMENT
    # ========================================================================
    
    def move_forward(self, speed=None, duration=None):
        """Move forward - all motors forward"""
        if not self.motors.available:
            logger.warning("Motors not available")
            return
        speed = speed or self.speed
        
        try:
            # Forward: all wheels go forward
            self.motors.motor_forward(0, speed)  # L1
            self.motors.motor_forward(1, speed)  # L2
            self.motors.motor_forward(2, speed)  # R1
            self.motors.motor_forward(3, speed)  # R2
            
            self.is_moving = True
            logger.info(f"Moving forward (speed: {speed})")
            
            if duration:
                time.sleep(duration)
                self.stop()
        except Exception as e:
            logger.error(f"Move forward error: {e}")
    
    def move_backward(self, speed=None, duration=None):
        """Move backward - all motors backward"""
        if not self.motors.available:
            logger.warning("Motors not available")
            return
        speed = speed or self.speed
        
        try:
            # Backward: all wheels go backward
            self.motors.motor_backward(0, speed)  # L1
            self.motors.motor_backward(1, speed)  # L2
            self.motors.motor_backward(2, speed)  # R1
            self.motors.motor_backward(3, speed)  # R2
            
            self.is_moving = True
            logger.info(f"Moving backward (speed: {speed})")
            
            if duration:
                time.sleep(duration)
                self.stop()
        except Exception as e:
            logger.error(f"Move backward error: {e}")
    
    # ========================================================================
    # STRAFE MOVEMENT (sideways - no rotation)
    # ========================================================================
    
    def turn_left(self, speed=None, duration=None):
        """Strafe left (move left without rotating)
        
        Mecanum formula for strafe left:
        L1 backward + L2 forward + R1 forward + R2 backward
        """
        if not self.motors.available:
            logger.warning("Motors not available")
            return
        speed = speed or self.speed
        
        try:
            self.motors.motor_backward(0, speed)  # L1
            self.motors.motor_forward(1, speed)   # L2
            self.motors.motor_forward(2, speed)   # R1
            self.motors.motor_backward(3, speed)  # R2
            
            self.is_moving = True
            logger.info(f"Strafing left (speed: {speed})")
            
            if duration:
                time.sleep(duration)
                self.stop()
        except Exception as e:
            logger.error(f"Strafe left error: {e}")
    
    def turn_right(self, speed=None, duration=None):
        """Strafe right (move right without rotating)
        
        Mecanum formula for strafe right:
        L1 forward + L2 backward + R1 backward + R2 forward
        """
        if not self.motors.available:
            logger.warning("Motors not available")
            return
        speed = speed or self.speed
        
        try:
            self.motors.motor_forward(0, speed)   # L1
            self.motors.motor_backward(1, speed)  # L2
            self.motors.motor_backward(2, speed)  # R1
            self.motors.motor_forward(3, speed)   # R2
            
            self.is_moving = True
            logger.info(f"Strafing right (speed: {speed})")
            
            if duration:
                time.sleep(duration)
                self.stop()
        except Exception as e:
            logger.error(f"Strafe right error: {e}")
    
    # ========================================================================
    # ROTATION MOVEMENT (rotate in place)
    # ========================================================================
    
    def rotate_left(self, speed=None, duration=None):
        """Rotate left in place (counter-clockwise)
        
        Mecanum formula for rotate left:
        L1 backward + L2 backward + R1 forward + R2 forward
        """
        if not self.motors.available:
            logger.warning("Motors not available")
            return
        speed = speed or self.speed
        
        try:
            self.motors.motor_backward(0, speed)  # L1
            self.motors.motor_backward(1, speed)  # L2
            self.motors.motor_forward(2, speed)   # R1
            self.motors.motor_forward(3, speed)   # R2
            
            self.is_moving = True
            logger.info(f"Rotating left (speed: {speed})")
            
            if duration:
                time.sleep(duration)
                self.stop()
        except Exception as e:
            logger.error(f"Rotate left error: {e}")
    
    def rotate_right(self, speed=None, duration=None):
        """Rotate right in place (clockwise)
        
        Mecanum formula for rotate right:
        L1 forward + L2 forward + R1 backward + R2 backward
        """
        if not self.motors.available:
            logger.warning("Motors not available")
            return
        speed = speed or self.speed
        
        try:
            self.motors.motor_forward(0, speed)   # L1
            self.motors.motor_forward(1, speed)   # L2
            self.motors.motor_backward(2, speed)  # R1
            self.motors.motor_backward(3, speed)  # R2
            
            self.is_moving = True
            logger.info(f"Rotating right (speed: {speed})")
            
            if duration:
                time.sleep(duration)
                self.stop()
        except Exception as e:
            logger.error(f"Rotate right error: {e}")
    
    # ========================================================================
    # STOP
    # ========================================================================
    
    def stop(self):
        """Stop all motors"""
        try:
            self.motors.stop_all()
            self.is_moving = False
            logger.info("STOP")
        except Exception as e:
            logger.error(f"Stop error: {e}")

# ============================================================================
# GLOBAL INSTANCE
# ============================================================================

_movement = None

def get_movement():
    """Get or create movement instance"""
    global _movement
    if _movement is None:
        _movement = Movement()
    return _movement


