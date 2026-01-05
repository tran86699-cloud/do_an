#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Motor Control Module - Using Raspbot_Lib"""

import sys
import os

# Add project path to sys.path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from utils import logger

# Try to import Raspbot_Lib
try:
    from Raspbot_Lib import Raspbot
    logger.info("? Raspbot_Lib imported successfully")
    RASPBOT_AVAILABLE = True
except ImportError as e:
    logger.warning(f"??  Raspbot_Lib import failed: {e}")
    logger.warning("   Running in MOCK MODE")
    Raspbot = None
    RASPBOT_AVAILABLE = False

class Motors:
    """Motor Controller using Raspbot_Lib"""
    
    def __init__(self):
        self.available = False
        self.bot = None
        
        try:
            if RASPBOT_AVAILABLE and Raspbot is not None:
                self.bot = Raspbot()
                self.available = True
                logger.info("? Raspbot connected")
                logger.info("? Motor control ready")
            else:
                logger.warning("??  Raspbot not available - MOCK MODE")
                self.available = False
            
        except Exception as e:
            logger.error(f"Motor initialization error: {e}")
            logger.warning("??  Using MOCK MODE")
            self.available = False
    
    def set_motor(self, motor_id, direction, speed):
        """Set motor (motor_id: 0=L1, 1=L2, 2=R1, 3=R2)"""
        if not self.available or self.bot is None:
            logger.debug(f"[MOCK] Motor {motor_id}: dir={direction}, speed={speed}")
            return
        
        try:
            speed = max(0, min(255, speed))
            self.bot.Ctrl_Car(motor_id, direction, speed)
            logger.debug(f"Motor {motor_id}: dir={direction}, speed={speed}")
        except Exception as e:
            logger.error(f"Motor error: {e}")
    
    def motor_forward(self, motor_id, speed):
        """Motor forward"""
        self.set_motor(motor_id, 0, speed)
    
    def motor_backward(self, motor_id, speed):
        """Motor backward"""
        self.set_motor(motor_id, 1, speed)
    
    def motor_stop(self, motor_id):
        """Stop motor"""
        self.set_motor(motor_id, 0, 0)
    
    def stop_all(self):
        """Stop all motors"""
        for motor_id in range(4):
            self.motor_stop(motor_id)
        logger.debug("All motors stopped")

_motors = None

def get_motors():
    global _motors
    if _motors is None:
        _motors = Motors()
    return _motors
