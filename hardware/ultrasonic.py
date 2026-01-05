#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ultrasonic Distance Sensor via Raspbot_Lib I2C"""

from utils import logger
import time
from Raspbot_Lib import Raspbot
from config.sensor_config import (
    ULTRASONIC_NEAR, ULTRASONIC_MID, ULTRASONIC_FAR
)

class UltrasonicSensor:
    """HC-SR04 Ultrasonic Sensor via I2C (Raspbot_Lib)"""
    
    def __init__(self):
        self.available = False
        self.distance = 0
        self.bot = None
        
        self._init_sensor()
    
    def _init_sensor(self):
        """Initialize ultrasonic sensor via Raspbot_Lib"""
        try:
            logger.info("Initializing ultrasonic (I2C via Raspbot_Lib)...")
            
            self.bot = Raspbot()
            
            # Enable ultrasonic sensor
            self.bot.Ctrl_Ulatist_Switch(1)
            time.sleep(0.5)
            
            self.available = True
            logger.info("✓ Ultrasonic sensor initialized (I2C)")
        
        except Exception as e:
            logger.error(f"Ultrasonic init error: {e}")
            self.available = False
    
    def measure_distance(self):
        """Measure distance in cm via I2C"""
        if not self.available or not self.bot:
            return 0
        
        try:
            # Read distance from I2C registers
            # Register 0x1b = HIGH byte, 0x1a = LOW byte
            dis_H_arr = self.bot.read_data_array(0x1b, 1)
            dis_L_arr = self.bot.read_data_array(0x1a, 1)
            
            if not dis_H_arr or not dis_L_arr:
                return self.distance
            
            dis_H = dis_H_arr[0]
            dis_L = dis_L_arr[0]
            
            # Combine bytes to get distance in mm
            distance_mm = (dis_H << 8) | dis_L
            distance_cm = distance_mm / 10  # Convert to cm
            
            # Filter invalid readings (0 or > 400cm)
            if distance_cm == 0 or distance_cm > 400:
                return self.distance
            
            self.distance = distance_cm
            return distance_cm
        
        except Exception as e:
            logger.error(f"Distance measurement error: {e}")
            return self.distance
    
    def get_distance_category(self):
        """Categorize distance"""
        if self.distance < ULTRASONIC_NEAR:
            return "NEAR"
        elif self.distance < ULTRASONIC_MID:
            return "MEDIUM"
        elif self.distance < ULTRASONIC_FAR:
            return "FAR"
        else:
            return "VERY_FAR"
    
    def is_obstacle(self, threshold=200):
        """Check if obstacle detected"""
        return self.distance < threshold / 10
    
    def cleanup(self):
        """Disable ultrasonic sensor"""
        try:
            if self.bot:
                self.bot.Ctrl_Ulatist_Switch(0)
        except:
            pass

_ultrasonic = None

def get_ultrasonic():
    """Get or create ultrasonic sensor instance"""
    global _ultrasonic
    if _ultrasonic is None:
        _ultrasonic = UltrasonicSensor()
    return _ultrasonic
