#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LED Control via Raspbot_Lib I2C - WS2812 LEDs"""

from utils import logger
import time
from Raspbot_Lib import Raspbot

class LED:
    """WS2812 LED Control via I2C (Raspbot_Lib)"""
    
    def __init__(self):
        self.available = False
        self.bot = None
        self.current_color = (0, 0, 0)
        
        self._init_led()
    
    def _init_led(self):
        """Initialize LED via Raspbot_Lib"""
        try:
            logger.info("Initializing LED (WS2812 via Raspbot_Lib)...")
            
            self.bot = Raspbot()
            
            # Turn off LED initially
            self.turn_off()
            
            self.available = True
            logger.info("âœ“ LED initialized (WS2812)")
        
        except Exception as e:
            logger.error(f"LED init error: {e}")
            self.available = False
    
    def set_color(self, color_name):
        """Set LED color by name"""
        if not self.available or not self.bot:
            logger.debug(f"LED not available, skipping color: {color_name}")
            return
        
        # Color mapping for WS2812
        colors = {
            'RED': (255, 0, 0),
            'GREEN': (0, 255, 0),
            'BLUE': (0, 0, 255),
            'YELLOW': (255, 255, 0),
            'PURPLE': (255, 0, 255),
            'CYAN': (0, 255, 255),
            'WHITE': (255, 255, 255),
            'ORANGE': (255, 165, 0),
            'PINK': (255, 105, 180),
            'OFF': (0, 0, 0)
        }
        
        rgb = colors.get(color_name.upper(), (0, 0, 0))
        self.set_rgb(*rgb)
    
    def set_rgb(self, r, g, b):
        """Set LED with RGB values (0-255)"""
        if not self.available or not self.bot:
            return
        
        try:
            # Clamp values
            r = max(0, min(255, int(r)))
            g = max(0, min(255, int(g)))
            b = max(0, min(255, int(b)))
            
            # Control WS2812 LED via I2C with RGB brightness
            self.bot.Ctrl_WQ2812_brightness_ALL(r, g, b)
            
            self.current_color = (r, g, b)
            logger.debug(f"LED: RGB({r},{g},{b})")
        
        except Exception as e:
            logger.error(f"LED set RGB error: {e}")
    
    def turn_off(self):
        """Turn off LED"""
        self.set_rgb(0, 0, 0)
    
    def blink(self, color='WHITE', times=3, duration=0.2):
        """Blink LED"""
        if not self.available or not self.bot:
            return
        
        try:
            for i in range(times):
                self.set_color(color)
                time.sleep(duration)
                self.turn_off()
                time.sleep(duration)
        except Exception as e:
            logger.error(f"LED blink error: {e}")
    
    def pulse(self, color='BLUE', duration=2.0):
        """Pulse LED (fade in/out)"""
        if not self.available or not self.bot:
            return
        
        try:
            colors = {
                'RED': (255, 0, 0),
                'GREEN': (0, 255, 0),
                'BLUE': (0, 0, 255),
                'YELLOW': (255, 255, 0),
                'PURPLE': (255, 0, 255),
                'CYAN': (0, 255, 255),
                'WHITE': (255, 255, 255)
            }
            
            base_rgb = colors.get(color.upper(), (0, 0, 255))
            steps = 20
            step_time = duration / (steps * 2)
            
            # Fade in
            for i in range(steps + 1):
                factor = i / steps
                r = int(base_rgb[0] * factor)
                g = int(base_rgb[1] * factor)
                b = int(base_rgb[2] * factor)
                self.set_rgb(r, g, b)
                time.sleep(step_time)
            
            # Fade out
            for i in range(steps, -1, -1):
                factor = i / steps
                r = int(base_rgb[0] * factor)
                g = int(base_rgb[1] * factor)
                b = int(base_rgb[2] * factor)
                self.set_rgb(r, g, b)
                time.sleep(step_time)
            
        except Exception as e:
            logger.error(f"LED pulse error: {e}")
    
    def cleanup(self):
        """Cleanup - turn off LED"""
        self.turn_off()

# ==============================
# GLOBAL INSTANCE
# ==============================

_led = None

def get_led():
    """Get or create LED instance"""
    global _led
    if _led is None:
        _led = LED()
    return _led


