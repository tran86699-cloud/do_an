#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Buzzer Control via Raspbot_Lib I2C"""

from utils import logger
import time
from Raspbot_Lib import Raspbot

class Buzzer:
    """Buzzer Control via I2C (Raspbot_Lib)"""
    
    def __init__(self):
        self.available = False
        self.bot = None
        
        self._init_buzzer()
    
    def _init_buzzer(self):
        """Initialize buzzer via Raspbot_Lib"""
        try:
            logger.info("Initializing buzzer (I2C via Raspbot_Lib)...")
            
            self.bot = Raspbot()
            
            self.available = True
            logger.info("? Buzzer initialized (I2C)")
        
        except Exception as e:
            logger.error(f"Buzzer init error: {e}")
            self.available = False
    
    def beep(self, duration=0.1, count=1, interval=0.1):
        """Beep buzzer via I2C"""
        if not self.available or not self.bot:
            return
        
        try:
            for i in range(count):
                self.bot.Ctrl_BEEP_Switch(1)  # Turn on
                time.sleep(duration)
                self.bot.Ctrl_BEEP_Switch(0)  # Turn off
                if i < count - 1:
                    time.sleep(interval)
        except Exception as e:
            logger.error(f"Beep error: {e}")
    
    def alarm(self, duration=1, pattern="fast"):
        """Play alarm via I2C"""
        if not self.available or not self.bot:
            return
        
        try:
            if pattern == "fast":
                beep_duration = 0.1
                interval = 0.05
            else:
                beep_duration = 0.2
                interval = 0.2
            
            start_time = time.time()
            while time.time() - start_time < duration:
                self.bot.Ctrl_BEEP_Switch(1)
                time.sleep(beep_duration)
                self.bot.Ctrl_BEEP_Switch(0)
                time.sleep(interval)
        except Exception as e:
            logger.error(f"Alarm error: {e}")
    
    def stop(self):
        """Stop buzzer"""
        if self.bot:
            try:
                self.bot.Ctrl_BEEP_Switch(0)
            except:
                pass
    
    def cleanup(self):
        """Cleanup"""
        self.stop()

_buzzer = None

def get_buzzer():
    """Get or create buzzer instance"""
    global _buzzer
    if _buzzer is None:
        _buzzer = Buzzer()
    return _buzzer
