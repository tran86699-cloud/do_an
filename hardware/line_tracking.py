#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Line Tracking Sensor via I2C Register 0x0a - CORRECT BIT ORDER"""

from utils import logger
from Raspbot_Lib import Raspbot

class LineTrackingSensor:
    """4-way Line Tracking via I2C - Correct hardware mapping"""
    
    def __init__(self):
        self.available = False
        self.state = [1, 1, 1, 1]  # [SW2, SW1, SW3, SW4]
        self.bot = None
        self._init_sensor()
    
    def _init_sensor(self):
        try:
            logger.info("Init line tracking (I2C 0x0a - HW order: SW2,SW1,SW3,SW4)...")
            self.bot = Raspbot()
            self.available = True
            logger.info("✓ Line tracking OK")
        except Exception as e:
            logger.error(f"Line tracking error: {e}")
            self.available = False
    
    def read_state(self):
        """Read from I2C reg 0x0a
        
        Bit order: [bit3, bit2, bit1, bit0] = [SW2, SW1, SW3, SW4]
        Position:  [L1,   L2,   R1,   R2]
        
        0 = line detected (black)
        1 = no line (white)
        """
        if not self.available or not self.bot:
            return [1, 1, 1, 1]
        
        try:
            track_arr = self.bot.read_data_array(0x0a, 1)
            if not track_arr:
                return [1, 1, 1, 1]
            
            track = track_arr[0]
            
            # Extract bits in CORRECT order matching hardware
            # bit3 = SW2 (L1, left front)
            # bit2 = SW1 (L2, left back)
            # bit1 = SW3 (R1, right front)
            # bit0 = SW4 (R2, right back)
            
            L1_SW2 = (track >> 3) & 0x01  # bit3
            L2_SW1 = (track >> 2) & 0x01  # bit2
            R1_SW3 = (track >> 1) & 0x01  # bit1
            R2_SW4 = (track >> 0) & 0x01  # bit0
            
            self.state = [L1_SW2, L2_SW1, R1_SW3, R2_SW4]
            return self.state
        except Exception as e:
            logger.error(f"Read error: {e}")
            return [1, 1, 1, 1]
    
    def is_on_line(self):
        """Check if any sensor detects line"""
        state = self.read_state()
        return any(s == 0 for s in state)
    
    def get_direction(self):
        """Get line direction
        
        Layout:
            L1(SW2)  L2(SW1)    R1(SW3)  R2(SW4)
            [left]              [right]
        """
        state = self.read_state()
        L1, L2, R1, R2 = state
        
        # All on line
        if L1 == 0 and L2 == 0 and R1 == 0 and R2 == 0:
            return "ALL_ON_LINE"
        
        # Left side detects line
        elif L1 == 0 or L2 == 0:
            return "LEFT"
        
        # Right side detects line
        elif R1 == 0 or R2 == 0:
            return "RIGHT"
        
        # Diagonal left (L1 + R2)
        elif L1 == 0 and R2 == 0:
            return "DIAGONAL_LEFT"
        
        # Diagonal right (L2 + R1)
        elif L2 == 0 and R1 == 0:
            return "DIAGONAL_RIGHT"
        
        # No line detected
        else:
            return "OFF_LINE"
    
    def cleanup(self):
        pass

_line_tracking = None

def get_line_tracking():
    global _line_tracking
    if _line_tracking is None:
        _line_tracking = LineTrackingSensor()
    return _line_tracking
