#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Line Following - Auto follow line"""

import time
import logging

logger = logging.getLogger('raspbot')

try:
    from hardware.line_tracking import get_line_tracking
    from hardware.movement import get_movement
    from hardware.buzzer import get_buzzer
    from hardware.display import get_display
except Exception as e:
    logger.warning(f"Hardware import error in line_following: {e}")

class LineFollowing:
    """Line following controller"""
    
    def __init__(self):
        try:
            self.line_tracking = get_line_tracking()
            self.movement = get_movement()
            self.buzzer = get_buzzer()
            self.display = get_display()
            self.is_running = False
            logger.info("✓ LineFollowing initialized")
        except Exception as e:
            logger.error(f"LineFollowing init error: {e}")
            self.line_tracking = None
            self.movement = None
            self.buzzer = None
            self.display = None
            self.is_running = False
    
    def start(self, duration=60, speed=100):
        """Start line following"""
        try:
            logger.info(f"Starting line following ({duration}s)...")
            
            if not self.line_tracking or not self.line_tracking.available:
                logger.error("Line tracking sensor not available")
                return
            
            self.is_running = True
            start_time = time.time()
            
            print(f"\nLine Following (Press Ctrl+C to stop)")
            print("Robot following line...\n")
            
            try:
                while time.time() - start_time < duration and self.is_running:
                    # Read line state
                    state = self.line_tracking.read_state()
                    direction = self.line_tracking.get_direction()
                    
                    print(f"State: {state} | Direction: {direction}", end='\r')
                    
                    # Update display
                    if self.display and self.display.available:
                        self.display.show_frame(
                            "LINE FOLLOW",
                            f"Direction: {direction}",
                            f"State: {state}"
                        )
                    
                    # Control based on line position
                    if direction == "ALL_ON_LINE" and self.movement:
                        self.movement.move_forward(speed=speed, duration=0.1)
                    elif direction == "LEFT" and self.movement:
                        self.movement.turn_left(speed=speed, duration=0.1)
                    elif direction == "RIGHT" and self.movement:
                        self.movement.turn_right(speed=speed, duration=0.1)
                    elif direction == "OFF_LINE":
                        if self.movement:
                            self.movement.stop()
                        if self.buzzer:
                            self.buzzer.beep(duration=0.1, count=1)
                        print("\n⚠ Line lost!")
                    else:
                        if self.movement:
                            self.movement.move_forward(speed=speed, duration=0.1)
                    
                    time.sleep(0.05)
                
                if self.movement:
                    self.movement.stop()
                if self.display:
                    self.display.clear()
                print("\n✓ Line following complete!")
            
            except KeyboardInterrupt:
                print("\n⚠ Stopped")
                if self.movement:
                    self.movement.stop()
                if self.display:
                    self.display.clear()
            
            except Exception as e:
                logger.error(f"Line following run error: {e}")
                if self.movement:
                    self.movement.stop()
        
        except Exception as e:
            logger.error(f"Line following error: {e}")
        
        finally:
            self.is_running = False

_line_following = None

def get_line_following():
    """Get line following instance"""
    global _line_following
    if _line_following is None:
        try:
            _line_following = LineFollowing()
        except Exception as e:
            logger.error(f"Failed to create LineFollowing: {e}")
            _line_following = None
    return _line_following


