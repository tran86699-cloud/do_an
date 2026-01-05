#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Line Following - Auto follow line with PID"""

import time
import logging

logger = logging.getLogger('raspbot')

try:
    from hardware.line_tracking import get_line_tracking
    from hardware.movement import get_movement
    from hardware.buzzer import get_buzzer
    from hardware.display import get_display
    from hardware.pid_controller import LinePID  
except Exception as e:
    logger.warning(f"Hardware import error in line_following: {e}")

class LineFollowing:
    """Line following controller with PID"""
    
    def __init__(self):
        try:
            self.line_tracking = get_line_tracking()
            self.movement = get_movement()
            self.buzzer = get_buzzer()
            self.display = get_display()
            self.is_running = False
            
            # Initialize PID controller
            try:
                self.pid = LinePID(
                    kp=50.0,       
                    ki=0.5,
                    kd=15.0,
                    base_speed=100
                )
                logger.info("âœ“ PID controller initialized")
            except Exception as pid_error:
                logger.warning(f"PID init error: {pid_error}, using simple control")
                self.pid = None
            
            logger.info("âœ“ LineFollowing initialized")
        except Exception as e:
            logger.error(f"LineFollowing init error: {e}")
            self.line_tracking = None
            self.movement = None
            self.buzzer = None
            self.display = None
            self.pid = None
            self.is_running = False
    
    def _read_sensors_array(self):
        """
        Read line sensors as array [L2, L1, R1, R2]
        where 1 = black line, 0 = white/off line
        """
        try:
            state = self.line_tracking.read_state()
            
            # Convert state to array
            sensors = [
                1 if state & 0b1000 else 0,  # L2 (leftmost)
                1 if state & 0b0100 else 0,  # L1
                1 if state & 0b0010 else 0,  # R1
                1 if state & 0b0001 else 0   # R2 (rightmost)
            ]
            
            return sensors
            
        except Exception as e:
            logger.error(f"Sensor read error: {e}")
            return [0, 0, 0, 0]
    
    def start(self, duration=60, speed=100):
        """Start line following with PID or fallback to simple control"""
        try:
            logger.info(f"Starting line following ({duration}s)...")
            
            if not self.line_tracking or not self.line_tracking.available:
                logger.error("Line tracking sensor not available")
                return
            
            self.is_running = True
            start_time = time.time()
            
            # Print mode
            if self.pid:
                self.pid.base_speed = speed
                self.pid.reset()
                print(f"\nâ­ PID Line Following - Speed {speed} (Press Ctrl+C to stop)")
            else:
                print(f"\nLine Following - Simple Mode (Press Ctrl+C to stop)")
            
            print("Robot following line...\n")
            
            try:
                while time.time() - start_time < duration and self.is_running:
                    
                    # ===== PID CONTROL PATH =====
                    if self.pid:
                        # Read sensors
                        sensors = self._read_sensors_array()
                        
                        # Check if line is lost (all sensors white)
                        if all(s == 0 for s in sensors):
                            if self.movement:
                                self.movement.stop()
                            if self.buzzer:
                                self.buzzer.beep(duration=0.2, count=2)
                            print("\nâš  Line lost! (all sensors white)")
                            time.sleep(0.5)
                            continue
                        
                        # Compute motor speeds using PID
                        left_speed, right_speed = self.pid.compute_line_following(sensors)
                        
                        # Apply motor speeds
                        if self.movement:
                            # Calculate average speed and speed difference
                            avg_speed = (left_speed + right_speed) // 2
                            speed_diff = left_speed - right_speed
                            
                            if abs(speed_diff) < 10:
                                # Going straight
                                self.movement.move_forward(speed=avg_speed, duration=0.05)
                            elif speed_diff > 0:
                                # Turn right (left motor faster)
                                self.movement.turn_right(speed=abs(speed_diff), duration=0.05)
                            else:
                                # Turn left (right motor faster)
                                self.movement.turn_left(speed=abs(speed_diff), duration=0.05)
                        
                        # Update display
                        if self.display and self.display.available:
                            self.display.show_frame(
                                "PID LINE",
                                f"L:{left_speed:3d} R:{right_speed:3d}",
                                f"Sen:{sensors}"
                            )
                        
                        # Print status
                        print(f"Sensors:{sensors} L:{left_speed:3d} R:{right_speed:3d}", end='\r')
                    
                    # ===== FALLBACK: Original simple control =====
                    else:
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
                        
                        # Original control logic
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
                            print("\nâš  Line lost!")
                        else:
                            if self.movement:
                                self.movement.move_forward(speed=speed, duration=0.1)
                    
                    time.sleep(0.05)
                
                # Cleanup
                if self.movement:
                    self.movement.stop()
                if self.display:
                    self.display.clear()
                print("\nâœ“ Line following complete!")
            
            except KeyboardInterrupt:
                print("\nâš  Stopped")
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

