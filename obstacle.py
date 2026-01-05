#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Obstacle Avoidance - Auto avoid obstacles"""

import time

# Simple logger fallback
import logging
logger = logging.getLogger('raspbot')

try:
    from hardware.ultrasonic import get_ultrasonic
    from hardware.movement import get_movement
    from hardware.buzzer import get_buzzer
    from hardware.display import get_display
    from hardware.pid_controller import ObstaclePID
except Exception as e:
    logger.warning(f"Hardware import error in obstacle_avoidance: {e}")

try:
    from config.sensor_config import SAFE_DISTANCE, OBSTACLE_THRESHOLD
except:
    SAFE_DISTANCE = 250
    OBSTACLE_THRESHOLD = 200

class ObstacleAvoidance:
    """Obstacle avoidance controller"""
    
    def __init__(self):
        try:
            self.ultrasonic = get_ultrasonic()
            self.movement = get_movement()
            self.buzzer = get_buzzer()
            self.display = get_display()
            self.obstacle_pid = ObstaclePID(safe_distance=50, stop_distance=15)
            self.is_running = False
            logger.info("âœ“ ObstacleAvoidance initialized")
        except Exception as e:
            logger.error(f"ObstacleAvoidance init error: {e}")
            self.ultrasonic = None
            self.movement = None
            self.buzzer = None
            self.display = None
            self.is_running = False
    
    def start(self, duration=60, speed=150):
        """Start obstacle avoidance"""
        try:
            logger.info(f"Starting obstacle avoidance ({duration}s)...")
            
            if not self.ultrasonic or not self.ultrasonic.available:
                logger.error("Ultrasonic sensor not available")
                return
            
            self.is_running = True
            start_time = time.time()
            
            print(f"\nObstacle Avoidance (Press Ctrl+C to stop)")
            print("Robot moving forward, avoiding obstacles...\n")
            
            try:

                
                while time.time() - start_time < duration and self.is_running:
                    # Read distance
                    distance = self.ultrasonic.measure_distance()
                    category = self.ultrasonic.get_distance_category()
                    
                    # ? NEW: Compute safe speed using PID
                    if self.obstacle_pid:
                        safe_speed = self.obstacle_pid.compute_safe_speed(
                            distance, 
                            desired_speed=speed  # Use speed parameter (default 150)
                        )
                    else:
                        # Fallback if PID failed
                        safe_speed = speed if distance >= 20 else 0
                    
                    # Print status
                    print(f"Distance: {distance:.1f}cm | Speed: {safe_speed:3d} [{category}]", end='\r')
                    
                    # Update display
                    if self.display and self.display.available:
                        self.display.show_frame(
                            "PID OBSTACLE",
                            f"Dist: {distance:.1f}cm",
                            f"Speed: {safe_speed:3d}"
                        )
                    
                    # ? NEW: Apply PID-computed speed
                    if safe_speed > 0:
                        # Safe to move - use PID speed
                        if self.movement:
                            self.movement.move_forward(speed=safe_speed, duration=0.1)
                    
                    else:
                        # Distance < 15cm - stop and avoid
                        print(f"\n? Obstacle at {distance:.1f}cm - avoiding...")
                        
                        if self.movement:
                            self.movement.stop()
                        
                        if self.buzzer:
                            self.buzzer.beep(duration=0.2, count=2)
                        
                        print("Backing up...")
                        if self.movement:
                            self.movement.move_backward(speed=speed, duration=1)
                        time.sleep(0.5)
                        
                        print("Turning right...")
                        if self.movement:
                            self.movement.rotate_right(speed=120, duration=1)
                        time.sleep(0.5)
                        
                        print("Continuing...")
                    
                    time.sleep(0.1)
                
                if self.movement:
                    self.movement.stop()
                if self.display:
                    self.display.clear()
                print("\nâœ“ Obstacle avoidance complete!")
            
            except KeyboardInterrupt:
                print("\nâš  Stopped")
                if self.movement:
                    self.movement.stop()
                if self.display:
                    self.display.clear()
            
            except Exception as e:
                logger.error(f"Obstacle avoidance run error: {e}")
                if self.movement:
                    self.movement.stop()
        
        except Exception as e:
            logger.error(f"Obstacle avoidance error: {e}")
        
        finally:
            self.is_running = False

_obstacle_avoidance = None

def get_obstacle_avoidance():
    """Get obstacle avoidance instance"""
    global _obstacle_avoidance
    if _obstacle_avoidance is None:
        try:
            _obstacle_avoidance = ObstacleAvoidance()
        except Exception as e:
            logger.error(f"Failed to create ObstacleAvoidance: {e}")
            _obstacle_avoidance = None
    return _obstacle_avoidance


