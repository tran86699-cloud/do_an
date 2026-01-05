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
            self.is_running = False
            logger.info("✓ ObstacleAvoidance initialized")
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
                if self.movement:
                    self.movement.move_forward(speed=speed)
                
                while time.time() - start_time < duration and self.is_running:
                    # Read distance
                    distance = self.ultrasonic.measure_distance()
                    category = self.ultrasonic.get_distance_category()
                    
                    print(f"Distance: {distance:.1f}cm [{category}]", end='\r')
                    
                    # Update display
                    if self.display and self.display.available:
                        self.display.show_frame(
                            "OBSTACLE AVOID",
                            f"Distance: {distance:.1f}cm",
                            f"Status: {category}"
                        )
                    
                    # Check obstacle
                    if distance < OBSTACLE_THRESHOLD / 10:
                        print(f"\n⚠ Obstacle detected at {distance:.1f}cm!")
                        
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
                        
                        print("Continuing forward...")
                        if self.movement:
                            self.movement.move_forward(speed=speed)
                    
                    time.sleep(0.1)
                
                if self.movement:
                    self.movement.stop()
                if self.display:
                    self.display.clear()
                print("\n✓ Obstacle avoidance complete!")
            
            except KeyboardInterrupt:
                print("\n⚠ Stopped")
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


