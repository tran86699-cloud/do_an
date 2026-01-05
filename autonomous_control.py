#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Autonomous Control - Obstacle Avoidance, Line Following, Patrol
"""

from hardware.movement import get_movement
from hardware.ultrasonic import get_ultrasonic
from hardware.line_tracking import get_line_tracking
from hardware.buzzer import get_buzzer
from utils import logger
import time

class AutonomousController:
    """Điều khiển robot tự động"""
    
    def __init__(self):
        self.movement = get_movement()
        self.ultrasonic = get_ultrasonic()
        self.line_tracking = get_line_tracking()
        self.buzzer = get_buzzer()
        self.is_running = False
    
    # ====================================================================
    # OBSTACLE AVOIDANCE - Tránh Chướng Ngại Vật
    # ====================================================================
    
    def obstacle_avoidance(self, duration=60, speed=150):
        """
        Tránh chướng ngại vật tự động
        
        1. Di chuyển phía trước
        2. Nếu phát hiện chướng ngại vật < 20cm:
           - Dừng lại
           - Quay phải 90 độ
           - Tiếp tục di chuyển
        """
        logger.info(f"🚫 Bắt đầu Tránh Chướng Ngại Vật ({duration}s)...")
        self.is_running = True
        start_time = time.time()
        
        try:
            self.movement.move_forward(speed=speed)
            
            while time.time() - start_time < duration and self.is_running:
                distance = self.ultrasonic.measure_distance()
                
                print(f"Khoảng cách: {distance:.1f}cm", end='\r')
                
                # Nếu quá gần
                if distance < 20:
                    print(f"\n🚫 Phát hiện chướng ngại vật! {distance:.1f}cm")
                    
                    # Dừng
                    self.movement.stop()
                    self.buzzer.beep(duration=0.2, count=2)
                    time.sleep(0.5)
                    
                    # Lùi lại
                    print("⬅️ Đang lùi...")
                    self.movement.move_backward(speed=speed, duration=1)
                    time.sleep(0.3)
                    
                    # Quay phải
                    print("🔄 Quay phải...")
                    self.movement.rotate_right(speed=120, duration=1)
                    time.sleep(0.3)
                    
                    # Tiếp tục
                    print("➡️ Tiếp tục di chuyển...")
                    self.movement.move_forward(speed=speed)
                
                time.sleep(0.1)
            
            self.movement.stop()
            self.buzzer.beep(duration=0.2, count=1)
            logger.info("✅ Hoàn tất Tránh Chướng Ngại Vật")
            
        except Exception as e:
            logger.error(f"❌ Lỗi Tránh Chướng Ngại Vật: {e}")
            self.movement.stop()
        
        finally:
            self.is_running = False
    
    # ====================================================================
    # LINE FOLLOWING - Theo Dõi Đường
    # ====================================================================
    
    def line_following(self, duration=60, speed=100):
        """
        Theo dõi đường gạch tự động
        
        1. Đọc 4 cảm biến đường
        2. Dựa vào vị trí của đường:
           - Nếu ở giữa: di chuyển phía trước
           - Nếu bên trái: quay trái
           - Nếu bên phải: quay phải
           - Nếu mất đường: dừng lại
        """
        logger.info(f"📍 Bắt đầu Theo Dõi Đường ({duration}s)...")
        self.is_running = True
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration and self.is_running:
                state = self.line_tracking.read_state()
                direction = self.line_tracking.get_direction()
                
                print(f"Đường: {state} | {direction}", end='\r')
                
                # Điều khiển dựa vào hướng
                if direction == "ALL_ON_LINE":
                    self.movement.move_forward(speed=speed, duration=0.1)
                    print("➡️ Tiến tới")
                
                elif direction == "LEFT":
                    self.movement.turn_left(speed=speed, duration=0.1)
                    print("⬅️ Quay trái")
                
                elif direction == "RIGHT":
                    self.movement.turn_right(speed=speed, duration=0.1)
                    print("➡️ Quay phải")
                
                elif direction == "DIAGONAL_LEFT":
                    self.movement.rotate_left(speed=80, duration=0.1)
                    print("↙️ Điều chỉnh trái")
                
                elif direction == "DIAGONAL_RIGHT":
                    self.movement.rotate_right(speed=80, duration=0.1)
                    print("↘️ Điều chỉnh phải")
                
                elif direction == "OFF_LINE":
                    self.movement.stop()
                    self.buzzer.beep(duration=0.1, count=1)
                    print("⚠️ Mất đường!")
                
                else:
                    self.movement.move_forward(speed=speed, duration=0.1)
                
                time.sleep(0.05)
            
            self.movement.stop()
            self.buzzer.beep(duration=0.2, count=1)
            logger.info("✅ Hoàn tất Theo Dõi Đường")
            
        except Exception as e:
            logger.error(f"❌ Lỗi Theo Dõi Đường: {e}")
            self.movement.stop()
        
        finally:
            self.is_running = False
    
    # ====================================================================
    # PATROL - Tuần Tra
    # ====================================================================
    
    def patrol(self, duration=60, speed=150):
        """
        Tuần tra tự động - Hình vuông
        
        1. Di chuyển phía trước (10s)
        2. Quay phải 90 độ (1s)
        3. Lặp lại để tạo hình vuông
        """
        logger.info(f"🔄 Bắt đầu Tuần Tra ({duration}s)...")
        self.is_running = True
        start_time = time.time()
        
        try:
            side_duration = 10
            
            while time.time() - start_time < duration and self.is_running:
                for side in range(4):
                    if time.time() - start_time >= duration or not self.is_running:
                        break
                    
                    # Tiến tới
                    print(f"Cạnh {side+1}: Tiến tới...")
                    side_start = time.time()
                    
                    self.movement.move_forward(speed=speed)
                    
                    while time.time() - side_start < side_duration:
                        if not self.is_running:
                            break
                        
                        distance = self.ultrasonic.measure_distance()
                        
                        # Nếu gặp chướng ngại vật
                        if distance < 30:
                            print(f"⚠️ Phát hiện chướng ngại vật! Lùi lại...")
                            self.movement.stop()
                            self.movement.move_backward(speed=speed, duration=1)
                            time.sleep(0.5)
                            break
                        
                        time.sleep(0.1)
                    
                    if not self.is_running:
                        break
                    
                    # Quay phải
                    print(f"Quay phải (90°)...")
                    self.movement.stop()
                    time.sleep(0.3)
                    
                    self.movement.rotate_right(speed=120, duration=1)
                    time.sleep(0.3)
                    
                    elapsed = int(time.time() - start_time)
                    print(f"Tiến độ: {elapsed}s/{duration}s")
            
            self.movement.stop()
            self.buzzer.beep(duration=0.2, count=1)
            logger.info("✅ Hoàn tất Tuần Tra")
            
        except Exception as e:
            logger.error(f"❌ Lỗi Tuần Tra: {e}")
            self.movement.stop()
        
        finally:
            self.is_running = False
    
    # ====================================================================
    # STOP
    # ====================================================================
    
    def stop(self):
        """Dừng chế độ tự động"""
        self.is_running = False
        self.movement.stop()
        logger.info("⏹ Chế độ tự động đã dừng")


# Global instance
_autonomous = None

def get_autonomous_controller():
    """Lấy hoặc tạo autonomous controller"""
    global _autonomous
    if _autonomous is None:
        _autonomous = AutonomousController()
    return _autonomous


