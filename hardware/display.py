#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OLED Display Module - BALANCED FONT SIZE"""

from utils import logger
import os
import time
from datetime import datetime

class Display:
    """OLED Display - Balanced font size (not too big, not too small)"""
    
    def __init__(self, address=0x3C, contrast=255):
        self.address = address
        self.available = False
        self.device = None
        self.width = 128
        self.height = 64
        self.font_large = None
        self.font_medium = None
        self.font_small = None
        
        self._init_display(address, contrast)
    
    def _init_display(self, address, contrast):
        """Initialize display"""
        try:
            logger.info("Initializing OLED display...")
            
            from luma.core.interface.serial import i2c
            from luma.oled.device import ssd1306
            from PIL import Image, ImageDraw, ImageFont
            
            logger.info(f"Creating I2C interface (port=1, addr=0x{address:02X})...")
            serial = i2c(port=1, address=address)
            logger.info("✓ I2C interface created")
            
            logger.info("Initializing SSD1306 device...")
            self.device = ssd1306(serial)
            self.device.contrast(contrast)
            self.width = self.device.width
            self.height = self.device.height
            logger.info(f"✓ OLED initialized: {self.width}x{self.height}")
            
            self._load_fonts()
            
            self.available = True
            logger.info("✓ Display ready")
            
        except Exception as e:
            logger.error(f"Display init error: {type(e).__name__}: {e}")
            self.available = False
    
    def _load_fonts(self):
        """Load BALANCED fonts - not too big, not too small"""
        try:
            from PIL import ImageFont
            
            # Try different fonts - not monospace
            fonts_to_try = [
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
                "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
            ]
            
            font_path = None
            for path in fonts_to_try:
                if os.path.exists(path):
                    font_path = path
                    break
            
            if font_path:
                logger.info(f"Using font: {font_path}")
                # Balanced sizes: 12, 11, 10
                self.font_large = ImageFont.truetype(font_path, 12)
                self.font_medium = ImageFont.truetype(font_path, 11)
                self.font_small = ImageFont.truetype(font_path, 10)
            else:
                logger.warning("Custom fonts not found, using defaults")
                self.font_large = ImageFont.load_default()
                self.font_medium = ImageFont.load_default()
                self.font_small = ImageFont.load_default()
            
            logger.info("✓ Balanced fonts loaded")
        except Exception as e:
            logger.warning(f"Font loading failed: {e}")
            from PIL import ImageFont
            self.font_large = ImageFont.load_default()
            self.font_medium = ImageFont.load_default()
            self.font_small = ImageFont.load_default()
    
    def get_cpu_usage(self):
        """Get CPU usage"""
        try:
            cpu = os.popen("top -bn1 | grep 'Cpu(s)' | awk '{print $2}' | cut -d'%' -f1").read().strip()
            return cpu if cpu else "0"
        except:
            return "0"
    
    def get_ram_usage(self):
        """Get RAM usage"""
        try:
            cmd = "free | awk 'NR==2{printf \"%d\", 100*($3)/($2)}'"
            return os.popen(cmd).read().strip()
        except:
            return "0"
    
    def get_disk_usage(self):
        """Get Disk usage"""
        try:
            cmd = "df -h / | awk 'NR==2{printf \"%s\", $5}'"
            return os.popen(cmd).read().strip()
        except:
            return "0%"
    
    def get_ip_address(self):
        """Get IP address"""
        try:
            ip = os.popen("hostname -I").read().strip().split()[0]
            return ip
        except:
            return "x.x.x.x"
    
    def get_temperature(self):
        """Get CPU temperature"""
        try:
            temp = os.popen("vcgencmd measure_temp | cut -d'=' -f2 | cut -d\"'\" -f1").read().strip()
            return temp if temp else "N/A"
        except:
            return "N/A"
    
    def get_time(self):
        """Get current time"""
        return datetime.now().strftime("%H:%M:%S")
    
    def show_system_info(self):
        """Display system information - BALANCED SIZE"""
        if not self.available or self.device is None:
            logger.warning("Display not available")
            return
        
        try:
            from PIL import Image, ImageDraw
            
            image = Image.new('1', (self.width, self.height))
            draw = ImageDraw.Draw(image)
            
            cpu = self.get_cpu_usage()
            ram = self.get_ram_usage()
            disk = self.get_disk_usage()
            ip = self.get_ip_address()
            current_time = self.get_time()
            
            y = 2
            
            # More compact - show more info
            draw.text((2, y), "CPU: " + cpu + "%", font=self.font_medium, fill=1)
            y += 11
            
            draw.text((2, y), "RAM: " + ram + "%", font=self.font_medium, fill=1)
            y += 11
            
            draw.text((2, y), "DISK: " + disk, font=self.font_medium, fill=1)
            y += 11
            
            draw.text((2, y), "TIME: " + current_time, font=self.font_small, fill=1)
            y += 10
            
            draw.text((2, y), "IP: " + ip, font=self.font_small, fill=1)
            
            self.device.display(image)
            
        except Exception as e:
            logger.error(f"show_system_info error: {e}")
    
    def show_system_monitor(self, duration=30):
        """System monitor - BALANCED SIZE"""
        if not self.available or self.device is None:
            logger.warning("Display not available")
            return
        
        try:
            from PIL import Image, ImageDraw
            
            start_time = time.time()
            update_count = 0
            
            while time.time() - start_time < duration:
                update_count += 1
                
                image = Image.new('1', (self.width, self.height))
                draw = ImageDraw.Draw(image)
                
                cpu = self.get_cpu_usage()
                ram = self.get_ram_usage()
                disk = self.get_disk_usage()
                temp = self.get_temperature()
                current_time = self.get_time()
                
                y = 2
                
                # CPU
                draw.text((2, y), "CPU: " + cpu + "%", font=self.font_large, fill=1)
                y += 12
                
                # RAM
                draw.text((2, y), "RAM: " + ram + "%", font=self.font_large, fill=1)
                y += 12
                
                # DISK
                draw.text((2, y), "DISK: " + disk, font=self.font_medium, fill=1)
                y += 11
                
                # TEMP
                draw.text((2, y), "TEMP: " + temp, font=self.font_medium, fill=1)
                y += 11
                
                # Footer: Time and counter
                footer = current_time + " [" + str(update_count) + "s]"
                draw.text((2, 52), footer, font=self.font_small, fill=1)
                
                self.device.display(image)
                time.sleep(1)
        
        except Exception as e:
            logger.error(f"show_system_monitor error: {e}")
    
    def show_frame(self, *lines):
        """Display custom frame"""
        if not self.available or self.device is None:
            logger.warning("Display not available")
            return
        
        try:
            from PIL import Image, ImageDraw
            
            image = Image.new('1', (self.width, self.height))
            draw = ImageDraw.Draw(image)
            
            y_pos = 6
            line_spacing = 13
            max_lines = 5  # Can show more lines now
            
            for i, line in enumerate(lines[:max_lines]):
                if line and y_pos < self.height:
                    draw.text((2, y_pos), str(line), font=self.font_medium, fill=1)
                    y_pos += line_spacing
            
            self.device.display(image)
            
        except Exception as e:
            logger.error(f"show_frame error: {e}")
    
    def show_message(self, title="", message="", subtitle=""):
        """Show message"""
        if not self.available or self.device is None:
            return
        
        try:
            from PIL import Image, ImageDraw
            
            image = Image.new('1', (self.width, self.height))
            draw = ImageDraw.Draw(image)
            
            y = 6
            
            if title:
                draw.text((2, y), title, font=self.font_large, fill=1)
                y += 13
            
            if message:
                draw.text((2, y), message, font=self.font_medium, fill=1)
                y += 13
            
            if subtitle:
                draw.text((2, y), subtitle, font=self.font_small, fill=1)
            
            self.device.display(image)
            
        except Exception as e:
            logger.error(f"show_message error: {e}")
    
    def clear(self):
        """Clear display"""
        if not self.available or self.device is None:
            return
        
        try:
            from PIL import Image
            blank = Image.new('1', (self.width, self.height))
            self.device.display(blank)
        except Exception as e:
            logger.error(f"clear error: {e}")
    
    def test(self):
        """Test display"""
        if not self.available:
            logger.error("Display not available for testing")
            print("✗ OLED Display not initialized")
            return
        
        logger.info("Running display test...")
        
        try:
            print("Test 1: System information (balanced size)...")
            self.show_system_info()
            time.sleep(2)
            
            print("Test 2: Custom frame (5 lines)...")
            self.show_frame("RASPBOT V2", "Display OK", "Font Size", "Balanced", "Perfect!")
            time.sleep(2)
            
            print("Test 3: Message...")
            self.show_message(
                title="SYSTEM OK",
                message="Font Balanced",
                subtitle="Easy to Read"
            )
            time.sleep(2)
            
            self.clear()
            print("✓ Display test complete!")
            
        except Exception as e:
            logger.error(f"Display test error: {e}")

_display = None

def get_display():
    """Get or create display instance"""
    global _display
    if _display is None:
        _display = Display(contrast=255)
    return _display

