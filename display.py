#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OLED Display Module - PERFECT FONT: DejaVuSansMono size 8"""

from utils import logger
import os
import time
from datetime import datetime

class Display:
    """OLED Display - Using DejaVuSansMono font size 8 (verified working)"""
    
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
            logger.info("âœ“ I2C interface created")
            
            logger.info("Initializing SSD1306 device...")
            self.device = ssd1306(serial)
            self.device.contrast(contrast)
            self.width = self.device.width
            self.height = self.device.height
            logger.info(f"âœ“ OLED initialized: {self.width}x{self.height}")
            
            self._load_fonts()
            
            self.available = True
            logger.info("âœ“ Display ready")
            
        except Exception as e:
            logger.error(f"Display init error: {type(e).__name__}: {e}")
            self.available = False
    
    def _load_fonts(self):
        """Load fonts - USING DejaVuSansMono size 8 (user verified)"""
        try:
            from PIL import ImageFont
            
            # USER VERIFIED: DejaVuSansMono.ttf size 8 works perfectly!
            font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
            
            if os.path.exists(font_path):
                logger.info(f"Using font: {font_path}")
                # All use size 8 - verified working by user
                self.font_large = ImageFont.truetype(font_path, 12)
                self.font_medium = ImageFont.truetype(font_path, 12)
                self.font_small = ImageFont.truetype(font_path, 12)
            else:
                logger.warning("DejaVuSansMono not found, trying alternatives")
                # Fallback
                fonts_to_try = [
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                    "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
                ]
                for path in fonts_to_try:
                    if os.path.exists(path):
                        font_path = path
                        self.font_large = ImageFont.truetype(font_path, 8)
                        self.font_medium = ImageFont.truetype(font_path, 8)
                        self.font_small = ImageFont.truetype(font_path, 8)
                        break
            
            logger.info("âœ“ Fonts loaded (DejaVuSansMono-8)")
        except Exception as e:
            logger.warning(f"Font loading failed: {e}")
            from PIL import ImageFont
            self.font_large = ImageFont.load_default()
            self.font_medium = ImageFont.load_default()
            self.font_small = ImageFont.load_default()
    
    def show_frame(self, *lines):
        """Display custom frame"""
        if not self.available or self.device is None:
            logger.warning("Display not available")
            return
        
        try:
            from PIL import Image, ImageDraw
            
            image = Image.new('1', (self.width, self.height))
            draw = ImageDraw.Draw(image)
            
            # Y position for size 8 font
            y_pos = 6
            line_spacing = 12  # Good spacing for size 8
            max_lines = 5  # Can fit 5 lines
            
            for i, line in enumerate(lines[:max_lines]):
                if line and y_pos < self.height:
                    text = str(line)
                    
                    # Size 8 can fit ~21 chars
                    if len(text) > 21:
                        text = text[:21]
                    
                    draw.text((2, y_pos), text, font=self.font_medium, fill=1)
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
                title_text = str(title)[:21]
                draw.text((2, y), title_text, font=self.font_large, fill=1)
                y += 12
            
            if message:
                message_text = str(message)[:21]
                draw.text((2, y), message_text, font=self.font_medium, fill=1)
                y += 12
            
            if subtitle:
                subtitle_text = str(subtitle)[:21]
                draw.text((2, y), subtitle_text, font=self.font_small, fill=1)
            
            self.device.display(image)
            
        except Exception as e:
            logger.error(f"show_message error: {e}")
    
    def show_startup_info(self, battery_percent, ip_address):
        """Show startup info"""
        if not self.available or self.device is None:
            return
        
        try:
            from PIL import Image, ImageDraw
            
            image = Image.new('1', (self.width, self.height))
            draw = ImageDraw.Draw(image)
            
            y = 6
            
            # Title
            draw.text((2, y), "RASPBOT V2", font=self.font_large, fill=1)
            y += 12
            
            # Battery
            battery_text = f"Bat: {battery_percent}%"
            draw.text((2, y), battery_text, font=self.font_medium, fill=1)
            y += 12
            
            # IP
            draw.text((2, y), str(ip_address), font=self.font_medium, fill=1)
            y += 12
            
            # Ready
            draw.text((2, y), "Ready!", font=self.font_small, fill=1)
            
            self.device.display(image)
            
        except Exception as e:
            logger.error(f"show_startup_info error: {e}")
    
    def show_system_info(self):
        """Display system information"""
        if not self.available or self.device is None:
            return
        
        try:
            from PIL import Image, ImageDraw
            
            image = Image.new('1', (self.width, self.height))
            draw = ImageDraw.Draw(image)
            
            cpu = self.get_cpu_usage()
            ram = self.get_ram_usage()
            disk = self.get_disk_usage()
            current_time = self.get_time()
            
            y = 6
            
            draw.text((2, y), f"CPU: {cpu}%", font=self.font_medium, fill=1)
            y += 12
            
            draw.text((2, y), f"RAM: {ram}%", font=self.font_medium, fill=1)
            y += 12
            
            draw.text((2, y), f"Disk: {disk}", font=self.font_medium, fill=1)
            y += 12
            
            draw.text((2, y), current_time, font=self.font_small, fill=1)
            
            self.device.display(image)
            
        except Exception as e:
            logger.error(f"show_system_info error: {e}")
    
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
    
    # Helper methods
    def get_cpu_usage(self):
        try:
            cpu = os.popen("top -bn1 | grep 'Cpu(s)' | awk '{print $2}' | cut -d'%' -f1").read().strip()
            return cpu if cpu else "0"
        except:
            return "0"
    
    def get_ram_usage(self):
        try:
            cmd = "free | awk 'NR==2{printf \"%d\", 100*($3)/($2)}'"
            return os.popen(cmd).read().strip()
        except:
            return "0"
    
    def get_disk_usage(self):
        try:
            cmd = "df -h / | awk 'NR==2{printf \"%s\", $5}'"
            return os.popen(cmd).read().strip()
        except:
            return "0%"
    
    def get_time(self):
        return datetime.now().strftime("%H:%M:%S")

_display = None

def get_display():
    """Get or create display instance"""
    global _display
    if _display is None:
        _display = Display(contrast=255)
    return _display


