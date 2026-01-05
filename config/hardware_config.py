# config/hardware_config.py
"""Hardware Configuration"""

# OLED Display (SSD1306) - 128x64
OLED_I2C_BUS = 1
OLED_I2C_ADDR = 0x3C
OLED_CONTRAST = 255
OLED_WIDTH = 128
OLED_HEIGHT = 64  # ? Changed from 32 to 64!

# Raspbot I2C Motor Controller
RASPBOT_I2C_ADDR = 0x2B
RASPBOT_I2C_BUS = 1

# Motor IDs (from Raspbot_Lib)
MOTOR_L1 = 0     # Left motor 1
MOTOR_L2 = 1     # Left motor 2
MOTOR_R1 = 2     # Right motor 1
MOTOR_R2 = 3     # Right motor 2

# Motor Speed Control
MIN_SPEED = 0
MAX_SPEED = 255
DEFAULT_SPEED = 150

# Motor Calibration
MOTOR_LEFT_CALIBRATION = 1.0
MOTOR_RIGHT_CALIBRATION = 1.0
