# config/robot_config.py
"""Robot Configuration"""

ROBOT_NAME = "Raspbot V2"
ROBOT_VERSION = "1.0.0"

# ==============================================================
# CORE FEATURES (Always ON)
# ==============================================================
MOTOR_ENABLED = True           # Mecanum wheel control
DISPLAY_ENABLED = True         # OLED 128x64 display

# ==============================================================
# SENSORS (Tested & Integrated)
# ==============================================================
SENSOR_ENABLED = True          # Ultrasonic + Line tracking sensors
VISION_ENABLED = True          # Vision interface + autonomous modes

# ==============================================================
# ADVANCED FEATURES (Coming soon)
# ==============================================================
VOICE_CONTROL_ENABLED = False  # AI/Ollama voice control - future integration
