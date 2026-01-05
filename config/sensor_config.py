# config/sensor_config.py
"""Sensor Configuration"""

# Ultrasonic Sensor (HC-SR04)
ULTRASONIC_TRIG_PIN = 17   # GPIO pin for TRIG
ULTRASONIC_ECHO_PIN = 27   # GPIO pin for ECHO
ULTRASONIC_TIMEOUT = 1     # Timeout in seconds
ULTRASONIC_SPEED = 34300   # Speed of sound (cm/s)

# Ultrasonic Distance Thresholds
ULTRASONIC_NEAR = 150      # 150mm - very close
ULTRASONIC_MID = 300       # 300mm - medium distance
ULTRASONIC_FAR = 425       # 425mm - far

# Obstacle Avoidance
OBSTACLE_THRESHOLD = 200   # Stop if < 200mm (20cm)
SAFE_DISTANCE = 250        # Safe distance (25cm)

# Line Tracking Sensor (4-way)
# X2 X1 X3 X4 = L1 L2 R1 R2
# 0 = black line detected, 1 = white
LINE_SENSOR_PINS = [5, 6, 13, 19]  # GPIO pins [L1, L2, R1, R2]

# Line Following Thresholds
LINE_THRESHOLD = 0.5       # Detect threshold

# IR Remote Control
IR_PIN = 26                # GPIO pin for IR receiver
IR_TIMEOUT = 0.1           # Timeout for IR signals

# IR Command Codes (NEC protocol)
IR_COMMANDS = {
    'power': 0x0,
    'rgb_light': 0x2,
    'buzzer': 0x5,
    'forward': 0x1,
    'backward': 0x9,
    'left': 0x4,
    'right': 0x6,
    'left_spin': 0x8,
    'right_spin': 0xa,
    'speed_up': 0x1A,
    'speed_down': 0x1B,
}

# Buzzer
BUZZER_PIN = 25            # GPIO pin for buzzer
BUZZER_FREQUENCY = 1000    # Hz

# Sensor Update Rate
SENSOR_UPDATE_RATE = 0.05  # 50ms (20 Hz)

# Autonomous Mode Settings
AUTONOMOUS_SPEED = 150     # Default speed
AUTONOMOUS_TURN_SPEED = 120  # Turn speed
AUTONOMOUS_MAX_RETRIES = 3 # Max retries for stuck detection
