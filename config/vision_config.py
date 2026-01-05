#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RASPBOT V2 - Vision Configuration
Centralized configuration for face tracking, servo control, and sensors
"""

# ============================================================================
# CAMERA SETTINGS
# ============================================================================

VISION_CAMERA_INDEX = 0
VISION_FRAME_WIDTH = 640
VISION_FRAME_HEIGHT = 480
VISION_FPS = 30

# Camera properties (0 = no change)
VISION_EXPOSURE = -5
VISION_BRIGHTNESS = 0
VISION_CONTRAST = 0
VISION_SATURATION = 0

# ============================================================================
# FACE DETECTION
# ============================================================================

VISION_FACE_SCALE_FACTOR = 1.05
VISION_FACE_MIN_NEIGHBORS = 8
VISION_FACE_MIN_SIZE = (50, 50)
VISION_FACE_MAX_SIZE = (400, 400)

# Face tracking smoothing
VISION_FACE_TRACKING_ALPHA = 0.4  # 0.0-1.0 (lower = more smooth)

# Outlier detection
VISION_FACE_TRACKING_OUTLIER_THRESHOLD = 120  # pixels

# Confidence streak (frames before FACE_TRACKING state)
VISION_FACE_TRACKING_STREAK = 5

# ============================================================================
# COLOR DETECTION (HSV ranges)
# ============================================================================

# RED (wrap-around in HSV)
VISION_COLOR_RED_LOWER1 = (0, 70, 72)
VISION_COLOR_RED_UPPER1 = (10, 255, 255)
VISION_COLOR_RED_LOWER2 = (170, 70, 72)
VISION_COLOR_RED_UPPER2 = (180, 255, 255)

# GREEN
VISION_COLOR_GREEN_LOWER = (54, 109, 78)
VISION_COLOR_GREEN_UPPER = (77, 255, 255)

# BLUE
VISION_COLOR_BLUE_LOWER = (92, 100, 62)
VISION_COLOR_BLUE_UPPER = (121, 251, 255)

# YELLOW
VISION_COLOR_YELLOW_LOWER = (26, 100, 91)
VISION_COLOR_YELLOW_UPPER = (32, 255, 255)

# Minimum area (pixels²) for color detection
VISION_COLOR_MIN_AREA = 500

# ============================================================================
# SERVO CONTROL
# ============================================================================

# Pan (horizontal) servo limits
VISION_SERVO_PAN_MIN = 0
VISION_SERVO_PAN_MAX = 180
VISION_SERVO_PAN_CENTER = 90

# Tilt (vertical) servo limits
VISION_SERVO_TILT_MIN = 0
VISION_SERVO_TILT_MAX = 180
VISION_SERVO_TILT_CENTER = 45

# Search mode parameters
VISION_SERVO_SEARCH_SPEED = 2      # degrees per frame
VISION_SERVO_SEARCH_RANGE = 45     # degrees from center

# ============================================================================
# AUTONOMOUS PARAMETERS
# ============================================================================

# Ultrasonic danger distance
VISION_ULTRASONIC_DANGER_DISTANCE = 20  # cm

# Face missing threshold (frames before SEARCH)
VISION_FACE_MISSING_THRESHOLD = 30

# Confidence streak threshold (frames before FACE_TRACKING)
VISION_FACE_CONFIDENCE_STREAK_THRESHOLD = 5

# Servo adjustment
VISION_SERVO_PAN_ADJUST_STEP = 50     # pixels
VISION_SERVO_TILT_ADJUST_STEP = 50    # pixels
VISION_SERVO_DEADZONE = 120           # pixels from center

# Search limits
VISION_SEARCH_PAN_MIN = 20
VISION_SEARCH_PAN_MAX = 160
VISION_SEARCH_TILT_MIN = 10
VISION_SEARCH_TILT_MAX = 90

# Smoothing alpha (same as VISION_FACE_TRACKING_ALPHA)
VISION_FACE_CENTER_SMOOTHING_ALPHA = 0.4

# Obstacle avoidance
VISION_OBSTACLE_AVOID_SPEED = 90          # 0-255
VISION_OBSTACLE_AVOID_DURATION = 0.8      # seconds
VISION_OBSTACLE_AVOID_ROTATE_SPEED = 90   # 0-255
VISION_OBSTACLE_AVOID_ROTATE_DURATION = 1.0  # seconds

# Patrol speed
VISION_PATROL_SPEED = 80  # 0-255

# ============================================================================
# STATE MACHINE
# ============================================================================

# Autonomous states
VISION_STATE_IDLE = 'IDLE'
VISION_STATE_PATROL = 'PATROL'
VISION_STATE_FACE_TRACKING = 'FACE_TRACKING'
VISION_STATE_SEARCH = 'SEARCH'

# ============================================================================
# PRINT CONFIG ON IMPORT
# ============================================================================

if __name__ == '__main__':
    print("="*70)
    print("RASPBOT V2 - Vision Configuration")
    print("="*70)
    
    print("\n📷 CAMERA:")
    print(f"  Index: {VISION_CAMERA_INDEX}")
    print(f"  Resolution: {VISION_FRAME_WIDTH}x{VISION_FRAME_HEIGHT}")
    print(f"  FPS: {VISION_FPS}")
    
    print("\n👤 FACE DETECTION:")
    print(f"  Scale Factor: {VISION_FACE_SCALE_FACTOR}")
    print(f"  Min Neighbors: {VISION_FACE_MIN_NEIGHBORS}")
    print(f"  Min Size: {VISION_FACE_MIN_SIZE}")
    print(f"  Max Size: {VISION_FACE_MAX_SIZE}")
    print(f"  Tracking Alpha: {VISION_FACE_TRACKING_ALPHA}")
    
    print("\n🎯 SERVO:")
    print(f"  Pan: {VISION_SERVO_PAN_MIN}-{VISION_SERVO_PAN_MAX}° (Center: {VISION_SERVO_PAN_CENTER}°)")
    print(f"  Tilt: {VISION_SERVO_TILT_MIN}-{VISION_SERVO_TILT_MAX}° (Center: {VISION_SERVO_TILT_CENTER}°)")
    
    print("\n⚙️  AUTONOMOUS:")
    print(f"  Danger Distance: {VISION_ULTRASONIC_DANGER_DISTANCE}cm")
    print(f"  Face Missing Threshold: {VISION_FACE_MISSING_THRESHOLD} frames")
    print(f"  Patrol Speed: {VISION_PATROL_SPEED}")
    
    print("\n" + "="*70 + "\n")


