"""
RASPBOT V2 - Improved PID Controller Library
===========================================

Enhanced PID controllers with:
- Output clamping
- Anti-windup (integral limits)
- Derivative filtering
- Clear API
- Specialized classes for different use cases

Author: RASPBOT V2 Project
Date: 2025-12-30
"""

import time
import math


# ============================================================================
# BASE PID CONTROLLER (Generic)
# ============================================================================

class BasePID:
    """
    Base PID controller with all essential features
    
    Features:
    - Proportional, Integral, Derivative control
    - Output clamping (min/max limits)
    - Anti-windup (integral clamping)
    - Derivative filtering (smooth D term)
    - Sample time handling
    - Enable/disable
    - Reset capability
    """
    
    def __init__(self, kp, ki, kd, 
                 output_min=-255, output_max=255,
                 integral_min=-1000, integral_max=1000,
                 derivative_filter=0.1,
                 sample_time=0.01):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_min: Minimum output value
            output_max: Maximum output value
            integral_min: Minimum integral accumulation
            integral_max: Maximum integral accumulation
            derivative_filter: Low-pass filter coefficient (0-1)
            sample_time: Expected sample time in seconds
        """
        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Output limits
        self.output_min = output_min
        self.output_max = output_max
        
        # Integral limits (anti-windup)
        self.integral_min = integral_min
        self.integral_max = integral_max
        
        # Derivative filter
        self.derivative_filter = derivative_filter
        
        # Sample time
        self.sample_time = sample_time
        
        # State variables
        self.error = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.filtered_derivative = 0.0
        self.output = 0.0
        
        # Timing
        self.last_time = time.time()
        
        # Control
        self.enabled = True
        
        # Debug
        self.debug_mode = False
    
    def compute(self, setpoint, measured_value):
        """
        Compute PID output
        
        Args:
            setpoint: Target value
            measured_value: Current measured value
            
        Returns:
            float: PID output (clamped to min/max)
        """
        if not self.enabled:
            return 0.0
        
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Use sample_time if dt is too small
        if dt < 0.001:
            dt = self.sample_time
        
        # Calculate error
        self.error = setpoint - measured_value
        
        # Proportional term
        p_term = self.kp * self.error
        
        # Integral term (with anti-windup)
        self.integral += self.error * dt
        self.integral = self._clamp(self.integral, 
                                    self.integral_min, 
                                    self.integral_max)
        i_term = self.ki * self.integral
        
        # Derivative term (with filtering)
        raw_derivative = (self.error - self.last_error) / dt
        self.filtered_derivative = (self.derivative_filter * raw_derivative + 
                                    (1 - self.derivative_filter) * self.filtered_derivative)
        d_term = self.kd * self.filtered_derivative
        
        # Calculate output
        self.output = p_term + i_term + d_term
        
        # Clamp output
        self.output = self._clamp(self.output, 
                                  self.output_min, 
                                  self.output_max)
        
        # Update state
        self.last_error = self.error
        self.last_time = current_time
        
        # Debug output
        if self.debug_mode:
            self._print_debug(p_term, i_term, d_term)
        
        return self.output
    
    def reset(self):
        """Reset PID controller state"""
        self.error = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.filtered_derivative = 0.0
        self.output = 0.0
        self.last_time = time.time()
    
    def enable(self):
        """Enable PID controller"""
        self.enabled = True
        self.reset()
    
    def disable(self):
        """Disable PID controller"""
        self.enabled = False
        self.output = 0.0
    
    def set_gains(self, kp=None, ki=None, kd=None):
        """Update PID gains"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
    
    def _clamp(self, value, min_val, max_val):
        """Clamp value to range"""
        return max(min_val, min(max_val, value))
    
    def _print_debug(self, p, i, d):
        """Print debug information"""
        print(f"[PID] Error: {self.error:.2f} | "
              f"P: {p:.2f} | I: {i:.2f} | D: {d:.2f} | "
              f"Output: {self.output:.2f}")


# ============================================================================
# LINE FOLLOWING PID
# ============================================================================

class LinePID(BasePID):
    """
    Specialized PID for line following
    
    Features:
    - Converts 4 line sensors to position (-2 to +2)
    - Outputs steering correction (-255 to +255)
    - Optimized gains for smooth line tracking
    """
    
    def __init__(self, kp=50.0, ki=0.5, kd=15.0, base_speed=100):
        """
        Initialize Line Following PID
        
        Args:
            kp: Proportional gain (default 50.0)
            ki: Integral gain (default 0.5)
            kd: Derivative gain (default 15.0)
            base_speed: Base movement speed (0-255)
        """
        super().__init__(
            kp=kp, ki=ki, kd=kd,
            output_min=-255, output_max=255,
            integral_min=-100, integral_max=100,
            derivative_filter=0.2,
            sample_time=0.05
        )
        self.base_speed = base_speed
    
    def calculate_position(self, sensors):
        """
        Calculate line position from 4 sensors
        
        Args:
            sensors: List [L2, L1, R1, R2] where 1=black, 0=white
            
        Returns:
            float: Position from -2 (far left) to +2 (far right)
                   0 = centered on line
        """
        l2, l1, r1, r2 = sensors
        
        # Weight each sensor
        weights = [-2, -1, 1, 2]
        position = 0.0
        total_active = 0
        
        for i, sensor in enumerate(sensors):
            if sensor == 1:  # On black line
                position += weights[i]
                total_active += 1
        
        if total_active > 0:
            position /= total_active
        else:
            # No line detected - return large error to stop
            position = self.last_error if self.last_error != 0 else 0
        
        return position
    
    def compute_line_following(self, sensors):
        """
        Compute motor speeds for line following
        
        Args:
            sensors: List [L2, L1, R1, R2]
            
        Returns:
            tuple: (left_speed, right_speed)
        """
        # Calculate line position
        position = self.calculate_position(sensors)
        
        # Compute PID correction (0 = centered)
        correction = self.compute(setpoint=0, measured_value=position)
        
        # Apply correction to motor speeds
        left_speed = self.base_speed - correction
        right_speed = self.base_speed + correction
        
        # Clamp to valid range
        left_speed = self._clamp(left_speed, 0, 255)
        right_speed = self._clamp(right_speed, 0, 255)
        
        return int(left_speed), int(right_speed)


# ============================================================================
# MOTOR SPEED PID (Requires encoder feedback)
# ============================================================================

class MotorPID(BasePID):
    """
    Specialized PID for motor speed control
    
    Features:
    - Maintains constant speed despite load changes
    - Requires encoder or speed feedback
    - Helps keep robot moving straight
    """
    
    def __init__(self, kp=0.5, ki=0.1, kd=0.05, target_speed=100):
        """
        Initialize Motor Speed PID
        
        Args:
            kp: Proportional gain (default 0.5)
            ki: Integral gain (default 0.1)
            kd: Derivative gain (default 0.05)
            target_speed: Target speed (0-255)
        """
        super().__init__(
            kp=kp, ki=ki, kd=kd,
            output_min=0, output_max=255,
            integral_min=-50, integral_max=50,
            derivative_filter=0.3,
            sample_time=0.05
        )
        self.target_speed = target_speed
    
    def compute_motor_speed(self, measured_speed):
        """
        Compute motor PWM to maintain target speed
        
        Args:
            measured_speed: Current measured speed (from encoder)
            
        Returns:
            int: Motor PWM value (0-255)
        """
        output = self.compute(self.target_speed, measured_speed)
        return int(output)
    
    def set_target_speed(self, speed):
        """Update target speed"""
        self.target_speed = self._clamp(speed, 0, 255)


# ============================================================================
# HEADING/ORIENTATION PID (Requires IMU/Gyro)
# ============================================================================

class HeadingPID(BasePID):
    """
    Specialized PID for heading control
    
    Features:
    - Maintains straight line movement
    - Corrects drift/rotation
    - Requires IMU/Gyroscope
    """
    
    def __init__(self, kp=2.0, ki=0.05, kd=1.0, target_heading=0):
        """
        Initialize Heading PID
        
        Args:
            kp: Proportional gain (default 2.0)
            ki: Integral gain (default 0.05)
            kd: Derivative gain (default 1.0)
            target_heading: Target heading in degrees (0-360)
        """
        super().__init__(
            kp=kp, ki=ki, kd=kd,
            output_min=-100, output_max=100,
            integral_min=-30, integral_max=30,
            derivative_filter=0.2,
            sample_time=0.05
        )
        self.target_heading = target_heading
    
    def compute_heading_correction(self, current_heading):
        """
        Compute correction to maintain heading
        
        Args:
            current_heading: Current heading in degrees (0-360)
            
        Returns:
            int: Correction value (-100 to +100)
                 Negative = turn left, Positive = turn right
        """
        # Calculate shortest angular distance
        error = self._angle_difference(self.target_heading, current_heading)
        
        # Compute PID
        correction = self.compute(setpoint=0, measured_value=error)
        
        return int(correction)
    
    def set_target_heading(self, heading):
        """Update target heading"""
        self.target_heading = heading % 360
    
    def _angle_difference(self, target, current):
        """Calculate shortest angle difference (-180 to +180)"""
        diff = (target - current + 180) % 360 - 180
        return diff


# ============================================================================
# OBSTACLE AVOIDANCE PID (Smooth deceleration)
# ============================================================================

class ObstaclePID(BasePID):
    """
    Specialized PID for smooth deceleration near obstacles
    
    Features:
    - Smooth speed reduction as obstacle approaches
    - Prevents jerky stops
    - Uses ultrasonic distance
    """
    
    def __init__(self, kp=3.0, ki=0.1, kd=1.5, 
                 safe_distance=50, stop_distance=15):
        """
        Initialize Obstacle PID
        
        Args:
            kp: Proportional gain (default 3.0)
            ki: Integral gain (default 0.1)
            kd: Derivative gain (default 1.5)
            safe_distance: Distance to start slowing (cm)
            stop_distance: Distance to stop completely (cm)
        """
        super().__init__(
            kp=kp, ki=ki, kd=kd,
            output_min=0, output_max=255,
            integral_min=-20, integral_max=20,
            derivative_filter=0.3,
            sample_time=0.05
        )
        self.safe_distance = safe_distance
        self.stop_distance = stop_distance
    
    def compute_safe_speed(self, distance, desired_speed=150):
        """
        Compute safe speed based on obstacle distance
        
        Args:
            distance: Current distance to obstacle (cm)
            desired_speed: Desired speed when no obstacle (0-255)
            
        Returns:
            int: Safe speed (0-255)
                 0 = obstacle too close
                 desired_speed = no obstacle detected
        """
        if distance >= self.safe_distance:
            # No obstacle - full speed
            return desired_speed
        
        elif distance <= self.stop_distance:
            # Too close - stop
            return 0
        
        else:
            # Interpolate speed based on distance
            # Linear interpolation between stop and safe distance
            speed_ratio = (distance - self.stop_distance) / \
                         (self.safe_distance - self.stop_distance)
            
            safe_speed = int(desired_speed * speed_ratio)
            
            # Use PID to smooth the transition
            smooth_speed = self.compute(setpoint=safe_speed, 
                                       measured_value=self.output)
            
            return int(smooth_speed)


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("RASPBOT V2 - PID Controller Test")
    print("=" * 60)
    
    # Test 1: Line Following PID
    print("\n[TEST 1] Line Following PID")
    line_pid = LinePID(kp=50, ki=0.5, kd=15, base_speed=100)
    line_pid.debug_mode = True
    
    # Simulate line sensors
    test_cases = [
        ([0, 1, 1, 0], "Centered"),
        ([1, 1, 0, 0], "Left"),
        ([0, 0, 1, 1], "Right"),
        ([1, 0, 0, 0], "Far Left"),
        ([0, 0, 0, 1], "Far Right")
    ]
    
    for sensors, description in test_cases:
        left, right = line_pid.compute_line_following(sensors)
        print(f"{description}: L={left}, R={right}")
        time.sleep(0.1)
    
    # Test 2: Obstacle PID
    print("\n[TEST 2] Obstacle Avoidance PID")
    obstacle_pid = ObstaclePID(safe_distance=50, stop_distance=15)
    
    distances = [100, 60, 40, 25, 15, 10]
    for dist in distances:
        speed = obstacle_pid.compute_safe_speed(dist, desired_speed=150)
        print(f"Distance: {dist}cm → Speed: {speed}")
        time.sleep(0.1)
    
    # Test 3: Motor Speed PID (simulated)
    print("\n[TEST 3] Motor Speed PID")
    motor_pid = MotorPID(target_speed=150)
    
    # Simulate speed feedback
    measured_speeds = [100, 120, 140, 145, 150, 150]
    for speed in measured_speeds:
        pwm = motor_pid.compute_motor_speed(speed)
        print(f"Measured: {speed} → PWM: {pwm}")
        time.sleep(0.1)
    
    print("\n" + "=" * 60)
    print("Test Complete!")