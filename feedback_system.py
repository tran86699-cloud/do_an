"""

hardware/feedback_system.py - FIXED VERSION

Feedback System for RASPBOT V2

- LED status indicators (5s timeout for mode changes)

- Buzzer notifications

- OLED display (battery %, IP address)

- Obstacle detection with camera fallback

"""



import time

import threading

from hardware.buzzer import Buzzer

from hardware.display import Display



class FeedbackSystem:

    def __init__(self, ultrasonic, camera=None):

        self.buzzer = Buzzer()

        self.display = Display()

        # LED - Optional (not implemented yet)

        self.led = None

        self.led_active = False

        

        self.ultrasonic = ultrasonic

        self.camera = camera

        

        # State tracking

        self.current_mode = 'idle'

        self.led_timer = None

        self.battery_monitor_thread = None

        self.obstacle_state = 'clear'

        self.obstacle_timer = None

        

        # Config

        self.OBSTACLE_WARNING_DISTANCE = 30

        self.OBSTACLE_STOP_DISTANCE = 15

        self.OBSTACLE_TIMEOUT = 3

        self.LOW_BATTERY_THRESHOLD = 20

        self.LED_MODE_DURATION = 5

        

        self._running = False

    

    # ==============================

    # STARTUP & CONNECTION

    # ==============================

    

    def startup_sequence(self):

        """Beep beep beep - Robot ready to connect"""

        print("[FEEDBACK] Startup sequence...")

        for i in range(3):

            self.buzzer.beep(0.2)

            time.sleep(0.3)

        print("[FEEDBACK] Ready for connection!")

    

    def connection_success(self, ip_address):

        """Short beep - Connected to PC successfully"""

        print(f"[FEEDBACK] Connected to PC at {ip_address}")

        self.buzzer.beep(0.1)

        self.update_display_ip(ip_address)

    

    # ==============================

    # MODE CHANGE

    # ==============================

    

    def mode_changed(self, new_mode):

        """

        Beep x2 + LED color for 5 seconds

        new_mode: 'manual', 'patrol', 'obstacle', 'line_follow', etc.

        """

        print(f"[FEEDBACK] Mode changed to: {new_mode}")

        

        # Beep x2

        self.buzzer.beep(0.2)

        time.sleep(0.15)

        self.buzzer.beep(0.2)

        

        # LED color based on mode (5s timeout)

        self.current_mode = new_mode

        self._set_mode_led(new_mode)

        

        # Auto turn off after 5 seconds

        if self.led_timer:

            self.led_timer.cancel()

        self.led_timer = threading.Timer(self.LED_MODE_DURATION, self._turn_off_led)

        self.led_timer.start()

    

    def _set_mode_led(self, mode):

        """Set LED color based on mode"""

        color_map = {

            'manual': 'BLUE',

            'patrol': 'GREEN',

            'obstacle': 'YELLOW',

            'line_follow': 'PURPLE',

            'idle': 'WHITE'

        }

        color = color_map.get(mode, 'WHITE')

        

        if self.led:

            print(f"[FEEDBACK] LED: {color} for {self.LED_MODE_DURATION}s")

            # self.led.set_color(color)

        else:

            print(f"[FEEDBACK] LED not available (mode: {mode})")

        

        self.led_active = True

    

    def _turn_off_led(self):

        """Turn off LED after timeout"""

        print("[FEEDBACK] LED timeout - turning off")

        if self.led:

            # self.led.turn_off()

            pass

        self.led_active = False

    

    # ==============================

    # OBSTACLE DETECTION

    # ==============================

    

    def check_obstacle(self, current_speed):

        """

        Check obstacle and provide feedback

        Returns: 'clear', 'warning', 'blocked', 'alternate_path'

        """

        distance = self.ultrasonic.measure_distance()

        

        # Clear path

        if distance >= self.OBSTACLE_WARNING_DISTANCE:

            if self.obstacle_state != 'clear':

                print("[FEEDBACK] Path clear")

                self._turn_off_led()

                if self.obstacle_timer:

                    self.obstacle_timer.cancel()

                    self.obstacle_timer = None

            self.obstacle_state = 'clear'

            return 'clear'

        

        # Warning - slowing down

        elif distance >= self.OBSTACLE_STOP_DISTANCE:

            if self.obstacle_state != 'warning':

                print(f"[FEEDBACK] Obstacle warning at {distance}cm")

                self.led_active = True

            self.obstacle_state = 'warning'

            return 'warning'

        

        # Blocked - stopped

        else:

            if self.obstacle_state != 'blocked':

                print(f"[FEEDBACK] Obstacle blocked at {distance}cm")

                self.led_active = True

                self._request_clearance_buzzer()

                

                # Start 3s timer for camera fallback

                if self.obstacle_timer:

                    self.obstacle_timer.cancel()

                self.obstacle_timer = threading.Timer(

                    self.OBSTACLE_TIMEOUT, 

                    self._find_alternate_path

                )

                self.obstacle_timer.start()

            

            self.obstacle_state = 'blocked'

            return 'blocked'

    

    def _request_clearance_buzzer(self):

        """Buzzer pattern to request clearance"""

        print("[FEEDBACK] Requesting clearance...")

        self.buzzer.beep(0.5)

        time.sleep(0.2)

        self.buzzer.beep(0.3)

    

    def _find_alternate_path(self):

        """Use camera to find alternate path after 3s blockage"""

        print("[FEEDBACK] Still blocked after 3s - using camera to find path")

        

        if not self.camera:

            print("[FEEDBACK] No camera available - staying blocked")

            return 'blocked'

        

        print("[FEEDBACK] Camera analyzing for alternate path...")

        return 'alternate_path'

    

    # ==============================

    # BATTERY MONITORING

    # ==============================

    

    def start_battery_monitor(self, battery_reader):

        """

        Start background thread to monitor battery

        Beep every 3s if battery < 20%

        """

        self._running = True

        

        def monitor():

            while self._running:

                battery_percent = battery_reader.get_battery_percent()

                

                # Update OLED display

                self.update_display_battery(battery_percent)

                

                # Low battery warning

                if battery_percent < self.LOW_BATTERY_THRESHOLD:

                    print(f"[FEEDBACK] Low battery: {battery_percent}%")

                    self.buzzer.beep(0.1)

                    time.sleep(3)

                else:

                    time.sleep(1)

        

        self.battery_monitor_thread = threading.Thread(target=monitor, daemon=True)

        self.battery_monitor_thread.start()

        print("[FEEDBACK] Battery monitor started")

    

    def stop_battery_monitor(self):

        """Stop battery monitoring thread"""

        self._running = False

        if self.battery_monitor_thread:

            self.battery_monitor_thread.join(timeout=1)

        print("[FEEDBACK] Battery monitor stopped")

    

    # ==============================

    # OLED DISPLAY - FIXED METHODS

    # ==============================

    

    def update_display_battery(self, battery_percent):

        """Update OLED with battery percentage"""

        # Use show_frame instead of show_text

        if self.display and self.display.available:

            self.display.show_frame(

                f"Battery: {battery_percent}%",

                "",

                ""

            )

    

    def update_display_ip(self, ip_address):

        """Update OLED with IP address"""

        if self.display and self.display.available:

            self.display.show_frame(

                "",

                f"IP: {ip_address}",

                ""

            )

    

    def update_display_mode(self, mode):

        """Update OLED with current mode"""

        if self.display and self.display.available:

            self.display.show_frame(

                "",

                "",

                f"Mode: {mode}"

            )

    

    def update_display_full(self, battery_percent, ip_address, mode):

        """Update all OLED information at once - FIXED"""

        if self.display and self.display.available:

            # Use show_frame instead of show_multi_line

            self.display.show_frame(

                f"Battery: {battery_percent}%",

                f"IP: {ip_address}",

                f"Mode: {mode}"

            )

    

    # ==============================

    # CLEANUP

    # ==============================

    

    def cleanup(self):

        """Clean up all resources"""

        print("[FEEDBACK] Cleaning up...")

        self.stop_battery_monitor()

        

        if self.led_timer:

            self.led_timer.cancel()

        if self.obstacle_timer:

            self.obstacle_timer.cancel()

        

        self._turn_off_led()

        if self.display:

            self.display.clear()

        print("[FEEDBACK] Cleanup complete")





# ==============================

# EXAMPLE USAGE

# ==============================



if __name__ == "__main__":

    # Mock objects for testing

    class MockUltrasonic:

        def measure_distance(self):

            return 25

    

    class MockBatteryReader:

        def get_battery_percent(self):

            return 75

    

    # Initialize

    ultrasonic = MockUltrasonic()

    feedback = FeedbackSystem(ultrasonic)

    

    # Test startup

    feedback.startup_sequence()

    time.sleep(1)

    

    # Test connection

    feedback.connection_success("192.168.1.140")

    time.sleep(1)

    

    # Test mode change

    feedback.mode_changed("patrol")

    time.sleep(2)

    

    # Test display update

    feedback.update_display_full(75, "192.168.1.140", "TEST")

    time.sleep(3)

    

    # Cleanup

    feedback.cleanup()





