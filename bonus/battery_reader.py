"""
hardware/battery_reader.py
Battery Reader for RASPBOT V2
Monitors battery voltage and converts to percentage
"""

import time

class BatteryReader:
    """
    Read battery voltage and calculate percentage
    Assuming you have a voltage divider circuit or I2C battery monitor
    """
    
    def __init__(self, min_voltage=6.0, max_voltage=8.4):
        """
        Initialize battery reader
        
        Args:
            min_voltage: Minimum safe voltage (0% battery)
            max_voltage: Maximum voltage (100% battery)
            
        For 2S LiPo: 6.0V (empty) to 8.4V (full)
        For 3S LiPo: 9.0V (empty) to 12.6V (full)
        """
        self.min_voltage = min_voltage
        self.max_voltage = max_voltage
        
        # If using I2C battery monitor
        # self.i2c_address = 0x??
        # self.bus = smbus.SMBus(1)
    
    def get_voltage(self):
        """
        Read current battery voltage
        
        Returns:
            float: Battery voltage in volts
        """
        # TODO: Implement actual voltage reading
        # Option 1: Using I2C battery monitor
        # voltage = self._read_i2c_voltage()
        
        # Option 2: Using ADC (like ADS1115)
        # voltage = self._read_adc_voltage()
        
        # Option 3: Using GPIO with voltage divider
        # voltage = self._read_gpio_voltage()
        
        # Mock voltage for testing
        voltage = 7.5
        
        return voltage
    
    def get_battery_percent(self):
        """
        Calculate battery percentage from voltage
        
        Returns:
            int: Battery percentage (0-100)
        """
        voltage = self.get_voltage()
        
        # Linear interpolation
        percent = ((voltage - self.min_voltage) / 
                   (self.max_voltage - self.min_voltage)) * 100
        
        # Clamp to 0-100
        percent = max(0, min(100, percent))
        
        return int(percent)
    
    def is_low_battery(self, threshold=20):
        """
        Check if battery is below threshold
        
        Args:
            threshold: Percentage threshold (default 20%)
            
        Returns:
            bool: True if battery is low
        """
        return self.get_battery_percent() < threshold
    
    def _read_i2c_voltage(self):
        """Read voltage from I2C battery monitor"""
        # Example for INA219 or similar
        # voltage_reg = 0x02
        # data = self.bus.read_word_data(self.i2c_address, voltage_reg)
        # voltage = data * 0.001  # Convert to volts
        # return voltage
        pass
    
    def _read_adc_voltage(self):
        """Read voltage from ADC with voltage divider"""
        # Example for ADS1115
        # import Adafruit_ADS1x15
        # adc = Adafruit_ADS1x15.ADS1115()
        # raw_value = adc.read_adc(0, gain=1)
        # voltage = raw_value * (4.096 / 32767) * 2  # x2 for voltage divider
        # return voltage
        pass
    
    def _read_gpio_voltage(self):
        """Read voltage from GPIO pin (requires external ADC)"""
        # RPi doesn't have built-in ADC, need external chip
        pass


# ==============================
# SIMPLE VERSION - Mock for testing
# ==============================

class MockBatteryReader(BatteryReader):
    """Mock battery reader for testing"""
    
    def __init__(self):
        super().__init__()
        self._voltage = 7.5  # Default voltage
        self._declining = True
    
    def get_voltage(self):
        """Simulate declining battery"""
        if self._declining:
            self._voltage -= 0.01
            if self._voltage <= self.min_voltage:
                self._voltage = self.max_voltage
        return self._voltage


# ==============================
# EXAMPLE USAGE
# ==============================

if __name__ == "__main__":
    # Create battery reader
    battery = BatteryReader(min_voltage=6.0, max_voltage=8.4)
    
    # Monitor battery
    print("Monitoring battery...")
    for i in range(10):
        voltage = battery.get_voltage()
        percent = battery.get_battery_percent()
        low = battery.is_low_battery(threshold=20)
        
        print(f"Voltage: {voltage:.2f}V | Battery: {percent}% | Low: {low}")
        time.sleep(1)