import threading
import time
import lgpio
import collections


class UltrasonicSensor:
    def __init__(self, chip, trig_pin, echo_pin, filter_size=5, max_distance=400):
        self.chip = chip
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.distance = float('inf')
        self.lock = threading.Lock()
        self.running = True
        
        # Filter parameters
        self.filter_size = filter_size
        self.readings = collections.deque(maxlen=filter_size)
        self.max_distance = max_distance  # Maximum reliable distance in cm
        
        lgpio.gpio_claim_output(chip, trig_pin, 0)
        lgpio.gpio_claim_input(chip, echo_pin)
        
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()
    
    def measure_distance(self):
        lgpio.gpio_write(self.chip, self.trig_pin, 0)
        time.sleep(0.002)
        lgpio.gpio_write(self.chip, self.trig_pin, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, self.trig_pin, 0)
        
        timeout = time.time() + 0.02
        while lgpio.gpio_read(self.chip, self.echo_pin) == 0:
            if time.time() > timeout:
                return -1
        pulse_start = time.time()
        
        while lgpio.gpio_read(self.chip, self.echo_pin) == 1:
            if time.time() > timeout:
                return -1 
        pulse_end = time.time()
        
        duration = pulse_end - pulse_start
        return duration * 17150  # cm
    
    def filter_distance(self, distance):
        """Apply filtering to the raw distance measurement."""
        # Ignore invalid readings
        if distance <= 0 or distance > self.max_distance:
            # Don't add this reading to the filter
            return self.get_filtered_distance()
        
        # Add the new reading to our collection
        self.readings.append(distance)
        
        # Return filtered value
        return self.get_filtered_distance()
    
    def get_filtered_distance(self):
        """Calculate filtered distance based on readings in the queue."""
        if not self.readings:
            return float('inf')
        
        # Remove outliers (optional)
        valid_readings = list(self.readings)
        if len(valid_readings) >= 3:
            # Remove highest and lowest if we have enough readings
            valid_readings.remove(max(valid_readings))
            valid_readings.remove(min(valid_readings))
        
        # Calculate average of remaining readings
        if valid_readings:
            return sum(valid_readings) / len(valid_readings)
        else:
            return float('inf')
    
    def read_loop(self):
        while self.running:
            dist = self.measure_distance()
            raw_dist = dist if dist > 0 else float('inf')
            
            with self.lock:
                # Apply filter and update distance
                self.distance = self.filter_distance(raw_dist)
            
            time.sleep(0.05)  # 20 Hz
    
    def get_distance(self):
        with self.lock:
            return self.distance
    
    def get_raw_distance(self):
        """Get the last unfiltered measurement."""
        dist = self.measure_distance()
        return dist if dist > 0 else float('inf')
    
    def stop(self):
        self.running = False
        self.thread.join()


# Example usage
if __name__ == "__main__":
    # Example code to demonstrate the sensor with filtering
    try:
        h = lgpio.gpiochip_open(0)  # Open GPIO chip
        sensor = UltrasonicSensor(h, trig_pin=23, echo_pin=24, filter_size=10)
        
        print("Press Ctrl+C to exit")
        while True:
            distance = sensor.get_distance()
            print(f"Distance: {distance:.1f} cm")
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if 'sensor' in locals():
            sensor.stop()
        if 'h' in locals():
            lgpio.gpiochip_close(h)