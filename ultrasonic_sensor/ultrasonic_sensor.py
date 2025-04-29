import threading
import time
import lgpio

class UltrasonicSensor:
    def __init__(self, chip, trig_pin, echo_pin, min_range=3, max_range=100):
        self.chip = chip
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.min_range = min_range  # Minimum valid distance in cm
        self.max_range = max_range  # Maximum valid distance in cm
        self.distance = float('inf')
        self.lock = threading.Lock()
        self.running = True
        
        lgpio.gpio_claim_output(chip, trig_pin, 0)
        lgpio.gpio_claim_input(chip, echo_pin)
        
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()
    
    def measure_distance(self):
        # Send trigger pulse
        lgpio.gpio_write(self.chip, self.trig_pin, 0)
        time.sleep(0.002)
        lgpio.gpio_write(self.chip, self.trig_pin, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, self.trig_pin, 0)
        
        # Wait for echo to start with timeout
        timeout = time.time() + 0.02
        while lgpio.gpio_read(self.chip, self.echo_pin) == 0:
            if time.time() > timeout:
                return -1
        pulse_start = time.time()
        
        # Wait for echo to end with timeout
        while lgpio.gpio_read(self.chip, self.echo_pin) == 1:
            if time.time() > timeout:
                return -1
        pulse_end = time.time()
        
        # Calculate distance in cm
        duration = pulse_end - pulse_start
        distance_cm = duration * 17150
        
        return distance_cm
    
    def read_loop(self):
        while self.running:
            dist = self.measure_distance()
            
            # Apply range validation
            with self.lock:
                if dist < 0 or dist < self.min_range or dist > self.max_range:
                    self.distance = float('inf')  # Out of bounds
                else:
                    self.distance = dist
                    
            time.sleep(0.05)  # 20 Hz sampling rate
    
    def get_distance(self):
        with self.lock:
            return self.distance
    
    def stop(self):
        self.running = False
        self.thread.join()