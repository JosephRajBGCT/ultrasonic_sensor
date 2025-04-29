# ultrasonic_sensor.py
import threading
import time
import lgpio


class UltrasonicSensor:
    def __init__(self, chip, trig_pin, echo_pin):
        self.chip = chip
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.distance = float('inf')
        self.lock = threading.Lock()
        self.running = True

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

    def read_loop(self):
        while self.running:
            dist = self.measure_distance()
            with self.lock:
                self.distance = dist if dist > 0 else float('inf')
            time.sleep(0.05)  # 20 Hz

    def get_distance(self):
        with self.lock:
            return self.distance

    def stop(self):
        self.running = False
        self.thread.join()
