import time
import lgpio
import os
import math

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin, sensor_id=None):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.sensor_id = sensor_id or 'default'
        self.running = True

        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.trig_pin, 0)
        lgpio.gpio_claim_input(self.chip, self.echo_pin)

        self.filtered_distance = 0.0
        self.alpha = 0.3

        print(f"[STDOUT] Sensor {self.sensor_id} - Process PID: {os.getpid()}")

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
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        return distance

    def low_pass_filter(self, new_value):
        return self.alpha * new_value + (1 - self.alpha) * self.filtered_distance

    def get_filtered_distance(self):
        raw_distance = self.measure_distance()
        self.filtered_distance = self.low_pass_filter(raw_distance)
        return self.filtered_distance

    def stop(self):
        self.running = False
        if hasattr(self, 'chip'):
            lgpio.gpiochip_close(self.chip)
        print(f"[STDOUT] Sensor {self.sensor_id} stopped.")
