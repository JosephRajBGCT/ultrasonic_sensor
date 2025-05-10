import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import pigpio
import time

class DistanceSensor(Node):
    def __init__(self, sensor_id: str, trig_pin: int, echo_pin: int, alpha: float = 1.0):
        super().__init__('distance_sensor_node')
        self.sensor_id = sensor_id
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.alpha = alpha  # Smoothing factor for the low-pass filter (set to 1 for no smoothing)

        # Initialize pigpio and check connection
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Failed to connect to pigpio daemon!")
            exit()

        # Set GPIO modes
        self.pi.set_mode(self.trig_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.echo_pin, pigpio.INPUT)

        # Create a publisher for the Range message
        self.distance_publisher = self.create_publisher(Range, 'distance', 10)

        # Perform health check
        if not self.health_check():
            self.get_logger().error(f"Sensor {self.sensor_id} is not connected. Exiting...")
            exit()

        self.get_logger().info(f"Sensor {self.sensor_id} initialized and ready to measure distance.")

        # Initialize the filtered distance variable (starting with an initial value)
        self.filtered_distance = float('inf')

        # Set a timer for periodic measurement
        self.timer = self.create_timer(1.0 / 15, self.measure_distance)  # Publishing frequency: 15Hz

    def health_check(self):
        """ Perform a basic health check to ensure sensor is connected. """
        self.pi.write(self.trig_pin, 0)  # Ensure TRIG is low
        time.sleep(0.1)  # Wait for stabilization
        # Check if the ECHO pin is low (indicating sensor is not currently measuring)
        return self.pi.read(self.echo_pin) == 0

    def measure_distance(self):
        """ Measure distance using the ultrasonic sensor. """
        # Send a short pulse to trigger the sensor
        self.pi.write(self.trig_pin, 0)
        time.sleep(2e-6)  # Wait for sensor to stabilize
        self.pi.write(self.trig_pin, 1)
        time.sleep(10e-6)  # Send 10µs pulse
        self.pi.write(self.trig_pin, 0)

        # Measure the time it takes for the pulse to return
        start_time = time.time()
        while self.pi.read(self.echo_pin) == 0:  # Wait for echo to go high
            if time.time() - start_time > 0.03:  # Timeout after 30ms
                self.get_logger().warn(f"Sensor {self.sensor_id}: No echo received (Timeout).")
                return
        start = time.time()

        while self.pi.read(self.echo_pin) == 1:  # Wait for echo to go low
            if time.time() - start > 0.03:  # Timeout after 30ms
                self.get_logger().warn(f"Sensor {self.sensor_id}: No echo received (Timeout).")
                return
        end = time.time()

        # Calculate the distance
        duration = end - start  # Time in seconds
        distance_cm = (duration * 34300) / 2  # Speed of sound = 34300 cm/s

        # Ensure distance is within valid range (3cm to 100cm)
        if distance_cm < 3 or distance_cm > 100:
            distance_cm = float('inf')  # If outside range, set to infinity

        # Apply low-pass filter (if alpha < 1, smoothing will occur; alpha = 1 means no smoothing)
        self.filtered_distance = self.alpha * distance_cm + (1 - self.alpha) * self.filtered_distance

        # Prepare the Range message
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.sensor_id

        msg.range = round(self.filtered_distance, 2)  # Publish filtered distance in cm
        msg.min_range = 3.0  # Minimum valid range in cm
        msg.max_range = 100.0  # Maximum valid range in cm
        msg.field_of_view = 0.52  # Field of view in radians

        # Publish the Range message
        self.distance_publisher.publish(msg)

        # Log the published data
        self.get_logger().info(f"Sensor {self.sensor_id}: Filtered Distance: {msg.range} cm")

    def stop(self):
        """ Stop the sensor and clean up resources. """
        self.pi.stop()
        self.get_logger().info(f"Sensor {self.sensor_id} stopped.")

def main(args=None):
    rclpy.init(args=args)

    # Instantiate the sensor with ID, TRIG and ECHO pins, and filter alpha value
    sensor = DistanceSensor(sensor_id="Side Front", trig_pin=23, echo_pin=24, alpha=1.0)

    try:
        rclpy.spin(sensor)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and stop the sensor
        sensor.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
