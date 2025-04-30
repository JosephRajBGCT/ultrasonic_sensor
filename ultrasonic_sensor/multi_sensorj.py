import rclpy
from rclpy.node import Node
import threading
from sensor_msgs.msg import Range
from .ultrasonicsensorj import UltrasonicSensor  # Import the base class

class UltrasonicSensorNode(Node):
    def __init__(self, trig_pin, echo_pin, sensor_id):
        super().__init__(f'ultrasonic_sensor_node_{sensor_id}')
        self.get_logger().info(f"Sensor {sensor_id} Node Initialized")

        # Initialize the base ultrasonic sensor
        self.sensor = UltrasonicSensorBase(trig_pin, echo_pin, sensor_id)

        # Publisher for ultrasonic sensor distance
        self.publisher = self.create_publisher(Range, f'ultrasonic_distance_{sensor_id}', 10)

        # Thread to handle sensor measurements
        self.thread = threading.Thread(target=self.measure_and_publish, daemon=True)
        self.thread.start()

    def measure_and_publish(self):
        while rclpy.ok():
            # Get the filtered distance from the sensor base class
            distance = self.sensor.get_filtered_distance()
            
            # Create and publish the message
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"ultrasonic_sensor_{self.sensor.sensor_id}"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.52
            msg.min_range = 3.0
            msg.max_range = 100.0
            msg.range = float('inf') if distance < msg.min_range or distance > msg.max_range else float(distance)

            self.publisher.publish(msg)
            self.get_logger().info(f"Sensor {self.sensor.sensor_id} Distance: {msg.range:.2f} cm")
            time.sleep(0.2)  # Control the frequency of publishing

    def destroy_node(self):
        self.sensor.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize two sensors
    sensor_1 = UltrasonicSensorNode(23, 24, 1)
    sensor_2 = UltrasonicSensorNode(12, 13, 2)

    try:
        rclpy.spin(sensor_1)
        rclpy.spin(sensor_2)
    except KeyboardInterrupt:
        print("[STDOUT] KeyboardInterrupt received. Shutting down...")
    finally:
        sensor_1.destroy_node()
        sensor_2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

