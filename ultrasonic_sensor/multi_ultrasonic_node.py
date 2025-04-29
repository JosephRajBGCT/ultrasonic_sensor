# multi_ultrasonic_node.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Range
import lgpio
import os

from .ultrasonic_sensor import UltrasonicSensor


class MultiUltrasonicNode(Node):
    def __init__(self):
        super().__init__('multi_ultrasonic_node')
        self.chip = lgpio.gpiochip_open(0)

        self.sensor_configs = {
            'UT1': (23, 24),
            'UT2': (5, 6),
            'UT3': (12, 13),
            'UT4': (16, 17),
            'UT5': (19, 20),
            'UT6': (21, 22)
        }

        self.sensors = {}
        self.publishers = {}

        for name, (trig, echo) in self.sensor_configs.items():
            self.sensors[name] = UltrasonicSensor(self.chip, trig, echo)
            self.publishers[name] = self.create_publisher(Range, f'/ultrasonic/{name}', 10)

        self.timer = self.create_timer(0.1, self.publish_all)

    def publish_all(self):
        now = self.get_clock().now().to_msg()
        for name, sensor in self.sensors.items():
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = name
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.52
            msg.min_range = 3.0
            msg.max_range = 100.0

            distance = sensor.get_distance()
            msg.range = min(max(distance, msg.min_range), msg.max_range) if distance != float('inf') else float('inf')
            self.publishers[name].publish(msg)
            self.get_logger().info(f"[{name}] Range: {msg.range:.2f} cm")

    def destroy_node(self):
        for sensor in self.sensors.values():
            sensor.stop()
        lgpio.gpiochip_close(self.chip)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiUltrasonicNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("[STDOUT] KeyboardInterrupt received.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
