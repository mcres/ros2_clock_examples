# Based on https://github.com/ros2/demos/blob/humble/demo_nodes_py/demo_nodes_py/topics/talker.py

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Temperature


class FakeTemperatureSensorPublisher(Node):

    def __init__(self):
        super().__init__('fake_temperature_sensor_publisher')
        self.pub = self.create_publisher(Temperature, 'fake_temperature', 10)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Temperature()

        # If use_sim_time, time is fetched from /clock
        # Else, time is fetched from system clock
        msg.header.stamp.sec, msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()

        msg.temperature = 20.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = FakeTemperatureSensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()