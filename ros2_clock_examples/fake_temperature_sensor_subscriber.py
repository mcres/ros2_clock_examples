# Based on https://github.com/ros2/demos/blob/humble/demo_nodes_py/demo_nodes_py/topics/listener.py

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import datetime
from sensor_msgs.msg import Temperature


class FakeTemperatureSensorSubscriber(Node):

    def __init__(self):
        super().__init__('fake_temperature_sensor_subscriber')
        self.sub = self.create_subscription(Temperature, 'fake_temperature', self.temperature_callback, 10)

    def temperature_callback(self, msg):
        data_epoch = msg.header.stamp.sec
        data_datetime = str(datetime.datetime.fromtimestamp(data_epoch))
        self.get_logger().info(f'Reading {msg.temperature} ºC on {data_datetime}')


def main(args=None):
    rclpy.init(args=args)

    node = FakeTemperatureSensorSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()