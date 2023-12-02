import datetime
import time
import rclpy

from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class SlowedDownClockPublisherNode(Node):
    def __init__(self):
        super().__init__('past_clock_publisher')
        self.publisher = self.create_publisher(Clock, '/clock', 10)
        self.get_logger().info('Clock Publisher Node has started.')

        # Initialize seconds with current value
        current_datetime = datetime.datetime.now()
        self.clock_seconds = int(datetime.datetime.timestamp(current_datetime))

        # How slower the /clock is compared to system time
        system_to_sim_time_ratio = 2

        # Publish system time periodically
        self.create_timer(system_to_sim_time_ratio, self.publish_time)

        # Log system time so it can be compared to /clock
        self.create_timer(1, self.log_current_time)

    def publish_time(self):
        clock_msg = Clock()

        self.clock_seconds += 1
        clock_msg.clock.sec = self.clock_seconds

        self.publisher.publish(clock_msg)

    def log_current_time(self):
        sec, _ = self.get_clock().now().seconds_nanoseconds()
        data_datetime = str(datetime.datetime.fromtimestamp(sec))
        self.get_logger().info(f'System datetime: {data_datetime}')

def main(args=None):
    rclpy.init(args=args)
    node = SlowedDownClockPublisherNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
