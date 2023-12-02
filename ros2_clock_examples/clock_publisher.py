import datetime
import time
import rclpy

from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class ClockPublisherNode(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        self.publisher = self.create_publisher(Clock, '/clock', 10)
        self.get_logger().info('Clock Publisher Node has started.')

    def publish_time(self):
        clock_msg = Clock()

        current_datetime = datetime.datetime.now()

        # Go back in time
        years_back = 30
        past_datetime = datetime.datetime.now() - datetime.timedelta(days=years_back*365)
        
        past_epoch = datetime.datetime.timestamp(past_datetime)
        clock_msg.clock.sec = int(past_epoch)

        self.publisher.publish(clock_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClockPublisherNode()

    # Publish system time periodically
    timer_period = 0.2  # in seconds
    node.create_timer(timer_period, node.publish_time)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
