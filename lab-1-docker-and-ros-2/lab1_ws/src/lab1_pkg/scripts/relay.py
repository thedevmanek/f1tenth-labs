#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):

    def __init__(self):
        super().__init__('relay')
        self.publisher_ = self.create_publisher( AckermannDriveStamped,'drive_relay', 1)
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Recieved data is: "%s"' % msg)
        msg.drive.speed=3.0*msg.drive.speed
        msg.drive.steering_angle = 3.0*msg.drive.steering_angle
        self.get_logger().info('Publish data is: "%s"' % msg)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    relay_Node = Relay()
    rclpy.spin(relay_Node)
    relay_Node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()