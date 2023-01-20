#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """

    def __init__(self):
        super().__init__("safety_node")
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.0
        # TODO: create ROS subscribers and publishers.
        self.laserscan_subscriber = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, "/ego_racecar/odom", self.odom_callback, 10
        )
        self.ackermann_publisher = self.create_publisher(
            AckermannDriveStamped, "/drive", 1
        )

        self.laserscan_subscriber
        self.odom_subscriber

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
        pass

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        angle = scan_msg.angle_min
        break_bool = False
        for range in scan_msg.ranges:
            if range != math.nan or range != math.inf:
                speed_exp = self.speed * math.cos(angle)
                if speed_exp > 0:
                    ittc = range / speed_exp
                    if ittc < 1.5:
                        break_bool = True
                        break
            angle = angle + scan_msg.angle_increment
        # TODO: publish command to brake
        if break_bool == True:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            self.ackermann_publisher.publish(msg)

        pass


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
