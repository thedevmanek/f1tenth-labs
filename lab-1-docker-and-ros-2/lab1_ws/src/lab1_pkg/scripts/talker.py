#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class TalkerPublisher(Node):
  def __init__(self):
    super().__init__('talker')
    self.declare_parameter('v', 3.0)
    self.declare_parameter('d', 3.9)
    self.publisher_ = self.create_publisher( AckermannDriveStamped,'drive', 1)
    timer_period = 1  
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
      v = self.get_parameter('v')._value
      d = self.get_parameter('d')._value
      msg = AckermannDriveStamped()
      msg.drive.steering_angle = d
      msg.drive.speed = v
      self.get_logger().info(('Publishing \n msg %s' % (msg)))
      self.publisher_.publish(msg)

 
def main(args=None):

  rclpy.init(args=args)
  talker_publisher = TalkerPublisher()
  rclpy.spin(talker_publisher)
  talker_publisher.destroy_node() 
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()