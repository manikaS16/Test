#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class Subscriber(Node):
    def __init__(self):
        super().__init__('Subscriber')
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Change port if needed
        self.subscription = self.create_subscription(
            Twist,
            'etruck',
            self.listener_callback,
            10)
        self.get_logger().info('Subscriber started')

    def listener_callback(self, msg):
        speed = msg.linear.x
        angle = int(90 + speed * 45)
        angle = max(0, min(180, angle))
        self.ser.write(f"{angle}\n".encode())
        self.get_logger().info(f"Sent angle: {angle}")

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
