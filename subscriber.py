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
        # Map speed from [-1, 1] to [45, 135] degrees for motor
        speed = msg.linear.x
        steer = msg.angular.z
        # Clamp values to avoid overflow
        speed = max(-1.0, min(1.0, speed))
        steer = max(-1.0, min(1.0, steer))

        # Send raw float values to Arduino
        send_str = f"{speed:.2f},{steer:.2f}\n"
        self.ser.write(send_str.encode())

        self.get_logger().info(f"Sent speed: {speed:.2f}, steer: {steer:.2f}")

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
