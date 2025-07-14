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
        motor_angle = int(90 + speed * 45)
        motor_angle = max(45, min(135, motor_angle))  # Limit angles to safe range

        # Map steering from [-1, 1] to [45, 135] degrees for steering servo
        steer = msg.angular.z
        steer_angle = int(90 + steer * 45)
        steer_angle = max(45, min(135, steer_angle))

        # Send motor and steer angles separated by comma
        send_str = f"{motor_angle},{steer_angle}\n"
        self.ser.write(send_str.encode())

        self.get_logger().info(f"Sent motor_angle: {motor_angle}, steer_angle: {steer_angle}")

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
