#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Twist, 'etruck', 10)
        self.speed = 0.0
        self.step = 0.1

        self.get_logger().info("Press keys in terminal to control:")
        self.get_logger().info("'f' = forward, 'b' = backward, 's' = stop, 'q' = quit")

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        key = input("Enter command (f/b/s/q): ").strip()
        if key == 'q':
            self.get_logger().info('Quitting...')
            rclpy.shutdown()
            return
        elif key == 'f':
            self.speed += self.step
        elif key == 'b':
            self.speed -= self.step
        elif key == 's':
            self.speed = 0.0

        self.speed = max(-1.0, min(1.0, self.speed))

        msg = Twist()
        msg.linear.x = self.speed
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published speed: {self.speed:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
