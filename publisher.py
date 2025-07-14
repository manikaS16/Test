#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Publisher(Node):
    def __init__(self):
        super().__init__('Publisher')
        self.publisher_ = self.create_publisher(Twist, 'etruck', 10)
        self.speed_forward = 0.5
        self.speed_backward = -0.5
        self.step_forward = 0.1
        self.step_backward = -0.1
        self.speed = 0.0

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
            self.speed_forward += self.step_forward
            self.speed = self.speed_forward
        elif key == 'b':
            self.speed_backward -= self.step_backward
            self.speed = self.speed_backward
        elif key == 's':
            self.speed_forward = 0.0
            self.speed_backward = 0.0
            self.speed = 0.0
        
        msg = Twist()    
        msg.linear.x = self.speed
        self.speed = max(-1.0, min(1.0, self.speed))
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
