#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Publisher(Node):
    def __init__(self):
        super().__init__('Publisher')
        self.publisher_ = self.create_publisher(Twist, 'etruck', 10)

        # Motor speed variables
        self.speed_forward = 0.0
        self.speed_backward = 0.0
        self.step_forward = 0.1
        self.step_backward = -0.1
        self.speed = 0.0

        # Steering variables
        self.steer_left = 0.0
        self.steer_right = 0.0
        self.steer_step = 0.1
        self.steering = 0.0  # angular.z

        self.get_logger().info("Controls:")
        self.get_logger().info("'f' = faster forward")
        self.get_logger().info("'b' = faster backward")
        self.get_logger().info("'s' = stop")
        self.get_logger().info("'l' = steer left")
        self.get_logger().info("'r' = steer right")
        self.get_logger().info("'c' = center steering")
        self.get_logger().info("'q' = quit")

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        key = input("Enter command (f/b/s/l/r/c/q): ").strip()

        # Motor control
        if key == 'q':
            self.get_logger().info('Quitting...')
            rclpy.shutdown()
            return
        elif key == 'f':
            self.speed_forward += self.step_forward
            self.speed = self.speed_forward
            self.speed_backward = 0.0  # reset backward when going forward
        elif key == 'b':
            self.speed_backward += self.step_backward  # note: step_backward is negative
            self.speed = self.speed_backward
            self.speed_forward = 0.0  # reset forward when going backward
        elif key == 's':
            self.speed_forward = 0.0
            self.speed_backward = 0.0
            self.speed = 0.0

        # Steering control
        elif key == 'l':
            self.steering -= self.steer_step
        elif key == 'r':
            self.steering += self.steer_step
        elif key == 'c':
            self.steering = 0.0

        # Clamp values
        self.speed = max(-1.0, min(1.0, self.speed))
        self.steering = max(-1.0, min(1.0, self.steering))

        # Publish message
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.steering
        self.publisher_.publish(msg)

        self.get_logger().info(f"Speed: {self.speed:.2f}, Steering: {self.steering:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
