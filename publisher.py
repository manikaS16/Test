#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Publisher(Node):
    def __init__(self):
        super().__init__('Publisher')
        self.publisher_ = self.create_publisher(Twist, 'etruck', 10)

        # Motor speed variables
        self.motor_forward = 0.0
        self.motor_backward = 0.0
        self.motor_step_forward = 0.1
        self.motor_step_backward = -0.1
        self.motor = 0.0

        # Steering variables
        self.steer_left = 0.0
        self.steer_right = 0.0
        self.steer_step_left = 0.1
        self.steer_step_right = 0.1
        self.steer = 0.0

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
            self.motor_forward += self.motor_step_forward
            self.motor = self.motor_forward
            self.motor_backward = 0.0
        elif key == 'b':
            self.motor_backward += self.motor_step_backward
            self.motor = self.motor_backward
            self.motor_forward = 0.0
        elif key == 's':
            self.speed_forward_motor = 0.0
            self.speed_backward_motor = 0.0
            self.speed_motor = 0.0

        # Steering control
        elif key == 'l':
            self.steer_left -= self.steer_step_left
            self.steer = self.steer_left
        elif key == 'r':
            self.steer_right += self.steer_step_right
            self.steer = self.steer_right
        elif key == 'c':
            self.steer = 0.0

        # Clamp values
        self.motor = max(-1.0, min(1.0, self.motor))
        self.steer = max(-1.0, min(1.0, self.steer))

        # Publish message
        msg = Twist()
        msg.linear.x = self.motor
        msg.angular.z = self.steer
        self.publisher_.publish(msg)

        self.get_logger().info(f"Speed: {self.motor:.2f}, Steering: {self.steer:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
