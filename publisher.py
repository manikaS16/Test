#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import keyboard

def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))

def main():
    rospy.init_node('etruck_publisher')
    pub = rospy.Publisher('/etruck', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    speed = 0.0
    step = 0.1  # Increment per key press

    print("Controls: 'f'=faster forward | 'b'=faster backward | 's'=stop | 'q'=quit")

    while not rospy.is_shutdown():
        if keyboard.is_pressed('q'):
            break
        elif keyboard.is_pressed('f'):
            speed += step
        elif keyboard.is_pressed('b'):
            speed -= step
        elif keyboard.is_pressed('s'):
            speed = 0.0

        speed = clamp(speed, -1.0, 1.0)

        msg = Twist()
        msg.linear.x = speed
        pub.publish(msg)
        rospy.loginfo(f"[Publisher] Speed set to: {speed:.2f}")
        rate.sleep()

if __name__ == '__main__':
    main()
