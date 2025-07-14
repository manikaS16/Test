#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import serial

class ESCController:
    def __init__(self, port='/dev/ttyUSB0', baud=9600):
        self.ser = serial.Serial(port, baud, timeout=1)
        rospy.Subscriber('/etruck', Twist, self.callback)
        rospy.loginfo("[Subscriber] ESC Controller initialized.")

    def callback(self, msg):
        speed = msg.linear.x  # -1.0 to 1.0
        angle = int(90 + speed * 45)  # Map speed to servo angle (45-135)
        angle = max(0, min(180, angle))
        command = f"{angle}\n"
        self.ser.write(command.encode())
        rospy.loginfo(f"[Subscriber] Sent angle: {angle}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('etruck_subscriber')
    controller = ESCController()
    controller.run()
