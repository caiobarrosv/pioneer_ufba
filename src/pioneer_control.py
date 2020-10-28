#!/usr/bin/env python
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from std_msgs.msg import Float32
from math import sin, cos, tan
from time import sleep

class pioneer_control(object):
    def __init__(self):
        self.left_wheel_pub = rospy.Publisher('/pioneer_3dx/left_wheel_speed', Float32, queue_size=10)
        self.right_wheel_pub = rospy.Publisher('/pioneer_3dx/right_wheel_speed', Float32, queue_size=10)

        # Wait for some time before sending commands to the wheels and then
        # initialize the wheels velocities
        rospy.sleep(0.4)
        self.left_wheel_pub.publish(0.0)
        self.right_wheel_pub.publish(0.0)

    def main_control(self):
        while not rospy.is_shutdown():
            self.left_wheel_pub.publish(0.5)
            rospy.sleep(3)
            self.left_wheel_pub.publish(0.0)
            rospy.sleep(3)

def main():
    rospy.init_node('pioneer_vel_control')
    pioneer_vel_control = pioneer_control()
    pioneer_vel_control.main_control()
    
if __name__ == "__main__":
    main()