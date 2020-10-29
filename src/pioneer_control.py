#!/usr/bin/env python
import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from std_msgs.msg import Float32
from math import sin, cos, tan
from tf import TransformListener

class pioneer_control(object):
    def __init__(self):
        self.left_wheel_pub = rospy.Publisher('/pioneer_3dx/left_wheel_speed', Float32, queue_size=10)
        self.right_wheel_pub = rospy.Publisher('/pioneer_3dx/right_wheel_speed', Float32, queue_size=10)

        # Wait for some time before sending commands to the wheels and then
        # initialize the wheels velocities
        rospy.sleep(0.4)
        self.left_wheel_pub.publish(0.0)
        self.right_wheel_pub.publish(0.0)

        self.transf = TransformListener()
        self.transf.waitForTransform("base_link", "map", rospy.Time(0), rospy.Duration(4.0)) # rospy.Time.now()

    def main_control(self):
        while not rospy.is_shutdown():
            gt_link_pose, _ = self.transf.lookupTransform("base_link", "map", rospy.Time(0))
            link_pose, _ = self.transf.lookupTransform("base_link", "map", rospy.Time(0))
            print("Gt_link_pose:", gt_link_pose)
            print("link_pose:", link_pose)
            
            self.left_wheel_pub.publish(0.5)
            rospy.sleep(0.2)
            self.right_wheel_pub.publish(0)
            rospy.sleep(0.2)
            

def main():
    rospy.init_node('pioneer_vel_control')
    pioneer_vel_control = pioneer_control()
    pioneer_vel_control.main_control()
    
if __name__ == "__main__":
    main()