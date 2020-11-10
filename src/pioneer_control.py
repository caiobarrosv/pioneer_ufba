#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from math import sin, cos, tan, atan2
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd
import os

class pioneer_control(object):
    def __init__(self):
        ####################
        # DECLARE PUBLISHERS
        ####################
        self.robot_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.move_goal_callback, queue_size=10)
        
        ####################
        # ROBOT PARAMETERS
        ####################
        self.L = 0.316 # distance between wheels
        self.r = 0.09 # wheel radius

        ####################
        # ROS 
        ####################
        self.transf = TransformListener()

    def move_goal_callback(self, msg):
        """
        Acquire goal
        """
        xt = msg.pose.position.x
        yt = msg.pose.position.y
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        thetat = euler_from_quaternion([x, y, z, w])[-1]
        self.goal_pose = [xt, yt, thetat]

        # print("Goal position: ", self.goal_pose)
        # [x, y, theta] = self.get_robot_actual_pose()
        # heading_angle, theta = self.get_robot_state_control(xt, yt, thetat, x, y, theta)
        # print("Heading angle: ", heading_angle)
        # print("Theta: ", theta)

    def get_robot_state_control(self, xt, yt, target_angle, xr, yr, robot_angle):
        """
        robot_angle : Angle of the robot related to the map frame
        target_angle : Angle of the target related to the map frame
        """
        phi = atan2(yt - yr, xt - xr)
        heading_angle = robot_angle - phi
        theta = target_angle - phi

        return heading_angle, theta
    
    def control_loop(self, vx, heading_angle, theta):
        dr = -vx * cos(heading_angle)
        dtheta = v/r * sin(heading_angle)
        dheading_angle = v/r * sin(heading_angle) + omega

    def get_robot_actual_pose(self):      
        """
        Acquire the actual robot pose
        """  
        self.transf.waitForTransform("odom", "base_link", rospy.Time(0), rospy.Duration(5.0)) 
        robot_initial_position, robot_initial_angle = self.transf.lookupTransform("odom", "base_link", rospy.Time(0))

        x = robot_initial_position[0]
        y = robot_initial_position[1]
        theta = euler_from_quaternion(robot_initial_angle)[-1] # Obtain the robot angle theta around the Z axis (Yaw)

        return [x, y, theta]

    def rot_z_2d(self, x):
        """
        Rotation matrix around the Z axis
        """
        rot_z_matrix = np.array([[cos(x), -sin(x)],
                                 [sin(x),  cos(x)]])
        return rot_z_matrix

    def np_transp(self, matrix):
        """
        Transforms the list into array and transpose it
        """
        array = np.transpose(np.array(matrix))
        return array
    
    def stop_on_shutdown(self):
        """
        Stop the robot if the node is killed
        """
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.robot_cmd_vel.publish(vel)

    def plot_path(self):
        """
        Plot the robot ground truth and simulated path
        """
        plt.plot(self.gt_path_x[0], self.gt_path_y[0], 'bo')  # mark initial position
        gt_plot = plt.plot(self.gt_path_x, self.gt_path_y, 'r', label='Ground truth')         # mark path

        robot_plot = plt.plot(self.odometry_path_x, self.odometry_path_y, 'k', label='Odometria')

        plt.title('Robot path comparison')
        plt.legend()
        plt.grid()
        plt.tight_layout()
        plt.show()
    
    def save_array_to_csv(self):
        """
        Save the coordinates into csv files
        """
        script_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'csv_files'))
        
        df = pd.DataFrame({"x" : self.gt_path_x, "y" : self.gt_path_y})
        df.to_csv(script_path + "/" + "ground_truth_path.csv", index=False)

        df = pd.DataFrame({"x" : self.odometry_path_x, "y" : self.odometry_path_y})
        df.to_csv(script_path + "/" + "robot_path.csv", index=False)
    
    def diff_model(self, vx, w):
        """
        Calculate the wheel angular velocities based on the linear and angular 
        velocities of the robot
        """
        L = self.L
        r = self.r
        A = np.array([[r/2, r/2], [-r/L, r/L]])
        wheel_vel = np.matmul(np.linalg.inv(A), np.array([vx, w]))
        # essa ordem eh devido ao sinal negativo na segunda linha e primeira coluna da matriz A
        we = wheel_vel[0] 
        wd = wheel_vel[1]
        ve = we * r
        vd = wd * r
        return we, wd, ve, vd
    
    def publish_robot_vel(self, vx, w):
        """
        Send the velocity command to the robot
        """
        vel = Twist()
        vel.linear.x = vx
        vel.angular.z = w
        self.robot_cmd_vel.publish(vel)
    
    def odometry(self, vx, w, dt, pose, theta, gt_pose):
        """
        This method calculates and stores the odometry and the robot's pose from Gazebo 
        as arrays for comparison
        """
        L = self.L
        r = self.r

        self.gt_path_x.append(gt_pose[0])
        self.gt_path_y.append(gt_pose[1])
        we, wd, ve, vd = self.diff_model(vx, w)

        if vd == ve:
            pose = pose + np.array([vx*cos(theta)*dt, vx*sin(theta)*dt, 0])
            theta = pose[-1]
            self.odometry_path_x.append(pose[0])
            self.odometry_path_y.append(pose[1])
        else:
            R = (L / 2) * ((vd + ve) / (vd - ve))
            omega = (vd - ve) / L
            
            ICC_x = pose[0] - R*sin(theta)
            ICC_y = pose[1] + R*cos(theta)
            ICC = np.array([ICC_x, ICC_y])
            rot_z_mat = self.rot_z_2d(omega*dt)
            theta = theta + omega*dt
            
            ICC_origin = np.array([pose[0] - ICC_x, pose[1] - ICC_y])
            pose = np.matmul(rot_z_mat, ICC_origin) + ICC
            
            self.odometry_path_x.append(pose[0])
            self.odometry_path_y.append(pose[1])
        return pose, theta
    
    def main_control(self):
        """
        Calculate the robot path based on the ICC method
        """

        odometry_pose = self.get_robot_actual_pose()
        odometry_theta = odometry_pose[-1] # initial robot orientation
        
        self.gt_path_x = [odometry_pose[0]]
        self.gt_path_y = [odometry_pose[1]]
        # The initial pose of the odometry is set equal to the actual robot's pose
        self.odometry_path_x = [odometry_pose[0]] 
        self.odometry_path_y = [odometry_pose[1]]
        
        vx = 0.5 # linear velocity
        w = 0.4 # angular velocity
                        
        raw_input("Choose a goal and then press Enter to start the simulation.")
        self.publish_robot_vel(vx, w)
        t_final = rospy.get_rostime().secs
        t_inicial = rospy.get_rostime().secs
        dt = 0.01
        t_lim = 0
        tempo_de_simulacao = 10
        while not rospy.is_shutdown() and t_lim < tempo_de_simulacao:
            # Stores the robot's simulated pose (Ground truth pose) into an array
            gt_pose = self.get_robot_actual_pose()
            
            odometry_pose, odometry_theta = self.odometry(vx, w, dt, odometry_pose, odometry_theta, gt_pose)

            t_final = rospy.get_rostime().secs
            t_lim = (t_final - t_inicial)
            print(t_lim)
            rospy.sleep(dt)

        self.publish_robot_vel(0, 0) # Stop the robot
        self.save_array_to_csv() # Save the coordinates in csv files
        self.plot_path() # plot the paths

def main():
    rospy.init_node('pioneer_vel_control')
    pioneer_vel_control = pioneer_control()
    
    rospy.on_shutdown(pioneer_vel_control.stop_on_shutdown)
    pioneer_vel_control.main_control()
    
if __name__ == "__main__":
    main()