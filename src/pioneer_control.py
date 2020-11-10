#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from math import sin, cos, tan, atan2, atan
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

        self.odometry_path_pub = rospy.Publisher('/path_odometry', Path, queue_size=10)
        self.gt_path_pub = rospy.Publisher('/path_ground_truth', Path, queue_size=10)
        
        ####################
        # ROBOT PARAMETERS
        ####################
        self.L = 0.316 # distance between wheels
        self.r = 0.09 # wheel radius

        ####################
        # ROS 
        ####################
        self.transf = TransformListener()

        ####################
        # ROBOT STATE
        ####################
        # Initiate robot state
        odometry_position = self.get_robot_actual_pose()
        # Ground truth path
        self.gt_path_x = [odometry_position[0]]
        self.gt_path_y = [odometry_position[1]]
        self.gt_orientation = [odometry_position[-1]]
        # The initial pose of the odometry is set equal to the actual robot's pose
        self.odometry_path_x = [odometry_position[0]] 
        self.odometry_path_y = [odometry_position[1]]
        self.odometry_orientation = [odometry_position[-1]]
        self.robot_linear_velocity = []
        self.robot_angular_velocity = []
        self.time_interval = []

        self.odometry_path = []
        self.odometry_path_id = 0
        self.ground_truth_path = []
        self.gt_path_id = 0

    def update_robot_gt_state(self, pose, orientation):
        self.gt_path_x.append(pose[0])
        self.gt_path_y.append(pose[1])
        self.gt_orientation.append(orientation)
        
    def update_odometry_state(self, pose, orientation):
        self.odometry_path_x.append(pose[0])
        self.odometry_path_y.append(pose[1])
        self.odometry_orientation.append(orientation)

    def publish_path_rviz_ground_truth(self, pose):
        # Publish path in RVIZ
        navPath = Path()
        navPath.header.frame_id = "map"
        pos = PoseStamped()
        pos.header.stamp = rospy.Time(0)
        pos.pose.position.x = pose[0]
        pos.pose.position.y = pose[1]
        self.ground_truth_path.append(pos)
        navPath.poses = self.ground_truth_path
        self.gt_path_pub.publish(navPath)
    
    def publish_path_rviz_odometry(self, pose):
        # Publish path in RVIZ
        navPath = Path()
        navPath.header.frame_id = "map"
        pos = PoseStamped()
        pos.header.stamp = rospy.Time(0)
        pos.pose.position.x = pose[0]
        pos.pose.position.y = pose[1]
        self.odometry_path.append(pos)
        navPath.poses = self.odometry_path
        self.odometry_path_pub.publish(navPath)
    
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
        plt.figure(1)
        plt.plot(self.gt_path_x[0], self.gt_path_y[0], 'go')  # mark initial position
        gt_plot = plt.plot(self.gt_path_x, self.gt_path_y, 'g', label='Ground truth')         # mark path

        robot_plot = plt.plot(self.odometry_path_x, self.odometry_path_y, 'r', label='Odometria')

        plt.title('Robot path comparison')
        plt.legend()
        plt.grid()
        plt.tight_layout()
        
        plt.figure(2)
        plt.subplot(211)
        plt.plot(self.time_interval, self.robot_linear_velocity)
        plt.xlabel('tempo (s)')
        plt.ylabel('Velocidade Linear (m/s)')
        plt.grid()
        plt.subplot(212)
        plt.xlabel('tempo (s)')
        plt.ylabel('Velocidade Angular (rad/s)')
        plt.grid()
        plt.plot(self.time_interval, self.robot_angular_velocity)

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

        df = pd.DataFrame({"time_interval" : self.time_interval, 
                           "linear_velocity" : self.robot_linear_velocity, 
                           "angular_velocity" : self.robot_angular_velocity})
        df.to_csv(script_path + "/" + "robot_velocities.csv", index=False)
    
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
    
    def odometry(self, vx, w, dt):
        """
        This method calculates and stores the odometry and the robot's pose from Gazebo 
        as arrays for comparison
        """
        L = self.L
        r = self.r
        
        we, wd, ve, vd = self.diff_model(vx, w)

        # Last odometry data
        x = self.odometry_path_x[-1]
        y = self.odometry_path_y[-1]
        theta = self.odometry_orientation[-1]
        pose = [x, y, theta]

        if vd == ve:
            pose = pose + np.array([vx*cos(theta)*dt, vx*sin(theta)*dt, 0])
            theta = pose[-1]
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
            
        # We need to update the odometry state
        self.update_odometry_state(pose, theta)
        self.publish_path_rviz_odometry(pose)
    
    def get_robot_state_control(self, odometry_pose):
        [xt, yt, target_angle] = self.goal_pose
        [xr, yr, robot_angle] = odometry_pose

        phi = atan2(yt - yr, xt - xr)
        heading_angle = robot_angle - phi
        theta = target_angle - phi
        dist_r = np.linalg.norm(np.array((xt, yt)) - np.array((xr, yr)))

        return heading_angle, theta, dist_r
    
    def control_loop(self, v_max):
        """
        Calculates the linar and angular velocity based on the paper entitled
        "A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment" (Park and Kuipers, 2011)
        
        Arguments:
            v_max {float}: angular velocity
        """
        # Last odometry data
        x = self.odometry_path_x[-1]
        y = self.odometry_path_y[-1]
        theta = self.odometry_orientation[-1]
        pose = [x, y, theta]

        heading_angle, theta, dist_r = self.get_robot_state_control(pose)

        k1 = 1
        k2 = 3

        curvature = -1/dist_r * (k2*(heading_angle - atan(-k1*theta)) + (1 + k1/(1+(k1*theta)**2))*sin(heading_angle))
        
        beta = 0.4
        lambda_ = 4
        vx = v_max / (1 + beta*curvature**lambda_)
        w = curvature * vx

        self.robot_linear_velocity.append(vx)
        self.robot_angular_velocity.append(w)
        
        # print('Distance: ', dist_r)
        # print('Angular velocity: ', w)
        
        return vx, w, dist_r
    
    def main_control(self):
        """
        Calculate the robot path based on the ICC method
        """
        v_max = 0.3 # linear velocity
                                
        raw_input("Choose a goal and then press Enter to start the simulation.")
        
        # Diminui o tempo para 0.01. O caminho final foi bem ruim
        # Com o dt = 0.05, obtive uma boa aprox.
        dt = 0.08 # Time interval
        
        dist_r = 10
        thresh = 0.05
        time = 0
        while not rospy.is_shutdown() and dist_r > thresh:
            # Stores the robot's simulated pose (Ground truth pose) into an array
            gt_pose = self.get_robot_actual_pose()

            self.publish_path_rviz_ground_truth(gt_pose)
            
            # Calculate the control's signal
            vx, w, dist_r = self.control_loop(v_max)
            
            # Send the linear and angular velocity to the robot
            self.publish_robot_vel(vx, w)
            
            # Update the robot ground truth state
            self.update_robot_gt_state(gt_pose[0:2], gt_pose[-1])
            
            # Robot's odometry
            self.odometry(vx, w, dt)

            time += dt
            self.time_interval.append(time)

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