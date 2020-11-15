#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
import math
from math import sin, cos, tan, atan2, atan, pi
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd
import os
import copy

class pioneer_control(object):
    def __init__(self):
        ####################
        # DECLARE PUBLISHERS
        ####################
        self.robot_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.move_goal_callback, queue_size=1)

        self.odometry_path_pub = rospy.Publisher('/path_odometry', Path, queue_size=1)
        self.gt_path_pub = rospy.Publisher('/path_ground_truth', Path, queue_size=1)
        self.vis_pub = rospy.Publisher('marker_goal', Marker, queue_size=1)
        
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
        self.left_wheel_angular_velocity = []
        self.right_wheel_angular_velocity = []

        self.odometry_path = []
        self.odometry_path_id = 0
        self.ground_truth_path = []
        self.gt_path_id = 0
        self.path_curvature = []
        self.vertical_plot_slow_down = []
        self.vertical_plot_transition = []
        self.slow_down_flag = []
        self.transition_flag = []
        self.speed_up_flag = []
        poses = [[0, 5, 0], [3.54, 3.54, 1.57/2], [5, 0, 0], [4.5, -2.17, -0.785], [2.5, -4.33, -1.57],
                 [0, -5, -1.57], [-3.54, -3.54, -2.36], [-5, 0, -3.14], [-3.54, 3.54, 0]]
        self.goal_pose = poses[1]
        self.path1 = [[1, 0, 0], [3, 0, 0], [3, -3, -2.36], [-3, -3, 1.57], [-3, 0, 1.57], [0, 0, 0]]
        self.path2 = [[1, 1, 1.57/2], [3, 1, 0], [3, -3, -2.36], [-3, -3, 1.57], [-3, 0, 1.57], [0, 0, 0]]
                
    def publish_goal(self, goal_pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 200
        marker.header.stamp = rospy.Time.now()
        marker.ns = "marker"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.pose.position.x = goal_pose[0]
        marker.pose.position.y = goal_pose[1]
        angle = quaternion_from_euler(0.0, 0.0, goal_pose[2])
        marker.pose.orientation.x = angle[0]
        marker.pose.orientation.y = angle[1]
        marker.pose.orientation.z = angle[2]
        marker.pose.orientation.w = angle[3]
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.vis_pub.publish(marker)

    def update_robot_gt_state(self, pose, orientation):
        self.gt_path_x.append(pose[0])
        self.gt_path_y.append(pose[1])
        self.gt_orientation.append(orientation)
        
    def update_odometry_state(self, pose, orientation):
        self.odometry_path_x.append(pose[0])
        self.odometry_path_y.append(pose[1])
        self.odometry_orientation.append(orientation)

    def publish_path_rviz_ground_truth(self, pose):
        """
        Publish the ground truth path to RVIZ

        Arguments:
            pose (list):Ground truth position (X and Y) of the robot
        """
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
        """
        Publish the odometry path to RVIZ

        Arguments:
            pose (list):Position X and Y of the robot
        """
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
        Get the goal data from the 2D Nav tool in Rviz
        """
        xt = msg.pose.position.x
        yt = msg.pose.position.y
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        thetat = euler_from_quaternion([x, y, z, w])[-1]
        # self.goal_pose = [xt, yt, thetat]        

    def get_robot_actual_pose(self):      
        """
        Get the actual robot pose from the robot based_link frame to the odom frame

        Returns:
            x (float):Position X of the robot
            y (float):Position Y of the robot
            theta (float):Orientation of the robot

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

        Arguments:
            x (float):Orientation angle (rad/s)

        Returns:
            rot_z_matrix (Numpy array):Rotation matrix around Z axis
        """
        rot_z_matrix = np.array([[cos(x), -sin(x)],
                                 [sin(x),  cos(x)]])
        return rot_z_matrix

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
        # plt.figure(1)
        # plt.plot(self.gt_path_x[0], self.gt_path_y[0], 'go')  # mark initial position
        # gt_plot = plt.plot(self.gt_path_x, self.gt_path_y, 'g', label='Ground truth')         # mark path

        # robot_plot = plt.plot(self.odometry_path_x, self.odometry_path_y, 'r', label='Odometria')

        # plt.title('Robot path comparison')
        # plt.legend()
        # plt.grid()
        # plt.tight_layout()
        
        plt.figure(2)
        plt.subplot(411)
        plt.plot(self.time_interval, self.slow_down_flag, 'r', label='Slow down flag')
        plt.plot(self.time_interval, self.transition_flag, 'k', label='Transition flag')
        plt.plot(self.time_interval, self.speed_up_flag, 'b', label='Speed up flag')
        plt.legend()
        plt.grid()

        plt.subplot(412)
        plt.plot(self.time_interval, self.robot_linear_velocity)
        
        for i in range(len(self.vertical_plot_slow_down)):
            plt.vlines(self.vertical_plot_slow_down[i], 0, 0.7, 'g', 'dashed')
        
        for i in range(len(self.vertical_plot_transition)):
            plt.vlines(self.vertical_plot_transition[i], 0, 0.7, 'r', 'dashdot')

        plt.xlabel('tempo (s)')
        plt.ylabel('Velocidade Linear (m/s)')
        plt.grid()
        plt.yticks(np.arange(0, 0.7, 0.1))

        plt.subplot(413)
        plt.xlabel('tempo (s)')
        plt.ylabel('Velocidade Angular (rad/s)')
        plt.grid()
        plt.plot(self.time_interval, self.robot_angular_velocity)

        for i in range(len(self.vertical_plot_slow_down)):
            plt.vlines(self.vertical_plot_slow_down[i], -0.5, 0.5, 'g', 'dashed')
        
        for i in range(len(self.vertical_plot_transition)):
            plt.vlines(self.vertical_plot_transition[i], -0.5, 0.5, 'r', 'dashed')

        plt.yticks(np.arange(-0.5, 0.5, 0.1))

        plt.subplot(414)
        plt.xlabel('tempo (s)')
        plt.ylabel('Coordenada X (m)')
        plt.yticks(np.arange(0, 3, 0.5))
        plt.grid()
        plt.plot(self.time_interval, self.odometry_path_x[0:-1])
        plt.show()
    
    def save_array_to_csv(self, string):
        """
        Save the coordinates into csv files
        """
        script_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'csv_files'))
        
        df = pd.DataFrame({"x" : self.gt_path_x, "y" : self.gt_path_y})
        df.to_csv(script_path + "/" + "ground_truth_path" + "_" + string + ".csv", index=False)

        df = pd.DataFrame({"x" : self.odometry_path_x, "y" : self.odometry_path_y})
        df.to_csv(script_path + "/" + "robot_path" + "_" + string + ".csv", index=False)

        print("len_time: ", len(self.time_interval))
        print("robot_linear_velocity: ", len(self.robot_linear_velocity))
        print("robot_angular_velocity: ", len(self.robot_angular_velocity))
        print("left_wheel_angular_velocity: ", len(self.left_wheel_angular_velocity))
        print("right_wheel_angular_velocity: ", len(self.right_wheel_angular_velocity))

        df = pd.DataFrame({"time_interval" : self.time_interval, 
                           "linear_velocity" : self.robot_linear_velocity, 
                           "angular_velocity" : self.robot_angular_velocity,
                           "left_wheel_angular" : self.left_wheel_angular_velocity,
                           "right_wheel_angular" : self.right_wheel_angular_velocity})
        df.to_csv(script_path + "/" + "robot_velocities" + "_" + string + ".csv", index=False)
    
    def diff_slow_down_model(self, vx, w):
        """
        Calculate the wheel angular velocities based on the linear and angular 
        velocities of the robot
        
        Arguments:
            vx (float):Linear velocity(m/s)
            w (float):Angular velocity(rad/s)

        Returns:
            we (float):Angular velocity of the left wheel (rad/s)
            wd (float):Angular velocity of the right wheel (rad/s)
            ve (float):Linear velocity of the left wheel (m/s)
            vd (float):Linear velocity of the right wheel (m/s)
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

        Arguments:
            vx (float):Linear velocity(m/s)
            w (float):Angular velocity(rad/s)
        """
        vel = Twist()
        vel.linear.x = vx
        vel.angular.z = w
        self.robot_cmd_vel.publish(vel)
    
    def odometry(self, vx, w, dt):
        """
        This method calculates and stores the odometry data for comparison
        
        Arguments:
            vx (float):Linear velocity(m/s)
            w (float):Angular velocity(rad/s)
            dt (float):Time interval
        """
        L = self.L
        r = self.r
        
        we, wd, ve, vd = self.diff_slow_down_model(vx, w)

        self.right_wheel_angular_velocity.append(wd)
        self.left_wheel_angular_velocity.append(we)

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
    
    def get_robot_state_control(self, goal_pose):
        """
        Calculate the robot states as heading angle, theta, phi, and distance rom the target

        Parameters:
            odometry_pose (list):Position and Orientation of the robot based on the odometry
        
        Returns:
            heading_angle (float):Angle between the robot and the line of sight between the robot and the target
            theta (float):Angle between the target orientation and the line of sight between the robot and the target
            disrt_r (float):Distance between the robot and the target
        """
        x = self.odometry_path_x[-1]
        y = self.odometry_path_y[-1]
        theta = self.odometry_orientation[-1]

        [xt, yt, target_angle] = goal_pose
        [xr, yr, robot_angle] = [x, y, theta]

        phi = atan2(yt - yr, xt - xr)
        
        heading_angle = robot_angle - phi # delta
        theta = target_angle - phi
        
        if heading_angle < -pi:
            heading_angle += 2*pi
        elif heading_angle > pi:
            heading_angle -= 2*pi
        
        if theta < -pi:
            theta += 2*pi
        elif theta > pi:
            theta -= 2*pi

        dist_r = np.linalg.norm(np.array((xt, yt)) - np.array((xr, yr)))

        return heading_angle, theta, dist_r
    
    
    def main_control(self):
        """
        Control the robot based on the Odometry data (calculated using the ICC method)
        and compares with the ground truth from Gazebo
        """
        v_max = 0.5 # linear velocity
        vx = v_max
        v_initial= 0
        w_initial = 0
                                
        raw_input("Choose a goal and then press Enter to start the simulation.")
        
        # Diminui o tempo para 0.01. O caminho final foi bem ruim
        # Com o dt = 0.05, obtive uma boa aprox.
        dt = 0.15 # Time interval
        dist_r = 10
        
        ###########################
        # Control parameters
        k1 = 1
        k2 = 3
        beta = 0.4
        lambda_ = 2
        ###########################
        
        ###########################
        # Parameters for transition
        # between points in the path
        threshold_distance_goal = 0.1
        
        transition_desired_velocity = 0.3
        
        threshold_slow_down_distance = 0.3
        slow_down_time_current = 0
        slow_down_time_limit = 0.5
        slow_down_mode = 0

        transition_threshold_distance = 0.1
        transition_time_current = 0
        transition_time_limit = 1
        transition_mode = 0

        speed_up_time_current = 0
        speed_up_time_limit = 1
        speed_up_mode = 0
        ###########################

        time = 0
        path_points = len(self.path2)
        path_iter = 0
        count_ = 0
        count2_ = 0
        count3_ = 0
        ended_slow_down_mode = 0
        ended_transition_mode = 0
        print("Path points: " + str(path_points))
        while path_iter < path_points:
            goal_pose = self.path2[path_iter]
            print("Goal pose: ", goal_pose)
            
            # Last odometry data
            _, _, dist_r = self.get_robot_state_control(goal_pose)
            
            while not rospy.is_shutdown() and dist_r > threshold_distance_goal:
                # Plot the goal in RVIZ
                self.publish_goal(goal_pose)

                # Stores the robot's simulated pose (Ground truth pose) into an array
                gt_pose = self.get_robot_actual_pose()

                self.publish_path_rviz_ground_truth(gt_pose)

                # Last odometry data
                heading_angle, theta, dist_r = self.get_robot_state_control(goal_pose)

                ##########################################
                # TRANSITIONS
                ##########################################
                #-------------------------------
                # STARTS THE SLOWING DOWN PROCESS
                #-------------------------------
                if transition_threshold_distance < dist_r and dist_r < threshold_slow_down_distance:
                    count_ += 1
                    slow_down_mode = 1 # Starts to slow down mode 
                    ended_slow_down_mode = 0
                    if count_ < 2:
                        # memorize the initial velocity and the time when the transitions starts
                        v_initial = vx
                        self.vertical_plot_slow_down.append(time)

                # The slow down occurs for a fixed period of time
                if slow_down_mode and slow_down_time_current < slow_down_time_limit:
                    slow_down_time_current += dt
                    ended_slow_down_mode = 0
                else:
                    # memorize when the time when the slow down stops
                    if slow_down_mode:
                        self.vertical_plot_slow_down.append(time)
                        slow_down_time_current = 0 # reset the transition time
                        v_initial = 0 # reset v_initial
                        slow_down_mode = 0 # slow down has ended
                        count_ = 0
                        w_initial = w
                        ended_slow_down_mode = 1
                
                #-------------------------------
                # STARTS THE TRANSITION PROCESS
                #-------------------------------
                if ended_slow_down_mode:
                    count2_ += 1
                    transition_mode = 1 # A transition starts
                    if count2_ < 2:
                        # w_intial = w
                        self.vertical_plot_transition.append(time)
                
                # The transition occurs for a fixed period of time
                if transition_mode and transition_time_current < transition_time_limit:
                    transition_time_current += dt
                    ended_transition_mode = 0
                else:
                    if count2_ > 0:
                        self.vertical_plot_transition.append(time)
                        transition_time_current = 0
                        transition_mode = 0
                        count2_ = 0
                        ended_transition_mode = 1
                        v_initial2 = vx
                        w_initial2 = w
                        ended_slow_down_mode = 0
                
                #-------------------------------
                # STARTS THE SPEED UP LINEAR VELOCITY AGAIN
                #-------------------------------
                if ended_transition_mode:
                    count3_ += 1
                    speed_up_mode = 1

                if speed_up_mode and speed_up_time_current < speed_up_time_limit:
                    speed_up_time_current += dt
                else:
                    if count3_ > 0:
                        speed_up_time_current = 0
                        speed_up_mode = 0
                        count3_ = 0
                        ended_transition_mode = 0

                self.slow_down_flag.append(slow_down_mode)
                self.transition_flag.append(transition_mode)
                self.speed_up_flag.append(speed_up_mode)

                ##########################################
                # Calculate the control signals
                ##########################################
                if path_iter + 1 < path_points:
                    heading_angle_next, theta_next, dist_next = self.get_robot_state_control(self.path2[path_iter+1])
                else:
                    heading_angle_next, theta_next, dist_next = self.get_robot_state_control(self.path2[-1])
                
                curvature = -1/dist_r * (k2*(heading_angle - atan(-k1*theta)) + (1 + k1/(1+(k1*theta)**2))*sin(heading_angle))

                if slow_down_mode:
                    # during the transition, the linear velocity is held constant
                    sigmoid = 1/0.98 * (1 / (1 + math.exp(-9.2*(slow_down_time_current/slow_down_time_limit - 0.5))) - 0.01)
                    vx = vel_transition = (1 - sigmoid) * v_initial + sigmoid * transition_desired_velocity
                    w = curvature * vx # angular velocity
                elif transition_mode:
                    vx = transition_desired_velocity                    
                    sigmoid = 1/0.98 * (1 / (1 + math.exp(-9.2*(transition_time_current/transition_time_limit - 0.5))) - 0.01)                    
                    curvature_next = -1/dist_next * (k2*(heading_angle_next - atan(-k1*theta_next)) + (1 + k1/(1+(k1*theta_next)**2))*sin(heading_angle_next))

                    w_final = curvature_next * transition_desired_velocity
                    w = vel_transition = (1 - sigmoid) * w_initial + sigmoid * w_final
                elif speed_up_mode:
                    curvature_next = -1/dist_next * (k2*(heading_angle_next - atan(-k1*theta_next)) + (1 + k1/(1+(k1*theta_next)**2))*sin(heading_angle_next))
                    sigmoid = 1/0.98 * (1 / (1 + math.exp(-9.2*(speed_up_time_current/speed_up_time_limit - 0.5))) - 0.01)
                    
                    vx_final2 = v_max / (1 + beta*curvature**lambda_) # linear velocity
                    vx = vel_transition = (1 - sigmoid) * v_initial2 + sigmoid * vx_final2

                    w_initial2 = curvature_next * transition_desired_velocity
                    w_final = curvature * vx # angular velocity
                    w = vel_transition = (1 - sigmoid) * w_initial2 + sigmoid * w_final
                else:
                    vx = v_max / (1 + beta*curvature**lambda_) # linear velocity
                    w = curvature * vx # angular velocity

                ##########################################
                # Update states and publish to robot
                ##########################################

                self.robot_linear_velocity.append(vx)
                self.robot_angular_velocity.append(w)

                # Update the robot ground truth state
                self.update_robot_gt_state(gt_pose[0:2], gt_pose[-1])

                # Robot's odometry
                self.odometry(vx, w, dt)

                # Send the linear and angular velocity to the robot
                self.publish_robot_vel(vx, w)

                time += dt
                self.time_interval.append(time) # Used to plot the data

                rospy.sleep(dt)

            path_iter += 1
            print("Path_iter: ", path_iter)
            
        self.publish_robot_vel(0, 0) # Stop the robot
        self.save_array_to_csv("path") # Save the coordinates in csv files
        self.plot_path() # plot the paths

def main():
    rospy.init_node('pioneer_vel_control')
    pioneer_vel_control = pioneer_control()
    
    # rospy.on_shutdown is used to stop the robot in cased of the node is killed
    rospy.on_shutdown(pioneer_vel_control.stop_on_shutdown)
    pioneer_vel_control.main_control()
    
if __name__ == "__main__":
    main()

# print("Angular velocity: ", w)
# print("Linear velocity: ", vx)
# print("Dist_r: ", dist_r)
# print("Curvature: ", curvature)
# print('\n')