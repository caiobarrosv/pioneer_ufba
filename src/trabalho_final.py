#!/usr/bin/env python

# ROS Imports
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from visualization_msgs.msg import Marker
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Python imports
from rtree import index
import random
import csv
import pandas as pd
import os
import copy
import numpy as np
import matplotlib.pyplot as plt
import math
from math import sin, cos, tan, atan2, atan, pi, isnan, sqrt
import cv2

rospy.init_node('pioneer_vel_control')

class Tree(object):
	def __init__(self, num_of_dimensions):
		"""
		Tree representation
		:param X: Search Space
		"""
		p = index.Property()
		p.dimension = num_of_dimensions
		self.V = index.Index(interleaved=True, properties=p)  # vertices in an rtree
		self.V_count = 0
		self.E = {}  # edges in form E[child] = parent

class RRT_Test(object):
	def __init__(self):
		# SUBSCRIBERS
		map_topic = "/map"
		rospy.Subscriber(map_topic, OccupancyGrid, self.mapCallBack, queue_size=1)
		rospy.sleep(1.0)
		
		# PUBLISHERS
		self.path_pub = rospy.Publisher("/Path", Path, queue_size=10)
		self.marker_pub = rospy.Publisher("/marker", Marker, queue_size=10)
		self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
		self.edge_length = 20
		self.dist_to_goal = 50.0
		self.r = 0.5 # distance from X_NEAREST to X_NEW
		self.max_samples = 10000
		self.samples_taken = 0
		self.prc = 0.3
		self.height = 200 #185 # altura do mapa na imagem grande de 2000x2000
		self.width = 200 #198 # largura do mapa na imagem grande de 2000x2000
		self.img_dim = [self.height, self.width]
		self.dimension_offset = [1977, 1888] 
		self.X_dimensions = np.array([(0, self.img_dim[0]), (0, self.img_dim[1])])
		self.num_of_dimensions = len(self.X_dimensions)
		self.trees = []  
		self.add_tree()
		self.marker_array = []
		self.index = 0
		self.map_resolution = 0.05
		self.invert_coordinates = -1 # -1 = invert | 1 = not inverted
		self.tf = TransformListener()

	def mapCallBack(self, msg):
		self.mapData = msg
	
	def add_tree(self):
		self.trees = []
		self.trees.append(Tree(self.num_of_dimensions))
	
	def build_rrt(self, plot):
		self.add_vertex(0, self.pose_initial)
		self.add_edge(0, self.pose_initial, None)

		while not rospy.is_shutdown():
			x = self.sample()
			x_new, x_nearest = self.new_and_near(0, x)

			if x_new is None:
				continue

			self.show_edge_path_rviz(x_new, x_nearest) # novo, arvore		

			status = self.connect_to_point(0, x_nearest, x_new)
			solution = self.check_solution()

			if solution[0]:
				return solution[1]

			# check if can connect to goal after generating max_samples
			if self.samples_taken >= self.max_samples:
				return self.get_path()

	def add_vertex(self, tree, new):
		self.trees[tree].V.insert(0, new + new, new)
		self.trees[tree].V_count += 1  # increment number of vertices in tree
		self.samples_taken += 1  # increment number of samples taken

	def add_edge(self, tree, new, nearest):
		self.trees[tree].E[new] = nearest

	def sample(self):
		x = np.random.uniform(self.X_dimensions[:, 0], self.X_dimensions[:, 1])
		
		x = [int(x[i] + self.dimension_offset[i]) for i in range(2)] # 2D
		return tuple(x)

	def new_and_near(self, tree, x_rand):
		x_nearest = self.get_nearest(tree, x_rand)

		x_new = self.bound_point(self.steer(x_nearest, x_rand, self.edge_length))

		if not self.obstacle_free(x_new):
			return None, None

		self.samples_taken += 1

		x_new = tuple([int(x_new[i]) for i in range(2)]) # 2D
		x_nearest = tuple([int(x_nearest[i]) for i in range(2)]) # 2D
		
		return x_new, x_nearest

	def nearby(self, tree, x, n):
		return self.trees[tree].V.nearest(x, num_results=n, objects="raw") # from rtree module

	def get_nearest(self, tree, x):
		return next(self.nearby(tree, x, 1))

	def bound_point(self, point):
		inf_lim = self.dimension_offset	
		sup_lim = [self.X_dimensions[i, 1] + self.dimension_offset[i] for i in range(2)] 
		point = np.maximum(point, inf_lim) # returns the maximum of each position
		point = np.minimum(point, sup_lim)
		return tuple(point)

	def steer(self, start, goal, d):
		start, end = np.array(start), np.array(goal)
		v = end - start
		u = v / (np.sqrt(np.sum(v ** 2))) # unit vector
		steered_point = start + u * d
		return tuple(steered_point) # resized node in the direction pointed by u

	def obstacle_free(self, x):
		x = list(x)
		if x[0] < 300:
			x = [int(x[i] + self.dimension_offset[i]) for i in range(2)] # 2D

		if not isnan(x[0]):
			x = [int(x[i]) for i in range(2)] # 2D
		
			if self.img[x[0], x[1]] > 200: 
				return True
		else:
			return False	

	def dist_between_points(self, a, b):
		distance = np.linalg.norm(np.array(b) - np.array(a))
		return distance

	def es_points_along_line(self, start, end):
		d = self.dist_between_points(start, end)
		n_points = int(np.ceil(d / self.r))
		if n_points > 1:
			step = d / (n_points - 1)
			for i in range(n_points):
				next_point = self.steer(start, end, i * step)
				yield next_point 

	def collision_free(self, start, end):
		# Test each point on the line linking the new to the nearest
		points = self.es_points_along_line(start, end)
		coll_free = all(map(self.obstacle_free, points))
		return coll_free

	def connect_to_point(self, tree, nearest, new):
		if self.trees[tree].V.count(new) == 0 and self.collision_free(nearest, new):
			self.add_vertex(tree, new)
			self.add_edge(tree, new, nearest)
			return True
		return False

	def reconstruct_path(self, tree, x_init, x_goal):
		path = [x_goal]
		current = x_goal
		if x_init == x_goal:
			return path
		while not self.trees[tree].E[current] == x_init:
			path.append(self.trees[tree].E[current])
			current = self.trees[tree].E[current]
		path.append(x_init)
		path.reverse()
		return path

	def connect_to_goal(self, tree, x_nearest):
		self.trees[tree].E[self.goal] = x_nearest

	def can_connect_to_goal(self, tree, x_nearest):
		if self.goal in self.trees[tree].E:
			return True

		if self.collision_free(x_nearest, self.goal):
			return True
		return False

	def get_path(self):
		x_nearest = self.get_nearest(0, self.goal) 

		# status = self.can_connect_to_goal(0, x_nearest)
		if self.can_connect_to_goal(0, x_nearest):
			# print("Can connect to goal")
			self.connect_to_goal(0, x_nearest)

			return self.reconstruct_path(0, self.pose_initial, self.goal)
			# print("Could not connect to goal")
		return None

	def check_solution(self):
		# This function will terminate the RRT if the actual node can connect
		# the goal node and the samples are superior to the max_samples
		if self.prc and random.random() < self.prc:
			# print("Checking if can connect to goal at", str(self.samples_taken), "samples")
			path = self.get_path()
			if path is not None:
				return True, path
		
		return False, None

	def get_valid_map(self):
		# data = self.mapData.data
		# data = tuple(reversed(data))
		
		# w = self.mapData.info.width
		# h = self.mapData.info.height

		# data = np.reshape(data, (4000, 4000, 1))

		# global_cost_map = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid)
		# global_cost_map = np.array(global_cost_map.data)
		
		# matrix = np.ones(len(global_cost_map))*255
		# matrix[global_cost_map > 10] = 0 # > 60 is a collision
		# matrix = matrix.reshape(4000, 4000, 1) # (height, width)
		# matrix = cv2.rotate(matrix, cv2.ROTATE_180)
		
		# print("Max value: ", matrix.max())
		# print("Min value: ", matrix.min())
		# self.img = matrix
		# cv2.imwrite('/home/caio/projeto_robotica/src/pioneer_ufba/src/map.jpg', self.img)
		self.img = np.array(cv2.imread('/home/caio/projeto_robotica/src/pioneer_ufba/src/map.jpg'))
		self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

	def build_final_path_rviz(self, path):
		# print(path)
		navPath = Path()
		navPath.header.frame_id = "map"
		posList = []
				
		for point in path:
			if point is not None and not isnan(point[0]) and not isnan(point[0]):
				point = [self.invert_coordinates*(point[i] - 2000)*self.map_resolution for i in range(2)] # 2D

				pos = PoseStamped()
				pos.header.stamp = rospy.Time(0)
				pos.pose.position.x = point[1]
				pos.pose.position.y = point[0]
				posList.append(pos)

		navPath.poses = posList
		self.path_pub.publish(navPath)

	def show_edge_path_rviz(self, x_new, x_nearest):
		if x_nearest is not None and x_new is not None:
			x_new = [self.invert_coordinates*(x_new[i] - 2000)*self.map_resolution for i in range(2)] # ponto na arvore
			x_nearest = [self.invert_coordinates*(x_nearest[i] - 2000)*self.map_resolution for i in range(2)] # ponto proximo			

			edge = Marker()
			edge.type = edge.LINE_LIST
			edge.header.frame_id = "map"
			edge.header.stamp = rospy.Time.now()
			edge.ns = "edges"
			edge.id = self.index
			edge.action = edge.MODIFY
			edge.pose.orientation.w = 1
			edge.scale.x = 0.02
			edge.color.r = 1.0
			edge.color.a = 1.0
			# edge.lifetime = rospy.Duration(5)
			edge.points.append(Point(x_nearest[1], x_nearest[0], 0.0)) # Ponto proximo da arvore e incorporado
			edge.points.append(Point(x_new[1], x_new[0], 0.0)) # Ponto na arvore			
			self.index += 1
			self.marker_pub.publish(edge)

	def delete_edge_path(self):
		i = 0
		while i < 200: 
			edge = Marker()
			edge.type = edge.LINE_LIST
			edge.header.frame_id = "map"
			edge.header.stamp = rospy.Time.now()
			edge.id = i
			edge.ns = "edges"
			edge.action = edge.DELETEALL
			self.marker_pub.publish(edge)
			i+=1

	def main_plan(self, goal_tmp):
		image_center = (2000, 2000)
		while not rospy.is_shutdown():
			########################################
			# POSE DO OBJETIVO - EM RELACAO AO MAPA#
			########################################
			x_goal = goal_tmp[0]
			y_goal = goal_tmp[1]
			# Objetivo em pixels em relacao ao mapa
			self.goal = (int(image_center[0] - y_goal / self.map_resolution),
						 int(image_center[1] - x_goal / self.map_resolution))
			
			################
			# POSE DO ROBO #
			################
			# self.tf.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(2.0)) # rospy.Time.now()
			robot_pose, robot_orientation = self.tf.lookupTransform("map", "base_link", rospy.Time(0))
			robot_orientation_euler = euler_from_quaternion(robot_orientation)
			robo_x_inicial, robo_y_inicial = robot_pose[0], robot_pose[1]
							
			# Pose inicial do robo em relacao ao mapa em pixels
			self.pose_initial = (int(image_center[0] - robo_y_inicial / self.map_resolution),
								 int(image_center[1] - robo_x_inicial / self.map_resolution))
						
			###################
			# GERACAO DE ROTA #
			###################
			min_dist = 1000
			max_try = 20
			plot = False
			for i in range(max_try):
				# CONSTROI A ROTA
				if i == max_try - 1:
					plot = True

				path_part = self.build_rrt(plot)
				
				if path_part is not None:
					dist_total = 0
					for i in range(len(path_part)-1):
						dist = sqrt((path_part[i+1][1] - path_part[i][1])**2 + (path_part[i+1][0] - path_part[i][0])**2)
						dist_total += dist
					
					if dist_total < min_dist:
						min_dist = dist_total
						path = path_part

					self.index = 0
					self.trees = []
					self.add_tree()
			
			self.delete_edge_path()
				
			# PLOTA A ROTA E RETORNA
			if path is not None:
				self.build_final_path_rviz(path)
				path_list_m = []
				for point in path:
					point_m = [(2000 - point[i])*self.map_resolution for i in range(2)] # ponto na arvore a partir de /map
					path_list_m.append(point_m)
				
				path_list_with_angle = []
				# get the angle between the points
				end_point = 2
				for i in range(len(path_list_m)-end_point):
					x = path_list_m[i+1][1]
					y = path_list_m[i+1][0]

					x_prox = path_list_m[i+2][1]
					y_prox = path_list_m[i+2][0]

					ang = math.atan2(y_prox - y, x_prox - x)

					if i == len(path_list_m) - end_point - 1:
						x_lin = np.linspace(x, x_prox, 3, endpoint=True)
						y_lin = np.linspace(y, y_prox, 3, endpoint=True)
						for i in range(len(x_lin)):
							pose = [x_lin[i], y_lin[i], ang]
							path_list_with_angle.append(pose)
					else:
						pose = [x, y, ang]
						path_list_with_angle.append(pose)
			
				return path_list_with_angle
			else:
				print('Path not found')







class pioneer_control(object):
	def __init__(self):
		####################
		# DECLARE PUBLISHERS
		####################
		self.robot_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		self.odometry_path_pub = rospy.Publisher('/path_odometry', Path, queue_size=1)
		self.gt_path_pub = rospy.Publisher('/path_ground_truth', Path, queue_size=1)
		self.vis_pub = rospy.Publisher('marker_goal', Marker, queue_size=1)

		#####################
		# DECLARE SUBSCRIBERS
		#####################
		self.goal_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, self.base_pose_ground_truth_callback, queue_size=1)
		
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
		self.robot_linear_velocity_odometry = []
		self.robot_angular_velocity_odometry = []
		self.left_wheel_angular_velocity_odometry = []
		self.right_wheel_angular_velocity_odometry = []        
		self.left_wheel_angular_velocity_ground_truth_list = []
		self.right_wheel_angular_velocity_ground_truth_list = []
		self.robot_linear_velocity_ground_truth_list = []
		self.robot_angular_velocity_ground_truth_list = []
		
		self.time_interval = []

		self.control_state_theta = []
		self.control_state_heading = []
		self.control_state_dist_r = []
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
		# self.path = [[-1, -3, 1.57], [-1, -1, 1.57/2], [1, -0.5, 0], [1.5, 1.5, 3*math.pi/4], [1, 3, 3*math.pi/4], 
					#  [0, 3.4, math.pi], [-1, 2, -math.pi/4], [1, 1.5, 0]]
		# self.path = path_rrt
			
	def base_pose_ground_truth_callback(self, msg):
		linear_x = msg.twist.twist.linear.x
		linear_y = msg.twist.twist.linear.y
		self.robot_linear_velocity_ground_truth = math.sqrt(linear_x**2 + linear_y**2)
		self.robot_angular_velocity_ground_truth = msg.twist.twist.angular.z
				
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
		self.goal_pose = [xt, yt, thetat]        

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
		plt.plot(self.time_interval, self.robot_linear_velocity_odometry, 'k', label='Odometry')
		plt.plot(self.time_interval, self.robot_linear_velocity_ground_truth_list, 'r', label='Ground Truth')
		
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
		plt.plot(self.time_interval, self.robot_angular_velocity_odometry, 'k', label='Odometry')
		plt.plot(self.time_interval, self.robot_angular_velocity_ground_truth_list, 'r', label='Ground Truth')

		for i in range(len(self.vertical_plot_slow_down)):
			plt.vlines(self.vertical_plot_slow_down[i], -0.5, 0.5, 'g', 'dashed')
		
		for i in range(len(self.vertical_plot_transition)):
			plt.vlines(self.vertical_plot_transition[i], -0.5, 0.5, 'r', 'dashed')

		plt.yticks(np.arange(-0.5, 0.5, 0.1))

		plt.subplot(414)
		plt.xlabel('tempo (s)')
		plt.plot(self.time_interval, self.control_state_dist_r, 'k', label='Distance')
		plt.plot(self.time_interval, self.control_state_heading, 'r', label='Heading angle')
		plt.plot(self.time_interval, self.control_state_theta, 'b', label='Theta')
		plt.legend()
		plt.grid()
		plt.show()

		plt.figure(3)
		plt.subplot(211)
		plt.plot(self.time_interval, self.left_wheel_angular_velocity_ground_truth_list, 'k', label='Left Wheel GT')
		plt.plot(self.time_interval, self.left_wheel_angular_velocity_odometry, 'r', label='Left Wheel Odometry')
		plt.subplot(212)
		plt.plot(self.time_interval, self.right_wheel_angular_velocity_ground_truth_list, 'k', label='Right Wheel GT')
		plt.plot(self.time_interval, self.right_wheel_angular_velocity_odometry, 'r', label='Right Wheel Odometry')
		plt.legend()
		plt.grid()
		plt.show()
	
	def save_array_to_csv(self, string):
		"""
		Save the coordinates into csv files
		"""
		script_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'csv_files'))
		
		df = pd.DataFrame({"x" : self.gt_path_x, "y" : self.gt_path_y, "theta" : self.gt_orientation})
		df.to_csv(script_path + "/" + "ground_truth_path" + "_" + string + ".csv", index=False)

		df = pd.DataFrame({"x" : self.odometry_path_x, "y" : self.odometry_path_y, "theta" : self.odometry_orientation})
		df.to_csv(script_path + "/" + "robot_path" + "_" + string + ".csv", index=False)

		print("len_time: ", len(self.time_interval))
		print("robot_linear_velocity_odometry: ", len(self.robot_linear_velocity_odometry))
		print("robot_angular_velocity_odometry: ", len(self.robot_angular_velocity_odometry))
		print("left_wheel_angular_velocity_odometry: ", len(self.left_wheel_angular_velocity_odometry))
		print("right_wheel_angular_velocity_odometry: ", len(self.right_wheel_angular_velocity_odometry))

		df = pd.DataFrame({"time_interval" : self.time_interval, 
						   "linear_velocity_odometry" : self.robot_linear_velocity_odometry, 
						   "angular_velocity_odometry" : self.robot_angular_velocity_odometry,
						   "left_wheel_angular_odometry" : self.left_wheel_angular_velocity_odometry,
						   "right_wheel_angular_odometry" : self.right_wheel_angular_velocity_odometry,
						   "linear_velocity_ground_truth" : self.robot_linear_velocity_ground_truth_list, 
						   "angular_velocity_ground_truth" : self.robot_angular_velocity_ground_truth_list,
						   "left_wheel_angular_ground_truth" : self.left_wheel_angular_velocity_ground_truth_list,
						   "right_wheel_angular_ground_truth" : self.right_wheel_angular_velocity_ground_truth_list})
		df.to_csv(script_path + "/" + "robot_velocities" + "_" + string + ".csv", index=False)

		df = pd.DataFrame({"dist_r" : self.control_state_dist_r,
						   "heading_angle" : self.control_state_heading,
						   "theta" : self.control_state_theta})
		df.to_csv(script_path + "/" + "control_states" + "_" + string + ".csv", index=False)
	
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

		self.right_wheel_angular_velocity_odometry.append(wd)
		self.left_wheel_angular_velocity_odometry.append(we)

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
	
	def get_robot_state_control_odometry(self, goal_pose):
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
	
	def main_control(self, path_rrt):
		"""
		Control the robot based on the Odometry data (calculated using the ICC method)
		and compares with the ground truth from Gazebo
		"""
		self.path = path_rrt
		v_max = 0.4 # linear velocity
		vx = v_max
		v_initial= 0
		w_initial = 0
								
		# Diminui o tempo para 0.01. O caminho final foi bem ruim
		# Com o dt = 0.05, obtive uma boa aprox.
		dt = 0.15 # Time interval
		dist_r = 10
		
		##############################
		# Control parameters
		k1 = 1
		k2 = 3
		beta = 0.4
		lambda_ = 2
		##############################
		
		##############################
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
		##############################

		time = 0
		path_points = len(self.path)
		
		path_iter = 0
		count_, count2_, count3_ = 0, 0, 0
		ended_slow_down_mode, ended_transition_mode = 0, 0
		while path_iter < path_points:
			goal_pose = self.path[path_iter]
						
			# Last odometry data
			_, _, dist_r = self.get_robot_state_control_odometry(goal_pose)
			
			while not rospy.is_shutdown() and dist_r > threshold_distance_goal:
				# Plot the goal in RVIZ
				self.publish_goal(goal_pose)

				# Stores the robot's simulated pose (Ground truth pose) into an array
				gt_pose = self.get_robot_actual_pose()

				self.publish_path_rviz_ground_truth(gt_pose)

				# Last odometry data
				heading_angle, theta, dist_r = self.get_robot_state_control_odometry(goal_pose)

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
				
				#-------------------------------------------
				# STARTS THE SPEED UP LINEAR VELOCITY AGAIN
				#-------------------------------------------
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
				# Need to be improved
				if path_points == len(self.path):
					if transition_mode or speed_up_mode:
						if path_iter + 1 < path_points:
							heading_angle_next, theta_next, dist_next = self.get_robot_state_control_odometry(self.path[path_iter+1])
						else:
							heading_angle_next, theta_next, dist_next = self.get_robot_state_control_odometry(self.path[-1])
				else:
					heading_angle_next, theta_next, dist_next = self.get_robot_state_control_odometry(self.path[path_iter+1])

				curvature = -1/dist_r * (k2*(heading_angle - atan(-k1*theta)) + (1 + k1/(1+(k1*theta)**2))*sin(heading_angle))

				self.control_state_theta.append(theta)
				self.control_state_heading.append(heading_angle)
				self.control_state_dist_r.append(dist_r)

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

				self.robot_linear_velocity_ground_truth_list.append(self.robot_linear_velocity_ground_truth)
				self.robot_angular_velocity_ground_truth_list.append(self.robot_angular_velocity_ground_truth)

				we_gt, wd_gt, _, _ = self.diff_slow_down_model(self.robot_linear_velocity_ground_truth, self.robot_angular_velocity_ground_truth)
				self.left_wheel_angular_velocity_ground_truth_list.append(we_gt)
				self.right_wheel_angular_velocity_ground_truth_list.append(wd_gt)

				self.robot_linear_velocity_odometry.append(vx)
				self.robot_angular_velocity_odometry.append(w)
				
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
			
		self.publish_robot_vel(0, 0) # Stop the robot
		# self.save_array_to_csv("path") # Save the coordinates in csv files
		# self.plot_path() # plot the paths



def main():
	pioneer_vel_control = pioneer_control()
	
	# rospy.on_shutdown is used to stop the robot in cased of the node is killed
	rospy.on_shutdown(pioneer_vel_control.stop_on_shutdown)
	
	RRT_test_class = RRT_Test()
	RRT_test_class.get_valid_map()
	
	while not rospy.is_shutdown():
		print("Choose a goal using the '2D Nav Goal' in Rviz...")
		pose = rospy.wait_for_message('/move_base_simple/goal_rrt', PoseStamped)
		x = pose.pose.position.x
		y = pose.pose.position.y
		rot_q = pose.pose.orientation
		angle = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

		path_rrt = RRT_test_class.main_plan([x, y])
		path_rrt[-1][-1] = angle[-1] # theta value

		raw_input("Press enter to start.")
		pioneer_vel_control.main_control(path_rrt)
	
if __name__ == "__main__":
	main()