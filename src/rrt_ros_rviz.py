#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from copy import deepcopy, copy
from rtree import index
import random

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA

from math import sqrt, atan2, sin, cos, isnan
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs #import the packages first

import matplotlib.pyplot as plt

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

class RRT_Test():
	def __init__(self):
		
		rospy.init_node('detector', anonymous=False)
		
		# SUBSCRIBERS
		map_topic = "/map"
		rospy.Subscriber(map_topic, OccupancyGrid, self.mapCallBack, queue_size=1)
		
		# PUBLISHERS
		self.path_pub = rospy.Publisher("/Path", Path, queue_size=10)
		self.marker_pub = rospy.Publisher("/marker", Marker, queue_size=10)
		self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
		# self.vis_pub = rospy.Publisher("/marker_array", MarkerArray, queue_size=100)

		self.edge_length = 5
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

	def build_rrt(self):
		self.add_vertex(0, self.pose_initial)
		self.add_edge(0, self.pose_initial, None)

		while not rospy.is_shutdown():
			x = self.sample()
			x_new, x_nearest = self.new_and_near(0, x)

			if x_new is None:
				continue

			# self.show_edge_path_opencv(x_new, x_nearest, False)
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

		status = self.can_connect_to_goal(0, x_nearest)
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
				self.show_edge_path_rviz(path[-2], path[-1])
				print('Path 1')
				return True, path
		
		return False, None

	def get_valid_map(self):
		data = self.mapData.data
		data = tuple(reversed(data))
		
		w = self.mapData.info.width
		h = self.mapData.info.height

		data = np.reshape(data, (4000, 4000, 1))

		global_cost_map = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid)
		global_cost_map = np.array(global_cost_map.data)
		
		matrix = np.ones(len(global_cost_map))*255
		matrix[global_cost_map > 60] = 0 # > 60 is a collision
		matrix = matrix.reshape(4000, 4000, 1) # (height, width)
		matrix = cv2.rotate(matrix, cv2.ROTATE_180)
		
		print("Max value: ", matrix.max())
		print("Min value: ", matrix.min())
		self.img = matrix

	def show_edge_path_opencv(self, x_new, x_nearest, plot):
		img = self.img
		x_new = deepcopy(tuple(np.flip(x_new))) 
		x_nearest = deepcopy(tuple(np.flip(x_nearest)))
		cv2.circle(img, x_new, 2, (230, 230, 230), -1)
		cv2.line(img, x_nearest, x_new, (230, 230, 230), 1)

		if plot:
			y_dim = self.dimension_offset[0]
			y_off = self.img_dim[0]

			x_dim = self.dimension_offset[1]
			x_off = self.img_dim[1]

			# X is horizontal. The map starts at 1888 in x
			img = img[y_dim:y_dim + y_off, x_dim:x_dim + x_off]

			cv2.namedWindow("Window",cv2.WINDOW_NORMAL)
			cv2.resizeWindow("Window", 500,500)
			cv2.imshow("Window", img) # Show me the image
			cv2.waitKey(0)
			cv2.destroyAllWindows()

	def build_final_path_opencv(self, path, color):
		y_dim = self.dimension_offset[0]
		y_off = self.img_dim[0]

		x_dim = self.dimension_offset[1]
		x_off = self.img_dim[1]

		img = deepcopy(self.img)
		for i in range(len(path) - 1):
			x_prior = tuple([int(j) for j in path[i]])
			x_next = tuple([int(j) for j in path[i+1]])
			x_prior = x_prior[::-1]
			x_next = x_next[::-1]
			cv2.circle(img, x_next, 1, (220, 220, 220), -1)
			cv2.line(img, x_prior, x_next, color, 1)
			
			# k = cv2.waitKey(20) #time
			# if k == ord('q') & 0xFF:
				# break

		img = img[y_dim:y_dim + y_off, x_dim:x_dim + x_off]
		
		cv2.namedWindow("Window",cv2.WINDOW_NORMAL)
		cv2.resizeWindow("Window", 500,500)
		cv2.imshow("Window", img) # Show me the image
		cv2.waitKey(0)
		cv2.destroyAllWindows()

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

	def main_plan(self):
		# print("Please choose a goal in RVIZ.")
		# self.goal_info = rospy.wait_for_message("/move_base_simple/goal", PoseStamped)
		self.goal_info = True

		rate = rospy.Rate(10)
		image_center = (2000, 2000)
		if self.goal_info:
			while not rospy.is_shutdown():
				########################################
				# POSE DO OBJETIVO - EM RELACAO AO MAPA#
				########################################
				option = int(raw_input("[1] - Goal 1\n[2] - Goal 2\noption: "))
				if option == 1:
					x_goal = -3.09
					y_goal = -1.61
				elif option == 2:
					x_goal = -2
					y_goal = -1

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
				# print('Pose initial: ', self.pose_initial)
				# self.pose_initial = (2000, 2000)
				
				###################
				# GERACAO DE ROTA #
				###################
				min_dist = 1000
				max_try = 1
				for i in range(max_try):
					# CONSTROI A ROTA
					path_part = self.build_rrt()
					
					if path_part is not None:
						dist_total = 0
						for i in range(len(path_part)-1):
							dist = sqrt((path_part[i+1][1] - path_part[i][1])**2 + (path_part[i+1][0] - path_part[i][0])**2)
							dist_total += dist
						
						if dist_total < min_dist:
							min_dist = dist_total
							path = path_part
	
						print("Dist total: ", dist_total)
	
						self.index = 0
						# self.delete_edge_path()
						self.trees = []
						self.add_tree()
					
				print("min dist: ", min_dist)

				# PLOTA A ROTA
				if path is not None:
					self.build_final_path_rviz(path)
					return path
				else:
					print('Path not found')
				

# def main():
# 	RRT_ = RRT_Test()
# 	rospy.sleep(1)
# 	RRT_.get_valid_map()
# 	RRT_.main_plan()

# if __name__ == '__main__':
# 	try:
# 		main()
# 	except rospy.ROSInterruptException:
# 		pass
