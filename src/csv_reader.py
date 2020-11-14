import pandas as pd
import os
import numpy as np
import matplotlib.pyplot as plt

def plot_path(gt_x, gt_y, robot_x, robot_y, angular_velocity, linear_velocity, time_interval, left_wheel_angular, right_wheel_angular):
    plt.figure(1)
    plt.plot(gt_x[0], gt_y[0], 'go')  # mark initial position
    gt_plot = plt.plot(gt_x, gt_y, 'g', label='Ground truth')         # mark path
    robot_plot = plt.plot(robot_x, robot_y, 'r', label='Odometria')
    plt.title('Robot path comparison')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    
    plt.figure(2)
    plt.subplot(211)
    plt.plot(time_interval, linear_velocity)    
    plt.xlabel('tempo (s)')
    plt.ylabel('Velocidade linear (m/s)')
    plt.grid()
    plt.subplot(212)
    plt.xlabel('tempo (s)')
    plt.ylabel('Velocidade Angular (rad/s)')
    plt.grid()
    plt.plot(time_interval, angular_velocity)
    
    plt.figure(3)
    plt.subplot(211)
    plt.plot(time_interval, left_wheel_angular)    
    plt.xlabel('tempo (s)')
    plt.ylabel('Velocidade Angular da Roda Esquerda (m/s)')
    plt.grid()
    plt.subplot(212)
    plt.xlabel('tempo (s)')
    plt.ylabel('Velocidade Angular da Roda Direita (rad/s)')
    plt.grid()
    plt.plot(time_interval, right_wheel_angular)
    
    plt.show()

def get_coordinate_vectors_from_csv(filename):
    path_file = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'csv_files'))
    df = pd.read_csv(path_file + '/' + filename, usecols= ['x','y'])
    x = df.values[:, 0]
    y = df.values[:, 1]
    return x, y

def get_velocities_from_csv(filename):
    path_file = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'csv_files'))
    df = pd.read_csv(path_file + '/' + filename, usecols= ['angular_velocity','left_wheel_angular','linear_velocity','path_curvature','right_wheel_angular','time_interval'])
    angular_velocity = df.values[:, 0]
    left_wheel_angular = df.values[:, 1]
    linear_velocity = df.values[:, 2]
    path_curvature = df.values[:, 3]
    right_wheel_angular = df.values[:, 4]
    time_interval = df.values[:, 5]

    return angular_velocity, linear_velocity, time_interval, left_wheel_angular, right_wheel_angular

gt_x, gt_y = get_coordinate_vectors_from_csv('ground_truth_path_[2.5, -4.33, -1.57]_k1_1_k2_3.csv')
robot_x, robot_y = get_coordinate_vectors_from_csv('robot_path_[2.5, -4.33, -1.57]_k1_1_k2_3.csv')
angular_velocity, linear_velocity, time_interval, left_wheel_angular, right_wheel_angular = get_velocities_from_csv('robot_velocities_[2.5, -4.33, -1.57]_k1_1_k2_3.csv')
plot_path(gt_x, gt_y, robot_x, robot_y, angular_velocity, linear_velocity, time_interval, left_wheel_angular, right_wheel_angular)