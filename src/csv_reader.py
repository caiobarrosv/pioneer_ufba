import pandas as pd
import os
import numpy as np
import matplotlib.pyplot as plt

def plot_path(gt_x, gt_y, robot_x, robot_y):
    plt.plot(gt_x[0], gt_y[0], 'bo')  # mark initial position
    plt.plot(gt_x, gt_y, '-r', label='Ground truth') 

    plt.plot(robot_x, robot_y, '-k', label='Path executed')

    plt.title('Robot path comparison')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

def get_coordinate_vectors_from_csv(filename):
    path_file = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'csv_files'))
    df = pd.read_csv(path_file + '/' + filename, usecols= ['x','y'])
    x = df.values[:, 0]
    y = df.values[:, 1]
    return x, y

gt_x, gt_y = get_coordinate_vectors_from_csv('ground_truth_path.csv')
robot_x, robot_y = get_coordinate_vectors_from_csv('robot_path.csv')
plot_path(gt_x, gt_y, robot_x, robot_y)