import pandas as pd
import os
import numpy as np
import matplotlib.pyplot as plt
import math

t = [j for j in np.arange(5.5, 6.5, 0.01)]
tau = [j for j in np.arange(0, 1, 0.01)]

sigmoid_list = []
velocidade_antes = 0.5
velocidade_depois = 0.3
velocidade_tran = []
for i in range(len(t)):
    # print(1 + math.exp(-9.2*(t[i]/tau[i] - 0.5)))
    sigmoid = 1/0.98 * (1 /     (1 + math.exp(-9.2*(tau[i] - 0.5))) - 0.01)
    # print("Sigmoid: ", sigmoid)
    # print("tau: ", tau[i])
    # print("Part: ", math.exp(-9.2*(1/t[i] - 0.5)))
    print("Progress: ", 1/tau[i])
    sigmoid_list.append(sigmoid)
    velocidade_tran.append((1 - sigmoid)*velocidade_antes + sigmoid * velocidade_depois)


# print(sigmoid_list)

plt.plot(t, velocidade_tran, 'r')  # mark initial position
# plt.plot(t, sigmoid_list, 'r')  # mark initial position
# gt_plot = plt.plot(gt_x, gt_y, 'g', label='Ground truth')         # mark path
# robot_plot = plt.plot(robot_x, robot_y, 'r', label='Odometria')
# plt.title('Robot path comparison')
# plt.legend()
# plt.grid()
# plt.tight_layout()
plt.show()
    