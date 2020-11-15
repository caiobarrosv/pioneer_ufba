import pandas as pd
import os
import numpy as np
import matplotlib.pyplot as plt
import math

t = [j for j in np.arange(5.5, 6.5, 0.01)]
tau = [j for j in np.arange(0, 0.2, 0.01)]

sigmoid_list = []
velocidade_antes = 0.5
velocidade_depois = 0.3
velocidade_tran = []
for i in range(len(tau)):
    # print(1 + math.exp(-9.2*(t[i]/tau[i] - 0.5)))
    sigmoid = 1/0.98 * (1 /     (1 + math.exp(-9.2*(tau[i]/0.2 - 0.5))) - 0.01)
    # print("Sigmoid: ", sigmoid)
    # print("tau: ", tau[i])
    # print("Part: ", math.exp(-9.2*(1/t[i] - 0.5)))
    sigmoid_list.append(sigmoid)
    velocidade_tran.append((1 - sigmoid)*velocidade_antes + sigmoid * velocidade_depois)


# print(sigmoid_list)

plt.plot(tau, velocidade_tran, 'r')  # mark initial position
plt.show()
    