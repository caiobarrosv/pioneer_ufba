import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos

def rot_z_simples(x):
    """
    Rotation matrix around the Z axis
    """
    rot_z_matrix = np.array([[cos(x), -sin(x)],
                             [sin(x),  cos(x)]])
    return rot_z_matrix

def main():
    pose = [0, 0, 0]
    theta = pose[-1] # initial robot orientation
    x = [pose[0]]
    y = [pose[1]]
    
    L = 0.316
    r = 0.09
    
    A = np.array([[r/2, r/2], [-r/L, r/L]])

    vx = 0.5
    w = 0.1
    wheel_vel = np.matmul(np.linalg.inv(A), np.array([vx, w]))

    we = wheel_vel[0] 
    wd = wheel_vel[1]
    
    ve = we * r
    vd = wd * r

    print('ve: ', ve)
    print('vd: ', vd)
    
    dt = 0.1
    iteration = 0
    while iteration < 100:
        if vd == ve:
            pose = pose + np.array([vx*cos(theta)*dt, vx*sin(theta)*dt, 0])
            theta = pose[-1]
            x.append(pose[0])
            y.append(pose[1])
        else:
            R = (L / 2) * ((vd + ve) / (vd - ve))
            omega = (vd - ve) / L
            
            ICC_x = pose[0] - R*sin(theta)
            ICC_y = pose[1] + R*cos(theta)
            ICC = np.array([ICC_x, ICC_y])

            rot_z_mat = rot_z_simples(omega*dt)

            ICC_origin = np.array([pose[0] - ICC_x, pose[1] - ICC_y])

            theta = theta + omega*dt

            pose = np.matmul(rot_z_mat, ICC_origin) + ICC
            
            x.append(pose[0])
            y.append(pose[1])
        iteration += dt

    plt.plot(x, y) 
    plt.ylabel('Robot path comparison')
    plt.grid()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
