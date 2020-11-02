import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos

def rot_z_simples(x):
    """
    Rotation matrix around the Z axis

    Arguments:
		x {float} -- Rotation angle
    """
    rot_z_matrix = np.array([[cos(x), -sin(x)],
                             [sin(x),  cos(x)]])
    return rot_z_matrix

def main():
    pose = [0, 0, 0]
    theta = pose[-1] # initial robot orientation
    x = [pose[0]]
    y = [pose[1]]
    
    L = 0.316 # Distance between wheels
    r = 0.09 # Wheel radius
    dt = 0.1 # Time interval

    A = np.array([[r/2, r/2], [-r/L, r/L]])

    vx = 0.5 # Linear velocity
    w = 0.1 # Angular velocity

    wheel_vel = np.matmul(np.linalg.inv(A), np.array([vx, w])) # Wheel velocities
    
    we = wheel_vel[0] # Angular velocity of the left wheel
    wd = wheel_vel[1] # Angular velocity of the right wheel
    
    ve = we * r # Linear velocity of the left wheel
    vd = wd * r # Linear velocity of the right wheel

    print('ve: ', ve)
    print('vd: ', vd)
    for i in np.arange(0, 100, dt):
        if vd == ve:
            pose = pose + np.array([vx*cos(theta)*dt, vx*sin(theta)*dt, pose[-1]])
            theta = pose[-1]
            x.append(pose[0])
            y.append(pose[1])
        else:
            R = (L / 2) * ((vd + ve) / (vd - ve)) # Distance between P and ICC
            omega = (vd - ve) / L # Angular velocity
            
            ICC_x = pose[0] - R*sin(theta)
            ICC_y = pose[1] + R*cos(theta)
            ICC = np.array([ICC_x, ICC_y])

            rot_z_mat = rot_z_simples(omega*dt)

            ICC_origin = np.array([pose[0] - ICC_x, pose[1] - ICC_y]) # ICC at origin

            theta = theta + omega*dt

            pose = np.matmul(rot_z_mat, ICC_origin) + ICC # New pose
            x.append(pose[0])
            y.append(pose[1])

    plt.plot(x, y) 
    plt.ylabel('Robot path comparison')
    plt.grid()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()