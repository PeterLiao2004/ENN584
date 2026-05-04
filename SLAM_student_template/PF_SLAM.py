import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as io
from particle_filter import ParticleFilter
from util_functions import *

n_landmarks = 10
data = io.loadmat('slam_data.mat')


def create_plot(pf, true_lmarks, estimated_lmarks=None):
    
    plt.ion()
    fig1 = plt.figure()
    axes = fig1.add_subplot(111)
    
    x = []
    y = []
    b = []
    xl = []
    yl = []
    for particle in pf.particles:
        x.append(particle[0])
        y.append(particle[1])
        b.append(particle[2])

    for lmark in true_lmarks:
        xl.append(lmark[0])
        yl.append(lmark[1])

    if estimated_lmarks is not None:
        for lmark in estimated_lmarks:
            xl += lmark[0]
            yl += lmark[1]
        
    axes.plot(x, y, 'o', color='g')
    axes.plot(xl, yl, 'x', color='b')
    plt.axis([-2, 5, -2, 5])
    plt.pause(0.1)
    plt.show()
    return axes

#Update the plot at each stage of the simulation
def update_plot(pf, ground_truth, global_map, axes):
    axes.clear()
    
    axes.plot(pf.particles[:,0], pf.particles[:,1], 'o', color='g')
    axes.plot(ground_truth[:,0], ground_truth[:,1], 'x', color='b')
    axes.plot(pf.global_map[:,0], pf.global_map[:,1], 'x', color='r')
    
    
    plt.show()
    plt.pause(0.1)



def get_odom(step):
    
    return data['odom'][step]

def sense_landmarks(step):
    idx = step*n_landmarks
    return data['sensor'][range(idx,idx+10)]



if __name__ == "__main__":
    #number of steps to run the simulation for
    nsteps = 100
    resample_rate = 10
    
    #Initialise particle filter
    mu = [0, 0, 0]
    Sigma = np.array([0.05, 0.05, deg2rad(0.1)])**2
    pf = ParticleFilter(mu, Sigma, n_particles=50)
    
    #Uncertainty in odometry (R) and sensor (Q)
    R = np.matrix(np.diag((0.1, deg2rad(5)))**2)
    Q = np.matrix(np.diag((0.2, deg2rad(5)))**2)
    
    true_landmark_positions = data['map']
    true_robot_pose = data['xr']
    
    #Create plot with initial conditions
    axes = create_plot(pf, true_landmark_positions)
    
    #Run simulation for specified number of steps
    for step in range(nsteps):
        
        d, dth = get_odom(step)
    
        estimated_pose = pf.predict_all(d, dth, R)
        
        z = sense_landmarks(step)
        # range and bearing to each landmark, 
        
        if step == 0:
            global_map = pf.init_landmarks(z)
            update_plot(pf, true_landmark_positions, global_map, axes)
            continue

        resample = (step % resample_rate) == 0 
        global_map = pf.update(z, global_map, resample=resample)
        
        update_plot(pf, true_landmark_positions, global_map, axes)
        
        