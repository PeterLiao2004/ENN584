# -*- coding: utf-8 -*-
"""
Created on Tue Apr 16 15:24:40 2024

@author: ajvan
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
from sklearn.metrics import ConfusionMatrixDisplay
import numpy as np
import scipy.io as io

#basic helper functions
deg2rad = lambda x: x*np.pi/180
rad2deg = lambda x: x*180/np.pi

def wrap_to_pi(x):
    while x < -np.pi:
        x += 2*np.pi
    while x > np.pi:
        x -= 2*np.pi
    return x
'''
Create a plot in matplotlib when initialising the sim and return its handle 
to continue updating it throughout.
'''
n_landmarks = 10
data = io.loadmat('slam_data.mat')


def create_plot(mu, Sigma, lmarks, n_std=3.0):
    fig1 = plt.figure()
    plt.ion()
    axes = fig1.add_subplot(111)
    axes.plot([mu[1,0]], [mu[2,0]], 'x')
    axes.plot(lmarks[:,0], lmarks[:,1], '+')
    
    cov = Sigma[0:2,0:2]
    pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
    
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2, facecolor=None)
    
    transf = transforms.Affine2D() \
    .rotate_deg(45) \
    .scale(np.sqrt(cov[0, 0]) * n_std, np.sqrt(cov[1,1]) * n_std) \
    .translate(mu[0,0], mu[1,0])

    ellipse.set_transform(transf)
    axes.add_patch(ellipse)
    
    plt.axis([-2, 5, -2, 5])
    plt.pause(0.01)
    plt.show()
    return axes

#Update the plot at each stage of the simulation
def update_plot(mu, Sigma, axes):
    axes.clear()
    axes.plot([mu[0,0]], [mu[1,0]], 'x')
    lmark_x_idxs = [3 + 2*x for x in range(10)]
    lmark_y_idxs = [4 + 2*x for x in range(10)]
    axes.plot(mu[lmark_x_idxs, 0], mu[lmark_y_idxs, 0], '+')
    plot_cov(mu,Sigma,axes)
    plt.pause(0.1)
    plt.show()
    return

def plot_cov(mu, S, axes, n_std=3.0):
    
    
    cov = S[0:2,0:2]
    pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
    
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2)
    
    transf = transforms.Affine2D() \
    .rotate_deg(45) \
    .scale(np.sqrt(cov[0, 0]) * n_std, np.sqrt(cov[1,1]) * n_std) \
    .translate(mu[0,0], mu[1,0])

    ellipse.set_transform(transf + axes.transData)
    axes.add_patch(ellipse)
    
    for x in range(10): #10 landmarks
        cov = S[3+2*x:5+2*x, 3+2*x:5+2*x]
        if np.abs(cov[1,1]) < 1e-5:
            continue
        pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
        
        ell_radius_x = np.sqrt(1 + pearson)
        ell_radius_y = np.sqrt(1 - pearson)
        ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2)
        
        transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(np.sqrt(cov[0, 0]) * n_std, np.sqrt(cov[1,1]) * n_std) \
        .translate(mu[3+2*x,0], mu[4+2*x,0])

        ellipse.set_transform(transf + axes.transData)
        axes.add_patch(ellipse)


def get_odom(step):
    
    return data['odom'][step]

def sense_landmarks(step):
    idx = step*n_landmarks
    return data['sensor'][range(idx,idx+10)]

def init_landmarks(z, Q, mu, Sigma):
    #z: measurement of the landmarks (range, bearing)
    #Q: measurement noise covariance
    #mu: current state estimate (robot pose and landmark positions)
    #Sigma: current covariance estimate
    
    # Robot pose
    x = mu[0, 0]
    y = mu[1, 0]
    theta = mu[2, 0]
    
    
    for i in range(n_landmarks):
        r = z[2 * i]
        b = z[2 * i + 1]

        # Convert from polar (relative to robot) to global Cartesian
        angle = theta + b
        lx = x + r * np.cos(angle)
        ly = y + r * np.sin(angle)

        # Store landmark position in mu
        row1 = np.array([[lx]])
        row2 = np.array([[ly]])
        
        mu = np.append(mu, row1, axis=0)
        mu = np.append(mu, row2, axis=0)

        # Jacobian of landmark position wrt measurement [r, b]
        # lx = x + r cos(theta+b)
        # ly = y + r sin(theta+b)
        Jz = np.array([
            [np.cos(angle), -r * np.sin(angle)],
            [np.sin(angle),  r * np.cos(angle)]
        ])

        # Initialize landmark covariance block
        Sigma = np.matrix([[Sigma, 0],
                           [0, Jz*Q*Jz.T]])

    return mu, Sigma

def predict_step(mu, Sigma, d, dth, R):
    
    # mu: [x,y,theta]
    # #mu = f(mu, u) where f is the motion model and u is the control input (d, dth)
    #x_t = x_t-1 + dx
    
    dx = d * np.cos(mu[2,0]) #
    dy = d * np.sin(mu[2,0])
    
    x_new = mu[0,0] + dx
    y_new = mu[1,0] + dy
    
    theta_new = wrap_to_pi(mu[2,0] + dth)
    mu = np.matrix([[x_new], [y_new], [theta_new]])
    
    #Sigma = Jacobian * Sigma * Jacobian.T + R
    Jx = np.matrix([[1, 0, -dy],
                    [0, 1, dx],
                    [0, 0, 1]])
    
    Ju = np.matrix([[np.cos(mu[2,0]), 0],
                    [np.sin(mu[2,0]), 0],
                    [0, 1]])
    
    Sigma = Jx * Sigma * Jx.T + Ju*R*Ju.T
    
    return mu, Sigma

def update_step(landmark_id, z, mu, Sigma, Q):
    
    #landmark_id: id of the landmark being observed
    #z: measurement of the landmark (range, bearing)
    #mu: current state estimate (robot pose and landmark positions)
    #Sigma: current covariance estimate
    #Q: measurement noise covariance
    
    
    
    raise NotImplementedError


if __name__ == "__main__":
    #number of steps to run the simulation for
    nsteps = 100
    
    #Robot initial conditions and uncertainty
    mu = np.matrix([0, 0, deg2rad(0)]).transpose()
    Sigma = np.matrix(np.diag([0.1, 0.1, deg2rad(0.1)])**2)
    
    #Uncertainty in odometry (R) and sensor (Q)
    R = np.matrix(np.diag((0.5, deg2rad(50)))**2)
    Q = np.matrix(np.diag((0.5, deg2rad(5)))**2)
    
    true_landmark_positions = data['map']
    true_robot_pose = data['xr']
    
    #Create plot with initial conditions
    axes = create_plot(mu, Sigma, true_landmark_positions)
    
    #Run simulation for specified number of steps
    for step in range(nsteps):
        
        d, dth = get_odom(step)
        
        mu, Sigma = predict_step(mu, Sigma, d, dth, R)
        
        z = sense_landmarks(step)
        
        if step == 0:
            mu, Sigma = init_landmarks(z, Q, mu, Sigma)
            continue
         
        # for landmark_id, (r, b) in enumerate(z):
        #     zi = np.matrix(
        #         [[r],
        #          [b]]
        #         )
        #     mu, Sigma = update_step(landmark_id, zi, mu, Sigma, Q)
        #     print(Sigma.diagonal())
        
        update_plot(mu, Sigma, axes)
        
        