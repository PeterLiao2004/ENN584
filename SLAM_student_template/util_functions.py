import matplotlib.pyplot as plt
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

def wrap_nparray_to_pi(x):
    for idx, bearing in enumerate(x):
        while bearing < -np.pi:
            bearing += 2*np.pi
        while bearing > np.pi:
            bearing -= 2*np.pi
        x[idx] = bearing
    return x


def xy_to_rangebearing(start_points, reference_point):
    dx = (reference_point[0] - start_points[:, 0])
    dy = (reference_point[1] - start_points[:, 1])

    ranges = np.sqrt(dx**2 + dy**2)
    bearings = wrap_nparray_to_pi(np.arctan2(dy, dx))

    return ranges, bearings



def rangebearing_to_xy(start_points, range, bearing):
    bearings = wrap_nparray_to_pi(start_points[:,2] + bearing)
    x = start_points[:, 0] + range * np.cos(bearings)
    y = start_points[:, 1] + range * np.sin(bearings)
    return x, y