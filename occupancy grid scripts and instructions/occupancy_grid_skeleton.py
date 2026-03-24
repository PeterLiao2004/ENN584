#!/usr/bin/env python
'''EGH437 Occupancy Grid Practical. Code taken and adapted from following repositories and used within
the bounds of the open-source licenses.

https://github.com/salihmarangoz/robot_laser_simulator
https://github.com/ucb-ee106/lab8_starter/tree/master

This code simulates a simple field robot moving in a 2D environment and is compatible with ROS.

Usage:
roslaunch occupancy_demo start.launch
rosrun rqt_robot_steering rqt_robot_steering
'''
from PIL import Image 
import numpy as np
import time
# from std_msgs.msg import ColorRGBA
# from geometry_msgs.msg import Twist, Point
# from sensor_msgs.msg import LaserScan
# from rosgraph_msgs.msg import Clock
# from visualization_msgs.msg import Marker
# import tf_conversions
# from tf import TransformBroadcaster


pi = np.pi
d2r = lambda x: x*pi/180
r2d = lambda x: x*180/pi

def wrap2Pi(theta):

    while theta < -pi:
        theta += 2*pi
    while theta > pi:
        theta -= 2*pi

    return theta




class RealMap:
    def __init__(self, filename, resolution, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist):
        img = Image.open(filename)
        img = img.convert('1') # to gray image
        self.gridmap = 1.0 - np.asarray(img)
        self.gridmap = np.around(self.gridmap)
        self.resolution = resolution   # map resolution, m per pixel
        self.laser_min_angle = laser_min_angle    #rad
        self.laser_max_angle = laser_max_angle    #rad
        self.laser_resolution = laser_resolution  #rad
        self.laser_max_dist = laser_max_dist      #meter
        self.robot_x = 0.0                        #meter
        self.robot_y = 0.0                        #meter
        self.robot_theta = 0.0                    #rad

    def to_world (self, i, j):
        '''
        Convert from  grid coordinates (row, column) to world coordinates (x, y) in m
        '''
        # i: row, j: column, resolution (m per pixel), 
        # assuming the origin of the world is on bottom left
        
        map_height, map_width = self.gridmap.shape
        
        x = j * self.resolution 
        y = (map_height - i - 1) * self.resolution
        
        return x, y

    def to_map (self, x, y):

        map_height, map_width = self.gridmap.shape
        
        j = int(x / self.resolution)
        i = map_height - int(y / self.resolution) - 1

        return i, j

    def is_inside (self, i, j):
        return i<self.gridmap.shape[0] and j<self.gridmap.shape[1] and i>=0 and j>=0

    def get_borders(self):
        return [0.0, self.gridmap.shape[1] * self.resolution, 0.0, self.gridmap.shape[0] * self.resolution]

    def set_robot_pos(self, x, y, theta):
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta

    def get_measurements(self, occupancy_grid=None):
        #iterate through all laser beams and determine whether the laser collides with an obstacle in the world
        laser_data = []
        for i in np.arange(self.laser_min_angle, self.laser_max_angle+self.laser_resolution, self.laser_resolution):
            xp, yp, is_hit = self.raycast(self.robot_x, self.robot_y, wrap2Pi(np.radians(i) + self.robot_theta), self.laser_max_dist, occupancy_grid)

            if is_hit:
                laser_data.append(np.sqrt((xp-self.robot_x)**2+(yp-self.robot_y)**2))

            else:
                # if goes beyond max dist return max dist or nan
                #laser_data.append(self.laser_max_dist)
                laser_data.append(float('nan'))
        scan = np.array(laser_data)
        return scan

    def raycast(self, x0, y0, theta, max_dist, occupancy_grid=None):
        #calculate end-point of laser scan in world units
        x1 = x0 + max_dist*np.cos(theta)
        y1 = y0 + max_dist*np.sin(theta)
        #calculate pixel/occupancy locations of start and end points in world units
        i0, j0 = self.to_map(x0, y0)
        i1, j1 = self.to_map(x1, y1)

        max_dist_cells = max_dist / self.resolution

        #Find pixel/occupancy location of collision with laser using Bresenham method
        ip, jp, collision = self.bresenham(i0, j0, i1, j1, max_dist_cells, occupancy_grid=occupancy_grid)

        #Convert pixel location to world units
        xp, yp = self.to_world(ip, jp)

        return xp, yp, collision
    
    #bresenham method is used to plot the lines
    def bresenham (self, i0, j0, i1, j1, max_dist_cells, occupancy_grid=None):   # i0, j0 (starting point)
        
        '''
        Your code here. Implement the Bresenham method to detect collisions between the laser and an obstacle in the world.
        ~HINT~ if there is a collision at grid location [i,j], then self.gridmap[i][j]==1. When there is a collision, update your
        occupancy grid if it is implemented. Whenever there is no collision, also update!
        Inputs:
        i0, j0: starting row and column locations of the laser, respectively
        i1, j1: ending row and column locations, respectively
        max_dist_cells: maximum number of cells the laser can go through before it has travelled too far (15m in our case)
        Outputs:
        (row, col, collision)
        collision : Boolean. True if there was a collision, False otherwise
        row, col: Integers. Row and column location in the map where the collision occured. If there was no collision, these 
            are unused in the rest of the code so the values retuned are not important.
        '''
        
        
        
        raise NotImplementedError
        



class OccupancyGrid(object):
    def __init__(self, map_size, resolution, threshold):

        self.occupancy_grid = np.zeros(map_size) #initialize the map
        self.resolution = resolution #m per grid cell between adjacent cells
        self.threshold = threshold
    
    def update(self, i, j, collision):
        '''
        Your code here. Update the values in the occupancy grid based on whether the laser
        collided with an obstacle at that position or not.
        Inputs:
        i, j: Integers. The row and column location the laser passed through, respectively
        collision: Boolean. True if it did collide, False otherwise.
        Outputs:
        None
        '''

        raise NotImplementedError
        update_val = 0
        if collision:
            pass
        else:
            pass
        
        self.occupancy_grid[int(i)][int(j)] += update_val

    def ProbabilityToLogOdds(self, p):
        '''
        Your code here. Convert from log-odds to probability.
        Inputs:
        p = probability. Float in range [0,1]
        
        Outputs:
        l - log-odds. Float from -Inf to +Inf
        '''
        raise NotImplementedError

    def LogOddsToProbability(self, l):
        
        '''
        Your code here. Convert from log-odds to probability.
        Inputs:
        l - log-odds. Float from -Inf to +Inf
        Outputs:
        p = probability. Float in range [0,1]
        '''
        raise NotImplementedError


if __name__ == "__main__":
    #Load in the true map of obstacles from an image
    
    img_file = 'occupancy grid scripts and instructions/map.png'
    resolution = 0.05 #m per pixel
    min_scan_angle, max_scan_angle, angular_res, laser_range = d2r(-135), d2r(135), d2r(1), 15.0
    obstacle_map = RealMap(img_file, resolution, min_scan_angle, max_scan_angle, angular_res, laser_range)
    mapsize = np.shape(obstacle_map.gridmap)

    #create an occupancy grid 
    mapsize = np.shape(obstacle_map)
    occupied_threshold = 0.7
    occupancyGrid = OccupancyGrid(mapsize, resolution, occupied_threshold)

    #Individual tests of the bresenham method
    x, y, theta = 0, 0, 0 #set start position of the laser scan

    #cast an individual laser and find a collision
    x_col, y_col, colision = obstacle_map.raycast(x, y, theta, laser_range, occupancy_grid=None)

    #Do a full laser scan from the robot's position
    obstacle_map.set_robot_pos(x, y, theta)
    scan = obstacle_map.get_measurements(occupancy_grid=None)


