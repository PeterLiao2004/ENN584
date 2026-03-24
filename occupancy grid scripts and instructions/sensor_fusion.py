# -*- coding: utf-8 -*-
"""
ENN584 Sensor Fusion practical.

Your task in this practical is to implement the incomplete methods below to 
perform occupancy grid mapping with a robot in a 2D environment equipped with:
    1. a laser scanner
    2. a radar scanner
    3. both, with early sensor fusion methods
    4. both, with late sensor fusion methods
The laser scanner is more precise than the radar scanner but has less range and
is vulnerable to occluding obstacles in the environment (a simulated version
of smoke).

Read through the code in both this file and the util_funcs.py script to
understand the tools you have available to you, but only edit code in this
script. You are free to edit code in this script in any way you like, the code
provided is merely a suggestion and skeleton for you to build on if you would
like.

For assessment in the following practical session you will need to demonstrate
the following things:
    1. functional occupancy mapping with each sensor individually
    2. an understanding of what early and late sensor fusion techniques mean
       and how they differentiate from each other. Where are they both useful?
    3. Attempts at implementing and testing sensor fusion to improve robustness
       under uncertainty, for the purpose of occupancy grid mapping.
    4. an understanding of how the code works, how changing certain parameters
       will affect performance, and of other core concepts behind occupancy
       mapping.
You will demonstrate these things by answering tutor questions, inspecting
variables and methods in a live python environment, and through the outputs of
your code as it runs (print statements, plots, graphs)


-------------------------------------------------------------------------------
Created by: Anthony Vanderkop, Thierry Peynot
Last edited: May 15, 2024
-------------------------------------------------------------------------------
"""
import numpy as np
from util_funcs import wrapToPi, Robot, Map, load_path

#a couple of other useful quantities to have
pi = np.pi
d2r = lambda x: x*pi/180
r2d = lambda x: x*180/pi


class OccupancyGrid(object):
    def __init__(self, map_bounds, resolution, occupied_threshold):
        self.bounds = map_bounds #in form [ xmin, ymin, xmax, ymax]
        self.resolution = resolution
        xsize = (self.bounds[2] - self.bounds[0])/self.resolution
        ysize = (self.bounds[3] - self.bounds[1])/self.resolution
        self.grid = np.zeros((xsize, ysize))
    
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

    def prob_2_log_odds(self, p):
        '''
        Your code here. Convert from log-odds to probability.
        Inputs:
        p = probability. Float in range [0,1]
        
        Outputs:
        l - log-odds. Float from -Inf to +Inf
        '''
        raise NotImplementedError

    def log_odds_2_prob(self, l):
        
        '''
        Your code here. Convert from log-odds to probability.
        Inputs:
        l - log-odds. Float from -Inf to +Inf
        Outputs:
        p = probability. Float in range [0,1]
        '''
        raise NotImplementedError
        
    def plot_occupancy_grid(self):
        
        raise NotImplementedError
        
    def ij_to_world(self, i, j):
        raise NotImplementedError
        
    def world_to_ij(self, x, y):
        raise NotImplementedError
    


def laser_scanner_occupancy(robot, occupancy_grid):
    
    while True:
        laser_scan, _ = robot.step()
        
        #figure out all of the cells that i have new information on
        
        #update my belief that those cells are occupied based on this info
        
        break
    
    raise NotImplementedError()
    
def radar_occupancy(robot, occupancy_grid):
    
    while True:
        _, radar_scan = robot.step()
        break
    
    raise NotImplementedError()
    
def sensor_fusion_occupancy_early(robot, occupancy_grid):
    
    while True:
        laser_scan, radar_scan = robot.step()
        #figure out all of the cells that i have new information on
        
        #update my belief that those cells are occupied based on this info
        #do my sensors agree? or disagree?
        
        #combine info from the two sensors together and create only one occupancy map
        
        break
        
    raise NotImplementedError()
    
def sensor_fusion_occupancy_late(robot, occupancy_grid):
    
    while True:
        laser_scan, radar_scan = robot.step()
        
        #figure out all of the cells that i have new information on
        
        #update my belief that those cells are occupied based on this info
        
        #build occupancy mad from radar
        
        #build occupancy map from laser
        
        #combine them together
        
        break
        
    raise NotImplementedError()

if __name__ == "__main__":
    
    #load in a map
    mapfile = 'map.png'
    pathfile = 'map_sporadic_path.txt'
    true_map = Map(mapfile, resolution=0.05, origin='centre')
    path = load_path(pathfile)
    
    bot = Robot(pose = path[0],
                true_map=true_map,
                path=path)
    for i in range(len(path)):
        show = True
        laser_scan, radar_scan = bot.step(show_ray=show)
    
    
    occupancy_grid = OccupancyGrid()
    
    #create the robot
    bot = Robot()
    
    #continually update the occupancy map based on sensor measurements
    laser_scanner_occupancy(robot, occupancy_grid)