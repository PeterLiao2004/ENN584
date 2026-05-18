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
import matplotlib.pyplot as plt
from util_funcs import wrapToPi, Robot, Map, load_path

#a couple of other useful quantities to have
pi = np.pi
d2r = lambda x: x*pi/180
r2d = lambda x: x*180/pi


class OccupancyGrid(object):
    def __init__(self, map_bounds, resolution, occupied_threshold):
        self.bounds = map_bounds # in form [xmin, ymin, xmax, ymax]
        self.resolution = resolution
        self.occupied_threshold = occupied_threshold

        xsize = int((self.bounds[2] - self.bounds[0]) / self.resolution)
        ysize = int((self.bounds[3] - self.bounds[1]) / self.resolution)

        self.grid = np.zeros((ysize, xsize))
    
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
        i = int(i)
        j = int(j)

        # if the cell is out of bounds, ignore it
        if i < 0 or j < 0 or i >= self.grid.shape[0] or j >= self.grid.shape[1]:
            return

        # Odds of occupancy and free space. Change these to adjust how strongly
        # each observation updates the map.
        p_occ = 0.7
        p_free = 0.3
        
        # update the log-odds value in the grid based on whether we observed a collision or not.
        if collision:
            self.grid[i, j] += self.prob_2_log_odds(p_occ)
        else:
            self.grid[i, j] += self.prob_2_log_odds(p_free)

    def prob_2_log_odds(self, p):
        '''
        Your code here. Convert from probability to log-odds.
        Inputs:
        p = probability. Float in range [0,1]
        
        Outputs:
        l - log-odds. Float from -Inf to +Inf
        '''
        with np.errstate(divide='ignore', invalid='ignore'):
            return np.log(np.asarray(p) / (1 - np.asarray(p)))

    def log_odds_2_prob(self, l):
        
        '''
        Your code here. Convert from log-odds to probability.
        Inputs:
        l - log-odds. Float from -Inf to +Inf
        Outputs:
        p = probability. Float in range [0,1]
        '''
        l = np.asarray(l)
        p = np.empty_like(l, dtype=float)

        positive = l >= 0
        p[positive] = 1 / (1 + np.exp(-l[positive]))

        exp_l = np.exp(l[~positive])
        p[~positive] = exp_l / (1 + exp_l)

        return p.item() if p.shape == () else p
        
    def plot_occupancy_grid(self):
        prob_grid = self.log_odds_2_prob(self.grid)

        plt.figure()
        plt.imshow(prob_grid,
                   cmap='gray_r',
                   origin='lower',
                   vmin=0,
                   vmax=1,
                   extent=self.bounds)
        plt.colorbar(label='Occupancy probability')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title('Occupancy Grid')
        plt.show()
        
    def ij_to_world(self, i, j):
        x = self.bounds[0] + j * self.resolution
        y = self.bounds[1] + i * self.resolution
        return x, y
        
    def world_to_ij(self, x, y):
        i = int((y - self.bounds[1]) / self.resolution)
        j = int((x - self.bounds[0]) / self.resolution)
        return i, j
    


def laser_scanner_occupancy(robot, occupancy_grid):
    
    while True:
        laser_scan, _ = robot.step()
        
        #figure out all of the cells that i have new information on, 
        #
        #I WILL GIVE YOU THIS CODE IN A SECOND
        angles = robot.laser_angles
        angle_idx = 0
        for r, collision in laser_scan:
            start_pose = robot.get_pose()

            row_list, col_list = robot.map.cast_ray(start_pose[0], start_pose[1], r, wrapToPi(start_pose[2] + angles[angle_idx]))

            angle_idx += 1

            #use the row_list and col_list to update the map for each laser scan here


        
        #for each cell identified, orccupancy_grid.update()
        
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
    pathfile = 'map1_path.txt'
    true_map = Map(mapfile, resolution=0.05, origin='centre')
    path = load_path(pathfile)
    
    # Create robot and set it to the start of the path
    bot = Robot(pose = path[0],
                true_map=true_map,
                path=path)
    
    # Create an occupancy grid to represent the world around the robot.
    # occupancy_grid = OccupancyGrid(
    #     map_bounds=[xmin, ymin, xmax, ymax],
    #     resolution=0.05,
    #     occupied_threshold=0.5
    # )
    
    # Choose mapping mode
    #   - laser only
    #   - radar only
    #   - early fusion
    #   - late fusion
    
    #continually update the occupancy map based on sensor measurements
    # laser_scanner_occupancy(bot, occupancy_grid)
    
    # For each step, get sensor scans, convert measurement to grid cells, update the grid
    for i in range(len(path)):
        show = True
        laser_scan, radar_scan = bot.step(show_ray=show)
        
    # Convert log odds to probabilities for visualization
    
    # Plot the final occupancy grid map
    

