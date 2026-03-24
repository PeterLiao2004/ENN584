#!/usr/bin/env python


from PIL import Image 
import numpy as np
from numpy.random import normal, uniform
import matplotlib.pyplot as plt


pi = np.pi
d2r = lambda x: x*pi/180
r2d = lambda x: x*180/pi

def wrapToPi(theta):

    while theta < -pi:
        theta += 2*pi
    while theta > pi:
        theta -= 2*pi

    return theta

def load_path(pathfile):
    path = np.genfromtxt(pathfile, delimiter=',')
    return path

class Robot(object):
    def __init__(self,
                 pose=[0,0,0],
                 pose_covariance = [0.1, 0.1, d2r(5)],
                 laser_range = 3,
                 laser_covariance=[0.05, d2r(3)],
                 min_laser_angle = d2r(-60),
                 max_laser_angle = d2r(60),
                 laser_angular_resolution = d2r(3),
                 radar_range = 10,
                 radar_covariance = [0.4, d2r(6)],
                 min_radar_angle = d2r(-80),
                 max_radar_angle = d2r(80),
                 radar_angular_resolution = d2r(15),
                 path=None,
                 true_map=None,
                 noise=True
                 ):
        #setup robot pose and path to follow
        self.pose = pose
        self.pose_covariance = pose_covariance
        self.map = true_map
        self.path = path
        self.step_num = -1
        
        #setup sensors
        self.laser_range = laser_range
        self.laser_covariance = laser_covariance
        self.laser_angles = np.arange(min_laser_angle,
                                       max_laser_angle+laser_angular_resolution,
                                       laser_angular_resolution)
        self.laser = None #last laser scan output
        
        self.radar_range = radar_range
        self.radar_covariance = radar_covariance
        self.radar_angles = np.arange(min_radar_angle,
                                       max_radar_angle + radar_angular_resolution,
                                       radar_angular_resolution)
        self.radar = None #last radar scan output
        
        
        
        
        
    def step(self, step=None, show_ray=False):
        if step is None:
            self.step_num += 1
            if not self.step_num < len(self.path):
                self.step_num -= len(self.path)
        else:
            self.step_num = step
        
        self.set_pose(self.path[self.step_num])
        
        laser_scan = self.laser_scan(show_ray=show_ray)
        radar_scan = self.radar_scan(show_ray=show_ray)
        
        return laser_scan, radar_scan
    
    def set_pose(self, pose):
        self.pose = pose
        self.pose_noise = normal(scale=self.pose_covariance)
        
    def laser_scan(self, show_ray=False):
        ranges, collisions = [], []
        
        heading = self.pose[2]+self.pose_noise[2]
        for angle in self.laser_angles:
            angle = normal(angle, self.laser_covariance[1])
            
            r, collision = self.map.raycast(self.pose[0]+self.pose_noise[0],
                                             self.pose[1]+self.pose_noise[1],
                                             wrapToPi(heading+angle),
                                             self.laser_range,
                                             sensor='laser',
                                             show_ray=show_ray)
            if collision:
                r = normal(r, self.laser_covariance[0])
            
            ranges += [r]
            collisions += [collision]
            
        self.laser= (ranges, collisions)
        return self.laser
        
        
    def radar_scan(self, show_ray=False):
        ranges, collisions = [], []
        
        heading = wrapToPi(self.pose[2]+self.pose_noise[2])
        for angle in self.radar_angles:
            angle = normal(angle, self.radar_covariance[1])
            r, collision = self.map.raycast(self.pose[0]+self.pose_noise[0],
                                             self.pose[1]+self.pose_noise[1],
                                             wrapToPi(heading+angle),
                                             self.radar_range,
                                             sensor='radar',
                                             show_ray=show_ray)
            if collision:
                r = normal(r, self.radar_covariance[0])
            ranges += [r]
            collisions += [collision]
            
        self.radar= (ranges, collisions)
        
        return self.radar

    
class Map(object):
    def __init__(self, filename, resolution, occlusion_prob=0.01, origin='centre'):
        img = Image.open(filename)
        img = img.convert('L') # to gray image
        self.gridmap = (255 - np.asarray(img))/255
        self.occlusion_prob = occlusion_prob
        self.copy= np.copy(self.gridmap)
        self.resolution = resolution   # map resolution, m per pixel
        self.shape = self.gridmap.shape
        self.origin = origin            
        if origin not in ['centre', 'bottom-left']:
            raise ValueError('origin must be either \'centre\' or \'bottom-left\'')
            
    def to_world (self, i, j):
        '''
        Convert from  grid coordinates (row, column) to world coordinates (x, y) in m
        '''
        if self.origin == 'centre':
            origin = [int(x/2) for x in self.shape]   
        elif self.origin == 'bottom-left':
            origin = [0 for x in self.shape]
        
        dif_rows = i - origin[0]
        dif_cols = j - origin[1]
        x, y = self.resolution*dif_cols, -self.resolution*dif_rows
        return x, y

    def to_map (self, x, y):
        '''
        Convert from world coordinates (x, y) in m to grid coordinates (row, column)
        '''
        if self.origin == 'centre':
            origin = [int(x/2) for x in self.shape]   
        elif self.origin == 'bottom-left':
            origin = [0 for x in self.shape]
            
        row = int(-y/self.resolution) + origin[0]
        
        col = int(x/self.resolution) + origin[1]
        
        return row, col

    def is_inside (self, i, j):
        return i<self.gridmap.shape[0] and j<self.gridmap.shape[1] and i>=0 and j>=0

    def get_borders(self):
        return [0.0, self.gridmap.shape[1] * self.resolution, 0.0, self.gridmap.shape[0] * self.resolution]

    def raycast(self, x0, y0, theta, max_dist, sensor='laser', show_ray=False):
        
        
        #calculate pixel/occupancy locations of start and end points in world units
        x1 = x0 + max_dist*np.cos(theta)
        y1 = y0 + max_dist*np.sin(theta)
        
        i0, j0 = self.to_map(x0, y0)
        i1, j1 = self.to_map(x1, y1)
        
        dx = np.absolute(j1-j0)
        dy = -1 * np.absolute(i1-i0)
        sx = 1 if j0<j1 else -1
        sy = 1 if i0<i1 else -1
               
        err = dx+dy
        i_current, j_current = i0, j0
        row_list = []
        col_list = []
        collision = False
        n_steps = 0
        while True:            
            dist = np.sqrt((j_current - j0)**2 + (i_current - i0)**2)*self.resolution
            if dist > max_dist or not self.is_inside(i_current, j_current):
                break
            
            if show_ray:
                row_list += [i_current]
                col_list += [j_current]
            
            grid_value = self.gridmap[int(i_current)][int(j_current)]
            if  grid_value == 1:
                collision = True
                break
            
            if sensor == 'laser' and grid_value > 0:
                if uniform() < self.occlusion_prob:
                    collision = True
                    break
            
            e2 = 2*err
            if e2 >= dy:
                if j_current == j1:
                    break
                err += dy
                j_current += sx
            if e2 <= dx:
                if i_current == i1:
                    break
                err += dx
                i_current += sy
                
            
                
            n_steps += 1
                
                
        if show_ray:
            for i_current, j_current in zip(row_list, col_list):
                self.copy[i_current, j_current] = 1
            self.show_map()
        
                
        return dist, collision
    
    def show_map(self):
        plt.imshow(self.copy)
        #plt.pause(0.001)
        
        # Save map instead cuz no GUI on vs code
        plt.savefig("map_test.png")
        
    def reset_plot(self):
        self.copy = np.copy(self.gridmap)


if __name__ == "__main__":
    #Load in the true map of obstacles from an image
    
    img_file = 'map1_cloudy.png'
    resolution = 0.02 #m per pixel
    true_map = Map(img_file, resolution, origin='centre')
    
    
    #raycast test
    laser_range = 5
    n_steps = 10
    poses = np.zeros((n_steps, 3))
    for i in range(n_steps):
        poses[i, :] = [0, 0, wrapToPi(d2r(360*i/n_steps))]
        # x, y, theta = poses[i, 0], poses[i, 1], poses[i, 2]
        # dist, collision = true_map.raycast(x, y, theta, laser_range, show_ray=True)
        # print('ray at heading {0} collided {2} at distance {1}'.format(theta, dist, collision))
   
    #robot measurements test
    robot = Robot(pose=poses[0], true_map=true_map, path=poses)
    laser_scan, radar_scan = robot.step(show_ray=True)
    #robot path test
    
    path_file = open('map1_path.txt', 'w')
    x, y = 0,0
    px_x, px_y = 400,350
    while px_y < 600:
        px_y += 30
        x,y = true_map.to_world(px_y, px_x)
        path_file.write('{},{},{}\n'.format(x, y, d2r(-90)))
    path_file.write('{},{},{}\n'.format(x, y, d2r(-45)))
    while px_x < 750:
        px_x += 30
        x,y = true_map.to_world(px_y, px_x)
        path_file.write('{},{},{}\n'.format(x, y, d2r(0)))
    path_file.write('{},{},{}\n'.format(x, y, d2r(45)))
        
    while px_y > 120:
        px_y -= 30
        x,y = true_map.to_world(px_y, px_x)
        path_file.write('{},{},{}\n'.format(x, y, d2r(90)))
    path_file.write('{},{},{}\n'.format(x, y, d2r(135)))
    
    while px_x > 175:
        px_x -= 30
        x,y = true_map.to_world(px_y, px_x)
        path_file.write('{},{},{}\n'.format(x, y, d2r(180)))
        
    path_file.write('{},{},{}\n'.format(x, y, d2r(-135)))
    while px_y < 600:
        px_y += 30
        x,y = true_map.to_world(px_y, px_x)
        path_file.write('{},{},{}\n'.format(x, y, d2r(-90)))
    path_file.close()
        
    
    
    


