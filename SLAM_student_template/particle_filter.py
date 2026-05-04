# -*- coding: utf-8 -*-

from dbm import error

import numpy as np
from util_functions import *

class ParticleFilter:
    def __init__(self, mu, Sigma, n_particles=500, default_weight=1, add_noise=True, nlandmarks=10):
        self.default_weight = default_weight
        self.n_particles = n_particles
        self.n_resample = int(self.n_particles/10)
        
        self.alphas = np.ones(self.n_particles) * (1/self.n_particles)
        self.ones = np.ones_like(self.alphas)
        self.add_noise = add_noise
        
        self.particles = np.zeros((self.n_particles, 3))
        if self.add_noise:
            self.particles[:,0] = np.random.normal(mu[0], Sigma[0], self.n_particles)
            self.particles[:,1] = np.random.normal(mu[1], Sigma[1], self.n_particles)
            self.particles[:,2] = np.random.normal(mu[2], Sigma[2], self.n_particles)
        else:
            self.particles[:, 0] = mu[0]
            self.particles[:, 1] = mu[1]
            self.particles[:, 2] = mu[2]
        self.local_map = np.zeros((self.n_particles, nlandmarks, 2))
        self.global_map = np.zeros((nlandmarks, 2))
        self.weights = np.ones(self.n_particles)

    def predict_all(self, d, dth, R):
        # add noise
        if self.add_noise:
            noisy_d = d + np.random.normal(0, R[0,0], self.n_particles)
            noisy_dth = dth + np.random.normal(0, R[1,1], self.n_particles)
        else:
            noisy_d = d
            noisy_dth = dth * self.ones

        # use motion model to update particles poses based on the noisy odometer readings
        self.particles[:, 0] += noisy_d * np.cos(self.particles[:, 2])
        self.particles[:, 1] += noisy_d * np.sin(self.particles[:, 2])
        self.particles[:, 2] += noisy_dth

        return self.particles
        

    def update(self, z, global_map=None, resample=False):
        '''
        Inputs:
            z: sensor measurements, (r,b) to each landmark
            global_map: best estimate of robot pose and landmark positions before update. If input is None,
                    uses self.global_map
        Outputs:
            self.weights is updated
            global_map: updated and returns
        '''
        set_self_map = False
        if global_map is None:
            global_map = self.global_map
            set_self_map = True
        

        #perform a particle filter update, involving updating particle weights, resampling (if necessary), and then calculting
        #positions of the landmarks based on sensor readings and updated particle weights
        
        # update weights based on how well particles predict sensor measurements
        self.update_weights(z, global_map)
        
        # resample if necessary
        if resample:
            self.resample()
        
        # calculate landmark positions based on sensor readings and updated particle weights
        
        #create an empty global map to fill in
        new_global_map = np.zeros_like(global_map)
        
        # recalculate landmark positions from each particle

        for landmark_id, (r, b) in enumerate(z):
            # convert range and bearing measurements to x and y coordinates landmark estimates based on each particle's pose
            x, y = rangebearing_to_xy(self.particles, r, b)

            # save that landmark position to the local map for that particle
            self.local_map[:, landmark_id, 0] = x
            self.local_map[:, landmark_id, 1] = y
            
            # add the landmark estimate into the new global map, weighted by the particle's weight (so better particles contribute more to the global map)
            new_global_map[landmark_id, 0] = np.sum(self.weights * x)
            new_global_map[landmark_id, 1] = np.sum(self.weights * y)

        # after all particles are processed, normalize or finish the weighted average and store it as the new global map
        global_map = new_global_map
        self.global_map = new_global_map
        
        return global_map
        
        
    def update_weights(self, z, global_map):
        '''
        As part of the update step in PF SLAM, we need to update our weights
        for each particle according to how well new measurements line up with
        their positions
        
        To be called in self.update() method
        
        '''
        new_weights = np.ones(self.n_particles)

        for particle_idx in range(self.n_particles):
            particle = self.particles[particle_idx, :]
            particle[2] = wrap_to_pi(particle[2])

            for landmark_id, (range_meas, bearing_meas) in enumerate(z):
                landmark = global_map[landmark_id]

                dx = landmark[0] - particle[0]
                dy = landmark[1] - particle[1]

                pred_range = np.sqrt(dx**2 + dy**2)
                pred_bearing = wrap_to_pi(np.arctan2(dy, dx) - particle[2])

                range_error = pred_range - range_meas
                bearing_error = wrap_to_pi(pred_bearing - bearing_meas)

                # Use a Gaussian-like likelihood so better matches get larger weights.
                lambda_r = 1.0
                lambda_b = 1.0
                new_weights[particle_idx] *= np.exp(
                    -0.5 * (
                        lambda_r * range_error**2 +
                        lambda_b * bearing_error**2
                    )
                )

        total = np.sum(new_weights)

        if total == 0:
            self.weights = np.ones(self.n_particles) / self.n_particles
        else:
            self.weights = new_weights / total
        
        return self.weights
            
            


      
    
    def init_landmarks(self, z):
        '''
        Inputs:
            self: look at the initialization method to see what properties this
                class has
            z: sensor measurements of range and bearing to all landmarks
        Output:
            Global maps of landmark positions (should be a numpy array of size (2,2xn))
            save local maps for each particle to self.map (should be numpy array of size (n, 2, 2xn))
        '''
        #look at predict_all to see how the robot's position is estimated from
        #each particle. How can you use this idea to estimate landmark positions?
        global_map = np.zeros_like(self.global_map)

        for particle_idx in range(self.n_particles):
            for landmark_id, (r, b) in enumerate(z):
                
                #calculate position of landmark based on particle and sensor
                bearing = wrap_to_pi(self.particles[particle_idx, 2] + b)
                x = self.particles[particle_idx, 0] + r * np.cos(bearing)
                y = self.particles[particle_idx, 1] + r * np.sin(bearing)

                #Get position of landmark this particle predicts
                self.local_map[particle_idx, landmark_id, :] = [x, y]

                #add it to global map (then average after end of loop)
                global_map[landmark_id, :] += [x, y]
                

                
        global_map /= self.n_particles
        self.global_map = global_map
        return global_map
        
    def resample(self):
        '''
        Handle particle resampling here. In the initialization method, n_resample
        is defined and represents the number of particles we want to resample
        because we aren't confident in their estimate of robot pose.
        
        This function resamples those particle positions according to where we
        think our robot is.

        Outputs:
            self.particles and self.weights is updated (but not directly returned)

        '''
        weight_sum = np.sum(self.weights)

        if weight_sum <= 1e-12:
            self.weights = np.ones(self.n_particles) / self.n_particles
        else:
            self.weights = self.weights / weight_sum

        sorted_indices = np.argsort(self.weights)

        worst_indices = sorted_indices[:self.n_resample]
        best_indices = sorted_indices[self.n_resample:]

        best_particles = self.particles[best_indices]
        best_weights = self.weights[best_indices]
        best_weights = best_weights / np.sum(best_weights)

        mean_x = np.sum(best_weights * best_particles[:, 0])
        mean_y = np.sum(best_weights * best_particles[:, 1])

        mean_theta = np.arctan2(
            np.sum(best_weights * np.sin(best_particles[:, 2])),
            np.sum(best_weights * np.cos(best_particles[:, 2]))
        )

        mean_pose = np.array([mean_x, mean_y, mean_theta])

        std_x = np.sqrt(np.sum(best_weights * (best_particles[:, 0] - mean_x)**2))
        std_y = np.sqrt(np.sum(best_weights * (best_particles[:, 1] - mean_y)**2))

        theta_error = wrap_nparray_to_pi(best_particles[:, 2] - mean_theta)
        std_theta = np.sqrt(np.sum(best_weights * theta_error**2))

        std_pose = np.array([
            max(std_x, 0.05),
            max(std_y, 0.05),
            max(std_theta, 0.02)
        ])

        new_samples = np.random.normal(
            loc=mean_pose,
            scale=std_pose,
            size=(self.n_resample, 3)
        )

        new_samples[:, 2] = wrap_nparray_to_pi(new_samples[:, 2])

        self.particles[worst_indices] = new_samples

        self.weights = np.ones(self.n_particles) / self.n_particles
        
        return
        



