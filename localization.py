# localization.py
import numpy as np
import cv2
import math
import random
from config import *

class Map:
    def __init__(self, pgm_path, resolution=0.01):
        # resolution: meters per pixel (set to your map scale)
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError("Map load failed: " + pgm_path)
        self.grid = img  # 0..255
        self.h, self.w = img.shape
        self.res = resolution
        self.origin = (0, 0) # assume origin at top-left; adjust if needed

    def world_to_map(self, x, y):
        mx = int(x / self.res)
        my = int(y / self.res)
        return mx, self.h - my - 1

    def is_occupied(self, x, y):
        mx, my = self.world_to_map(x, y)
        if mx < 0 or my < 0 or mx >= self.w or my >= self.h:
            return True
        return self.grid[my, mx] < 128

class ParticleFilter:
    def __init__(self, map_obj, num_particles=NUM_PARTICLES):
        self.map = map_obj
        self.n = num_particles
        self.particles = np.zeros((self.n, 3))  # x, y, theta
        self.weights = np.ones(self.n) / self.n

    def init_uniform(self, x_min, x_max, y_min, y_max):
        xs = np.random.uniform(x_min, x_max, self.n)
        ys = np.random.uniform(y_min, y_max, self.n)
        thetas = np.random.uniform(-math.pi, math.pi, self.n)
        self.particles[:,0] = xs
        self.particles[:,1] = ys
        self.particles[:,2] = thetas

    def motion_update(self, delta_trans, delta_rot, trans_noise=0.02, rot_noise=0.02):
        # add noise to motion
        noisy_trans = np.random.normal(delta_trans, trans_noise, self.n)
        noisy_rot = np.random.normal(delta_rot, rot_noise, self.n)
        self.particles[:,0] += noisy_trans * np.cos(self.particles[:,2])
        self.particles[:,1] += noisy_trans * np.sin(self.particles[:,2])
        self.particles[:,2] += noisy_rot
        self.particles[:,2] = (self.particles[:,2] + np.pi) % (2*math.pi) - math.pi

    def measure_probability(self, lidar_scan, max_range=6000):
        # lidar_scan: list of (angle_deg, dist_mm)
        # We'll sample a few beams to save time
        sample_beams = lidar_scan[::max(1, len(lidar_scan)//30)]
        weights = np.zeros(self.n)
        for i in range(self.n):
            x, y, th = self.particles[i]
            prob = 1.0
            for ang_deg, dist_mm in sample_beams:
                if dist_mm == 0: continue
                ang = math.radians(ang_deg) + th
                # endpoint in world coords
                rx = x + (dist_mm/1000.0) * math.cos(ang)
                ry = y + (dist_mm/1000.0) * math.sin(ang)
                # check if map indicates occupied near endpoint
                try:
                    occupied = self.map.is_occupied(rx, ry)
                    if occupied:
                        prob *= 0.9
                    else:
                        prob *= 0.5
                except:
                    prob *= 0.1
            weights[i] = prob
        # normalize and avoid zeros
        weights += 1e-300
        weights /= np.sum(weights)
        self.weights = weights

    def resample(self):
        cdf = np.cumsum(self.weights)
        r = np.random.rand(self.n)
        idx = np.searchsorted(cdf, r)
        self.particles = self.particles[idx]
        self.weights = np.ones(self.n)/self.n

    def estimate(self):
        xm = np.average(self.particles[:,0], weights=self.weights)
        ym = np.average(self.particles[:,1], weights=self.weights)
        cos_t = np.average(np.cos(self.particles[:,2]), weights=self.weights)
        sin_t = np.average(np.sin(self.particles[:,2]), weights=self.weights)
        th = math.atan2(sin_t, cos_t)
        return xm, ym, th
