import numpy as np
import pylab as pl
from sys import platform

class SimpleEnvironment(object):

    def __init__(self):
        self.lower_limits = np.array([-50., -50.])    #[cm]
        self.upper_limits = np.array([50., 50.])

        self.InitMap()

        # TODO add robots
        self.robots = []

    def InitMap(self):
        """
        Add all obstacles in map
        Currently only works for axis-aligned rectangles defined as:
        [bottom left, __, top right, __]
        """
        self.obstacles = []
        cube_width = 5 #[cm]
        expand_obs = 3
        w = cube_width + expand_obs

        # EMPTY MAP
        # define vertices - make sure they're in order
        # cube1 = np.array([[0,0],[w, 0],[w, w], [0, w]])
        # self.obstacles.append(cube1)

        # T MAP
        box1 = np.array([ [-50,-50], [-50,-40], [50,-40], [50,-50] ])
        box2 = np.array([ [-50,-20], [-50,50], [-10,50], [-10,-20] ])
        box3 = np.array([  [20,-20], [20,50], [50,50], [50, -20] ])
        self.obstacles.append(box1)
        self.obstacles.append(box2)
        self.obstacles.append(box3)


    def ComputeDistance(self, config1, config2):
        dist = np.linalg.norm(config1-config2)
        return dist

    def SampleConfig(self):
        """Generates random configuration in Cfree"""
        while True:
            rand_config = self.lower_limits + np.multiply(np.random.rand(1,2),self.upper_limits-self.lower_limits)[0]
            if (not (self.CheckCollision(rand_config))):
                return rand_config

    def CheckCollision(self, config):
        # TODO implement better method with FCL
        # This only works for axis-aligned rectangles defined as [bottom left, __, top right, __]
        for o in self.obstacles:
            bl = o[0]
            tr= o[2]
            if(config[0]>bl[0] and config[0]<tr[0]):
                if(config[1]>bl[1] and config[1]<tr[1]):
                    return True
        return False

    def CheckCollisionMultiple(self,composite_config):
        #TODO make this much better
        n_robots = len(composite_config)

        for i in range(n_robots):
            #Check robot-environment collision
            collision_r_e = self.CheckCollision(composite_config[i])
            if collision_r_e:
                return True

            #Check robot-robot collision
            for j in range(n_robots):
                if (i==j):
                    continue
                dist = self.ComputeDistance(composite_config[i], composite_config[j])
                if (dist<0.5): #TODO change this hard-coded robot radius assumption
                    return True
        return False


    #     """
    #     Given composite configuration (numpy array of configurations), check
    #     for robot-robot and robot-environment collisions
    #     """
    #     pass

    def CollisionOnLine(self, config1, config2):
        diff = config2 - config1
        for i in range(10):
            check_state = config1 + diff/10.0 * i
            if self.CheckCollision(check_state):
                return True
        return False

    def PlotPolygon(self, o):
        "Assume obstacle is numpy array of vertices"
        #TODO fill in polygon!
        for i in range(len(o)-1, -1, -1):
            self.PlotEdge(o[i],o[i-1],'r.-',3)

    def InitializePlot(self):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])

        # Show all obstacles in environment
        for o in self.obstacles:
            self.PlotPolygon(o)

        #TODO plot robots

        # if platform=="darwin":
        pl.ion()
        pl.show()

    def PlotEdge(self, sconfig, econfig, color='k--', lwidth=0.2):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                color, linewidth=lwidth)
        pl.draw()

    def PlotPoint(self, config, color='b',size=1):
        marker = color + 'o'
        pl.plot(config[0],config[1],marker,markersize=size)
