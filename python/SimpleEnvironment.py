import numpy as np
import pylab as pl
from sys import platform

class SimpleEnvironment(object):

    def __init__(self, visualize):
        self.lower_limits = np.array([-50., -50.])    #[cm]
        self.upper_limits = np.array([50., 50.])
        self.visualize = visualize

        self.InitMap()

        # TODO add robots
        self.robots = []

    def InitMap(self):
        """Add all obstacles in map"""
        self.obstacles = []
        cube_width = 5 #[cm]
        expand_obs = 3
        w = cube_width + expand_obs

        # define vertices - make sure they're in order
        cube1 = np.array([[0,0],[w, 0],[w, w], [0, w]])
        self.obstacles.append(cube1)

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

    def CollisionOnLine(self, config1, config2):
        diff = config2 - config1
        for i in range(10):
            check_state = config1 + diff/10.0 * i
            if self.CheckCollision(check_state):
                return True
        return False

    def PlotPolygon(self, o):
        "Assume obstacle is numpy array of vertices"
        for i in range(len(o)-1, -1, -1):
            self.PlotEdge(o[i],o[i-1],'r.-')

    def InitializePlot(self):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])

        # Show all obstacles in environment
        for o in self.obstacles:
            self.PlotPolygon(o)

        #TODO plot robots
        if self.visualize:
            pl.ion()
            pl.show()

    def PlotEdge(self, sconfig, econfig, color='k-', lwidth=0.5):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                color, linewidth=lwidth)
        pl.draw()

    def PlotPoint(self, config, color='b',size=1):
        marker = color + 'o'
        pl.plot(config[0],config[1],marker,markersize=size)

if __name__ == "__main__":
    env = SimpleEnvironment()
    env.InitializePlot()

    env.PlotPoint(env.SampleConfig())

    origin = np.array([10,10])
    to = np.array([20,20])
    env.PlotEdge(origin,to)
    print(env.CollisionOnLine(origin,to))
    raw_input("")
    # for i in range(500):
    #     sample = env.SampleConfig()
    #     collision = env.CheckCollision(sample)
    #     mark_color = 'r' if collision else 'b'
    #     env.PlotPoint(sample,mark_color)
    # raw_input("Reached end.")
