#!/usr/bin/env python

import argparse, time
import numpy as np
import sys
from sys import platform

# from SimpleRobot import SimpleRobot
from SimpleEnvironment import SimpleEnvironment
from PRMPlanner import PRMPlanner
from MRdRRTPlanner import MRdRRTPlanner

if __name__ == "__main__":
    prm = PRMPlanner(N=300, load=True, visualize=False)

    prm.env.InitializePlot()
    prm.PlotRoadmap()
    raw_input("stop")


    # Test 2 robots
    mrdrrt = MRdRRTPlanner(prm, 2, visualize=False)
    # sconfigs = np.array([ [30,5], [5,30]])
    # gconfigs = np.array([ [-30,5], [5,-30] ])
    sconfigs = np.array([ [0,30], [-40,-30]])
    gconfigs = np.array([ [40,-30], [0, 20] ])

    # Test 3 robots
    # mrdrrt = MRdRRTPlanner(prm, 3, visualize=False)
    # sconfigs = np.array([ [30,5], [5,30], [30, 30]])
    # gconfigs = np.array([ [-30,5], [5,-30], [30, -20] ])

    # import IPython
    # IPython.embed()

    path = mrdrrt.FindPath(sconfigs, gconfigs)
    mrdrrt.VisualizePath(path)


    # import IPython
    # IPython.embed()
