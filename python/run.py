#!/usr/bin/env python

import argparse, time
import numpy as np
import sys

# from SimpleRobot import SimpleRobot
from SimpleEnvironment import SimpleEnvironment
from PRMPlanner import PRMPlanner
from MRdRRTPlanner import MRdRRTPlanner

if __name__ == "__main__":
    prm = PRMPlanner(N=300, load=True, visualize=False)
    # prm.env.InitializePlot()
    # prm.PlotRoadmap()

    # raw_input("stop")
    # import IPython
    # IPython.embed()


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

    path = mrdrrt.FindPath(sconfigs, gconfigs)
    mrdrrt.VisualizePath(path)


    # import IPython
    # IPython.embed()

    #  parser = argparse.ArgumentParser(description='script for testing planners')
     #
    #  parser.add_argument('-r', '--robot', type=str, default='simple',
    #                      help='The robot to load (herb or simple)')
    #  parser.add_argument('-p', '--planner', type=str, default='astar',
    #                      help='The planner to run (astar, bfs, dfs or hrrt)')
    #  parser.add_argument('-v', '--visualize', action='store_true',
    #                      help='Enable visualization of tree growth (only applicable for simple robot)')
    #  parser.add_argument('--resolution', type=float, default=0.1,
    #                      help='Set the resolution of the grid (default: 0.1)')
    #  parser.add_argument('-d', '--debug', action='store_true',
    #                      help='Enable debug logging')
    #  parser.add_argument('-m', '--manip', type=str, default='right',
    #                      help='The manipulator to plan with (right or left) - only applicable if robot is of type herb')
    #  args = parser.parse_args()
