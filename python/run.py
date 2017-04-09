#!/usr/bin/env python

import numpy as np

# from simple_robot import SimpleRobot
from simple_environment import SimpleEnvironment
from prm_planner import PRMPlanner
from mrdrrt_planner import MRdRRTPlanner


def main():
    print("Starting MRdRRT program.")
    # assert 1>5, "it works!"
    prm = PRMPlanner(N=300, load=True, visualize=True)
    raw_input("check roadmap and obstacles")

    test = 3

    if test == 1:
        # Test 2 robots
        mrdrrt = MRdRRTPlanner(prm, 2, visualize=True)
        sconfigs = np.array([ [0,30], [-40,-30]])
        gconfigs = np.array([ [40,-30], [0, 20] ])

    if test == 2:
        # Test 2 robots, for block map
        mrdrrt = MRdRRTPlanner(prm, 2, visualize=True)
        sconfigs = np.array([ [30,5], [5,30]])
        gconfigs = np.array([ [-30,5], [5,-30] ])

    if test == 3:
        # Test 3 robots
        mrdrrt = MRdRRTPlanner(prm, 3, visualize=True)
        sconfigs = np.array([ [30,-25], [5,30], [0, 0]])
        gconfigs = np.array([ [-30,-30], [5,-30], [40, -35] ])

    path = mrdrrt.FindPath(sconfigs, gconfigs)

if __name__ == "__main__":
    main()

    # import IPython
    # IPython.embed()
