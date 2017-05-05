#!/usr/bin/env python

import numpy as np
import cPickle as pickle

# from simple_robot import SimpleRobot
from simple_environment import SimpleEnvironment
from prm_planner import PRMPlanner
from mrdrrt_planner import MRdRRTPlanner

# assert 1>5, "it works!" TODO add these throughout code


def main():
    print("Starting...")
    # Map IDs: 1=cube center, 2=T seven node, 3=T four node
    prm = PRMPlanner(n_nodes=300, map_id=2, load=True, visualize=True)

    test = 4

    if test == 1:
        # Test 1 robot in cube map
        sconfigs = np.array([[0, -0.3]])
        gconfigs = np.array([[0.3, 0]])

    if test == 2:
        # Test 2 robots in cube map
        sconfigs = np.array([[0, 0.3], [-0.4, -0.3]])
        gconfigs = np.array([[0.4, -0.3], [0, 0.2]])

    if test == 3:
        # Test 2 robots in T map
        # sconfigs = np.array([[-0.25, -0.05], [0.25, -0.05]])
        # gconfigs = np.array([[0.15, -0.05], [-0.15, -0.05]])

        sconfigs = np.array([[-0.16, -0.05], [ 0.16, -0.05]])
        gconfigs = np.array([[ 0.15,  -0.05], [-0.15,  -0.05]])

    if test == 4:
        # Test 3 robots in T map
        sconfigs = np.array([[-0.25, -0.05], [0.25, -0.05], [0, 0.25]])
        gconfigs = np.array([[0.30, -0.05], [0, 0.30], [-0.30, -0.05]])

    if test == 5:
        # Test 4 robots in T map
        sconfigs = np.array([[-0.25, -0.35], [5, -0.3], [0, 0], [0.2, -0.35]])
        gconfigs = np.array([[0.05, 0.3], [0.15, 0.35], [0.4, -0.35], [-0.2, -38]])

    if test == 6:
        # Test 5 robots in T map
        sconfigs = np.array([[-0.4, -0.35], [5, -0.3], [0, 0], [0.2, -0.35], [0.35, -0.3]])
        gconfigs = np.array([[0.05, 0.3], [0.15, 0.35], [0.4, -0.35], [-0.2, -0.38], [-0.05, 0.45]])

    mrdrrt = MRdRRTPlanner(prm, n_robots=sconfigs.shape[0], visualize=True)
    path = mrdrrt.FindPath(sconfigs, gconfigs)
    print(path)

    filepath = '../paths/tmap_path2.p'
    with open(filepath, "wb") as f:
        pickle.dump(path, f)
        print("Saved MRdRRT path.")

if __name__ == "__main__":
    main()

    # import IPython
    # IPython.embed()
