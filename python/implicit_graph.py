from simple_environment import SimpleEnvironment
from prm_planner import PRMPlanner
from prm_graph import Graph

import numpy as np
import itertools


class ImplicitGraph(object):
    """Defines implicit graph (composite of PRM roadmaps) for MRdRRT."""

    def __init__(self, env, roadmap, n_robots=2):
        """Loads PRM roadmap that will define implicit graph."""
        self.roadmap = roadmap.graph
        self.env = env
        self.N = n_robots

    def RandomSample(self):
        """Returns array of randomly sampled node in composite config space."""
        c_config = []
        for i in range(self.N):
            c_config.append(self.env.SampleConfig())
        return np.array(c_config)

    def NodeIdsToConfigs(self, ids):
        """Converts composite node w/ node IDs to array of configurations."""
        configs = []
        for i in range(len(ids)):
            configs.append(self.roadmap.vertices[ids[i]])
        return np.array(configs)

    def ComputeCompositeDistance(self, config1, config2):
        """Computes distance in "composite configuration space".
        Defined as sum of Euclidean distances between PRM nodes in two configs.
        """
        dist = 0
        for i in range(len(config1)):
            dist += self.env.ComputeDistance(config1[i], config2[i])
        return dist

    def NearestNodeInGraph(self, config):
        """Returns nearest node in implicit graph to a composite configuration.
        Input: list of configurations
        Output: list of node IDs of closest node on implicit graph
        """
        nearest = []
        for i in range(len(config)):
            nearest.append(self.roadmap.GetNearestNode(config[i]))
        return nearest

    def GetNeighbors(self, node):
        """Returns list of neighbors for node in implicit graph.
        Input: node as set of IDs for each PRM (list)
        Output: list of nodes (list of PRM node ids)
        """
        neighbors_of_each = []  # list of lists
        for i in range(len(node)):
            neighbors_of_each.append(self.roadmap.edges[node[i]])

        # Return all possible combinations of neighbors
        neighbors = list(itertools.product(*neighbors_of_each))
        return neighbors

    def GetPathFromTree(self):
        # TODO
        pass
