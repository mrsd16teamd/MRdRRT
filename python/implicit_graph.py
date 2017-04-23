from simple_environment import SimpleEnvironment
from prm_planner import PRMPlanner
from prm_graph import Graph

import numpy as np
import itertools
import random
from profile_utils import timefunc

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

    @timefunc
    def GetNeighbors(self, node):
        """Returns list of neighbors for node in implicit graph.
        Input: node as set of IDs for each PRM (list)
        Output: list of nodes (list of PRM node ids)
        """
        neighbors_of_each = []  # list of lists

        for i in range(len(node)):
            neighbors_of_each.append(self.roadmap.edges[node[i]])
            neighbors_of_each[i].append(node[i])
            # TODO remove this hack that's here to make things faster
            # Without this, number of neighbors gets to order of 10e6 with 4 robots
            if len(neighbors_of_each[i]) > 30:
                neighbors_of_each[i] = random.sample(neighbors_of_each[i], 30)

        # Return all possible combinations of neighbors
        neighbors = list(itertools.product(*neighbors_of_each))

        return neighbors


    # @timefunc
    # def GetClosestCompositeConfig(self, qnear, qrand):
    #     """ Given randomly sampled comp config and
    #     nearest config on current tree, find closest neighbor of qnear
    #     """
    #
    #     neighbors_of_each = []
    #     for i in range(len(qnear)):
    #         neighbors_of_each.append(self.roadmap.edges[qnear[i]])
    #         neighbors_of_each[i].append(qnear[i])
    #         # TODO remove this hack that's here to make things faster
    #         # Without this, number of neighbors gets to order of 10e6 with 4 robots
    #         if len(neighbors_of_each[i]) > 30:
    #             neighbors_of_each[i] = random.sample(neighbors_of_each[i], 30)
    #
    #     # Return all possible combinations of neighbors
    #     neighbor_ids = np.array(list(itertools.product(*neighbors_of_each)))
    #     neighbor_ids.flatten()
    #     neighbor_configs = []
    #     for nid in neighbor_ids:
    #         neighbor_configs.append(self.roadmap.vertices[nid])
    #     neighbor_configs = np.array(neighbor_configs)
    #
    #     # Find closest composite neighbor




    @timefunc
    def GetClosestCompositeNeighbor(self, qnear, qrand):
        """ Given randomly sampled comp config and
        nearest config on current tree, find closest neighbor of qnear
        """
        nearest = ()
        all_neighbors = []
        for i in range(len(qnear)):
            all_neighbors.append(self.roadmap.edges[qnear[i]])
            all_neighbors[i].append(qnear[i])

        for neighbors_of_each in (all_neighbors):
            dataset = []
            for nid in neighbors_of_each:
                dataset.append(self.roadmap.vertices[nid])
            nn_id, _ = self.roadmap.flann.nn(np.array(dataset), qrand[i], 1)
            nearest += (neighbors_of_each[nn_id[0]],)

        print nearest

        return nearest
