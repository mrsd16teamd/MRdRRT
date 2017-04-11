import operator
from collections import defaultdict
import itertools


class Tree(object):
    """Tree structure, meant for use with implicit graph in MRdRRT.
    In this class, 'configuration' refers to a list of PRM node IDs, which
    correspond to positions of the robots.
    Adjacency list representation.
    """

    def __init__(self, planning_env, implicit_graph):
        self.vertices = []      # vertices of node ids
        self.edges = dict()
        self.env = planning_env
        self.implicitgraph = implicit_graph

    def AddVertex(self, config):
        """Add vertex to tree."""
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def AddEdge(self, sid, eid):
        """Add edge to tree.
        Each node points to its parent (where it came from), which helps for
        reconstructing path at end.
        """
        self.edges[eid] = sid

    def NearestNeighbors(self, config, K):
        """Given composite configuration, find K closest ones in current tree.
        """
        # TODO Actually support K nearest neighbors instead of just one
        # For now, we're just doing nearest neighbor

        min_dist = float("inf")
        nearest = None
        nid = None

        for vid, node in enumerate(self.vertices):
            node_config = self.implicitgraph.NodeIdsToConfigs(node)
            dist = self.implicitgraph.ComputeCompositeDistance(node_config, config)
            if (dist < min_dist):
                dist = min_dist
                nearest = node
                nid = vid

        return nearest, nid
