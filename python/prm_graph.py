import operator
import pyflann
import numpy as np
from collections import defaultdict

class Graph(object):
    """Graph structure with adjacency list representation for PRM."""

    def __init__(self, planning_env):
        self.vertices = []
        self.edges = defaultdict(list)
        self.env = planning_env
        self.flann = pyflann.FLANN()
        pyflann.set_distance_type('euclidean')
        self.built_flann_index = False

    def AddVertex(self, config):
        """Add vertex to list of nodes."""
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def AddEdge(self, sid, eid):
        """Add edge to adjacency list of both nodes."""
        self.edges[eid].append(sid)
        self.edges[sid].append(eid)

    def GetNeighbors(self, node_id, neighbor_dist_thres=0.055, max_neighbors=6):
        """Given node ID, retunrs the node ID of closest nodes on roadmap."""

        neighbor_ids = []
        neighbor_configs = []

        if len(self.vertices) > 2:
            test_config = self.vertices[node_id]
            max_neighbors = min(len(self.vertices), max_neighbors)

            if self.built_flann_index:
                nn_ids, dists = self.flann.nn_index(test_config, max_neighbors)
            else:
                nn_ids, dists = self.flann.nn(np.array(self.vertices), test_config, max_neighbors)

            nn_ids = (nn_ids.squeeze()).tolist()
            dists = (dists.squeeze()).tolist()

            for i, neighbor_id in enumerate(nn_ids):
                if dists[i] < neighbor_dist_thres  and dists[i] > 0.0:
                    neighbor_configs.append(self.vertices[neighbor_id])
                    neighbor_ids.append(neighbor_id)

        return neighbor_ids, neighbor_configs

    def GetNearestNode(self, config):
        """Returns vid of nearest node in graph"""

        if self.built_flann_index:
            nn_ids, _ = self.flann.nn_index(config, 2)
        else:
            nn_ids, _ = self.flann.nn(np.array(self.vertices), config, 2)

        nn_ids = (nn_ids.squeeze()).tolist()
        min_id = nn_ids[1]

        return min_id

    def FindMinDistNode(self, node_set, dist):
        """Utility for Djikstra - finds node ID to be explored next"""
        min_dist = 9999
        min_id = node_set[0]
        for node in node_set:
            if (dist[node] < min_dist):
                min_dist = dist[node]
                min_id = node
        return min_id

    def Djikstra(self, start_id, end_id):
        """Standard Djikstra search to find path in PRM roadmap.
        Takes queries in node id form and returns path in configs
        """
        nodes = self.vertices

        dist = [9999 for node in nodes]
        prev = [9999 for node in nodes]
        Q = [i for i,v in enumerate(nodes)]  # All nodes are unvisited at first

        dist[start_id] = 0

        while len(Q) !=  0:
            u = self.FindMinDistNode(Q, dist)
            Q.remove(u)  # make u as visited

            for v in self.edges[u]:
                alt = dist[u] + self.env.ComputeDistance(self.vertices[u], self.vertices[v])
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u

        path = [self.vertices[end_id]]
        target = end_id
        while prev[target] != 9999:
            target = prev[target]
            path.append(self.vertices[target])

        path.reverse()
        if (path[0] == self.vertices[start_id]).any():
            print("Found a path.")
            return path
        else:
            print("Could not find path.")
            return []
