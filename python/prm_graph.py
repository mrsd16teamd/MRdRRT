import operator
from collections import defaultdict


class Graph(object):
    """Graph structure with adjacency list representation for PRM."""

    def __init__(self, planning_env):
        self.vertices = []
        self.edges = defaultdict(list)
        self.env = planning_env

    def AddVertex(self, config):
        """Add vertex to list of nodes."""
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def AddEdge(self, sid, eid):
        """Add edge to adjacency list of both nodes."""
        self.edges[eid].append(sid)
        self.edges[sid].append(eid)

    def GetNeighbors(self, node_id, neighbor_dist_thres=10, max_neighbors=5):
        """Given node ID, retunrs the node ID of closest node on roadmap."""
        # TODO replace with FLANN
        config = self.vertices[node_id]
        neighbor_ids = []
        neighbor_configs = []

        for vid, v in enumerate(self.vertices):
            dist = self.env.ComputeDistance(config, v)
            if (dist < neighbor_dist_thres and dist != 0.0):
                neighbor_ids.append(vid)
                neighbor_configs.append(v)
                if len(neighbor_ids) > max_neighbors:
                    break

        return neighbor_ids, neighbor_configs

    def GetNearestNode(self, config):
        """Returns vid of nearest node in graph"""
        min_dist = 9999
        min_id = 0
        for vid, v in enumerate(self.vertices):
            if(self.env.ComputeDistance(config, v) < min_dist):
                min_dist = self.env.ComputeDistance(config, v)
                min_id = vid
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
            print "Found a path."
            return path
        else:
            print "Could not find path."
            return []
