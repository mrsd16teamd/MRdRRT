import operator
from collections import defaultdict

#TODO adjust this for need of MRdRRT

class Tree(object):

    def __init__(self, planning_env):
        self.vertices = []
        self.edges = defaultdict(list)
        self.env = planning_env

    def AddVertex(self, config):
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def AddEdge(self, sid, eid):
        # TODO this should only go one way
        self.edges[eid].append(sid)
        self.edges[sid].append(eid)

    def GetNeighbors(self, node_id):
        # TODO replace with FLANN
        config = self.vertices[node_id]
        neighbor_ids = []
        neighbor_configs = []

        for vid, v in enumerate(self.vertices):
            dist = self.env.ComputeDistance(config,v)
            if (dist < neighbor_dist_thres and dist != 0.0):
                neighbor_ids.append(vid)
                neighbor_configs.append(v)
                if len(neighbor_ids)>max_neighbors:
                    break

        return neighbor_ids, neighbor_configs

    def GetNearestNode(self, config):
        #return vid and v_config of nearest node in graph
        min_dist = 9999
        min_id = 0
        for vid, v in enumerate(self.vertices):
            if(self.env.ComputeDistance(config, v) < min_dist):
                min_dist = self.env.ComputeDistance(config, v)
                min_id = vid
        return min_id

    def GetPath(self):
        # TODO
        pass
