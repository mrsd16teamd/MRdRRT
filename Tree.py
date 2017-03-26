import operator
from collections import defaultdict
import itertools

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
        # Each node points to its parent/where it came from
        self.edges[eid].append(sid)

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
