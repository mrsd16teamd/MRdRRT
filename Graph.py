import operator

class Graph(object):

    neighbor_dist = 20
    max_neighbors = 3

    def __init__(self, planning_env):
        self.vertices = []
        self.edges = dict()
        self.env = planning_env

    def GetRootId(self):
        return 0

    def GetNeighbors(self, config):
        # TODO replace with better algorithm
        neighbor_ids = []
        neighbor_configs = []

        for vid, v in enumerate(self.vertices):
            dist = self.env.ComputeDistance(config,v)
            if (dist < self.neighbor_dist and dist != 0.0):
                neighbor_ids.append(vid)
                neighbor_configs.append(v)
                if len(neighbor_ids)>self.max_neighbors:
                    break

        return neighbor_ids, neighbor_configs

    def GetNearestNeighbor(self, config):
        #TODO return vid and v_config of nearest neighbor
        pass

    def AddVertex(self, config):
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def AddEdge(self, sid, eid):
        self.edges[eid] = sid
        self.edges[sid] = eid
