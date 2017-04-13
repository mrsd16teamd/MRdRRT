import numpy as np
import pylab as pl
import time

from prm_graph import Graph
from simple_environment import SimpleEnvironment
from prm_planner import PRMPlanner
from mrdrrt_tree import Tree
from implicit_graph import ImplicitGraph

from profile_utils import timefunc


class MRdRRTPlanner(object):
    """Multi-robot discrete RRT algorithm for coordinated centralized planning.

    Simple implementation of:

    Solovey, Kiril, Oren Salzman, and Dan Halperin.
    "Finding a needle in an exponential haystack: Discrete RRT for exploration
    of implicit roadmaps in multi-robot motion planning." Algorithmic Foundations
    of Robotics XI. Springer International Publishing, 2015. 591-607.
    """

    def __init__(self, prm, n_robots=2, visualize=False):
        self.n_robots = n_robots
        self.env = prm.env
        self.implicitgraph = ImplicitGraph(self.env, prm, n_robots)
        self.tree = Tree(self.env, self.implicitgraph)
        self.max_iter = 500
        self.prm = prm  # Here just for VisualizePlot
        self.visualize = visualize

    @timefunc
    def FindClosestToConfig(self, config1, neighbors):
        min_dist = float("inf")
        nearest = None

        for node in neighbors:
            config2 = self.implicitgraph.NodeIdsToConfigs(node)
            dist = self.implicitgraph.ComputeCompositeDistance(config2, config1)
            if(dist < min_dist):
                min_dist = dist
                nearest = node

        return min_dist, nearest

    @timefunc
    def Oracle(self, qnear, qrand):
        """Direction oracle, as defined in Oren's paper.
        Given randomly sampled comp config and nearest config on current tree,
        return qnew, a neighbor of qnear on the implicit graph that hasn't been
        explored yet, and is closest (by sum of euclidean distances) to qnear.
        """
        neighbors = self.implicitgraph.GetNeighbors(qnear)

        min_dist, nearest = self.FindClosestToConfig(qrand, neighbors)

        # check collision between qnear and nearest node
        # TODO clean this up, move somewhere else
        if nearest:
            nearest_config = self.implicitgraph.NodeIdsToConfigs(nearest)
            for i in range(len(qrand)):
                config1 = qrand[i]
                config2 = nearest_config[i]
                if self.env.CollisionOnLine(config1, config2):
                    return None

        return nearest

    @timefunc
    def Expand(self):
        """Takes random sample and tries to expand tree in direction of sample.
        """
        qrand = self.implicitgraph.RandomSample()
        qnear, near_id = self.tree.NearestNeighbors(qrand, 1)

        qnew = self.Oracle(qnear, qrand)
        if (qnew is not None and qnew not in self.tree.vertices):
            new_id = self.tree.AddVertex(qnew)
            self.tree.AddEdge(near_id, new_id)

    @timefunc
    def LocalConnector(self, config1, config2):
        """Check for collision free path between two composite configs.
        Given two composite configurations, check if collision free movement
        between them is possible. If coordinated movement/ordering is required,
        return ordering of robots as list.
        Doing naive implementation first, where robots are moved along path by
        constant phase/alpha from start to end configs and check collision at
        each point.
        Later, do more complicated one.
        Input: lists of node IDs
        """
        n_steps = 10
        steps = [i/float(n_steps) for i in range(n_steps+1)]

        for step in steps:
            q = config1 + (config2-config1)*step
            collision = self.env.CheckCollisionMultiple(q)
            if collision:
                return False  # couldn't connect

        return True  # Connection succeeded!

    @timefunc
    def ConnectToTarget(self, gids):
        """Check if it's possible to get to goal from closest nodes in current tree.
        Called at the end of each iteration.
        Input: list of goal configurations (goal composite config)
        """
        # TODO implement k-nearest neighbors...
        # Actually, how important is this?
        # Only checking closest node right now because that's all nearestneighbors does
        # for q in self.tree.NearestNeighbors(goal,1):
        g_config = self.implicitgraph.NodeIdsToConfigs(gids)
        neighbor, nid = self.tree.NearestNeighbors(g_config, 1)
        n_config = self.implicitgraph.NodeIdsToConfigs(neighbor)
        success = self.LocalConnector(n_config, g_config)
        return success, nid

    def ConstructPath(self, neighbor_of_goal, sconfigs, gconfigs, sids, gids):
        """Returns final path thru implicit graph to get from start to goal.
        Called when a collision-free path to goal config is found.
        Inputs:
            neighbor_of_goal: (node IDs) node that was successfully connected to goal
            gconfigs: list of configurations of final goal
        """
        path = [gconfigs]
        path.append(self.implicitgraph.NodeIdsToConfigs(gids))

        # Follow pointers to parents from goal to reconstruct path, reverse
        node_id = neighbor_of_goal
        while (node_id in self.tree.edges.keys()):  # TODO find better end condition
            node = self.tree.vertices[node_id]
            path.append(self.implicitgraph.NodeIdsToConfigs(node))
            node_id = self.tree.edges[node_id]

        path.append(self.implicitgraph.NodeIdsToConfigs(sids))
        path.append(sconfigs)
        path.reverse()
        return path

    def VisualizePath(self, path):
        """Plot paths of robots through environment, defined in path.
        """
        colors = ['r-', 'g-', 'b-', 'm-', 'y-']

        if not pl.get_fignums():
            self.env.InitializePlot()
            self.prm.PlotRoadmap()
        for robot in range(len(path[0])):
            robot_path = []
            for i in range(len(path)):
                robot_path.append(path[i][robot, :])
            self.prm.VisualizePath(robot_path, colors[robot])
        raw_input("Check paths")

    def ShowStartAndGoalConfigs(self, sconfigs, gconfigs):
        colors = ['r', 'g', 'b', 'm', 'y']
        for i in range(len(sconfigs)):
            self.env.PlotPoint(sconfigs[i], color=colors[i], size=10)
            self.env.PlotPoint(gconfigs[i], color=colors[i], size=10)

    def AnimatePath(self, path):
        """Same thing as VisualizePath, except animated from start to goal.
        Should help for noticing collisions between robots mid-path.
        """
        pass

    def FindPath(self, sconfigs, gconfigs):
        """Main function for MRdRRT. Expands tree to find path from start to goal.
        Inputs: list of start and goal configs for robots.
        """
        if len(sconfigs) != len(gconfigs):
            print("Start and goal configurations don't match in length")
            return

        self.ShowStartAndGoalConfigs(sconfigs, gconfigs)
        raw_input("Wait for start/goal configs, and enter to start planning")

        for i in range(len(sconfigs)):
            if self.env.CheckCollision(sconfigs[i]) or self.env.CheckCollision(gconfigs[i]):
                print("Start or goal configurations are in collision.")
                return

        print("Looking for a path...")
        sids = self.implicitgraph.NearestNodeInGraph(sconfigs)
        gids = self.implicitgraph.NearestNodeInGraph(gconfigs)

        # Put start config in tree
        self.tree.AddVertex(sids)

        i = 0
        while (i < self.max_iter):
            self.Expand()
            success, nid = self.ConnectToTarget(gids)
            if success:
                print("Found a path! Constructing final path now..")
                path = self.ConstructPath(nid, sconfigs, gconfigs, sids, gids)
                break

            if(i % 10 == 0):
                print(str(i) + "th iteration")
            i += 1
            print('-------')

        if (i == self.max_iter):
            print("Failed to find path - hit maximum iterations.")
        else:
            if self.visualize:
                self.VisualizePath(path)
