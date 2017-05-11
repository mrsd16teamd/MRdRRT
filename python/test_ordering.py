import cPickle as pickle
from collections import defaultdict, deque
import numpy as np
from time import sleep


def kahn_topsort(graph):
    """
    From: https://algocoding.wordpress.com/2015/04/05/topological-sorting-python/
    """
    in_degree = {u: 0 for u in graph}     # determine in-degree
    for u in graph:                          # of each node
        for v in graph[u]:
            in_degree[v] += 1
    print('in_degree: {}'.format(in_degree))

    Q = deque()                 # collect nodes with zero in-degree
    for u in in_degree:
        if in_degree[u] == 0:
            Q.appendleft(u)
    print('Q: {}'.format(Q))

    L = []     # list for order of nodes
    while Q:
        u = Q.pop()          # choose node of zero in-degree
        L.append(u)          # and 'remove' it from graph
        for v in graph[u]:
            in_degree[v] -= 1
            if in_degree[v] == 0:
                Q.appendleft(v)

    if len(L) == len(graph):
        return L
    else:                    # if there is a cycle,
        return []            # then return an empty list

def IsEmptyGraph(graph):
    empty = True
    for key in graph.keys():
        if (len(graph[key]) != 0):
            empty = False
    return empty

def AddAnglesToPath(point_path):
    """Post-processes path from roadmap before sending to Cozmo.
    Add angles to path
    For angles, just points robot towards next waypoint.
    """
    path_w_angles = []
    for i, point in enumerate(point_path[0:-1]):  # First point -> second to last
        # vec2next = point_path[i+1] - point_path[i]
        # angle = np.arctan2(vec2next[1], vec2next[0])
        # config = np.append(point, angle)
        # path_w_angles.append(config)
        if point[1] > 0: # y>0
            config = np.append(point, -np.pi/2)
        elif point[0] > 0:
            config = np.append(point, np.pi)
        else:
            config = np.append(point, 0)
        path_w_angles.append(config)

    return path_w_angles

def OrderRobotsOnPath(path):
    orders = []
    n_robots = len(path.keys())
    orders.append([])  # Assume first step is always ok
    for t in range(1, len(path[0])):
        start_pos = [path[r][t-1] for r in range(n_robots)]
        goal_pos = [path[r][t] for r in range(n_robots)]

        # Build dependency graph
        dgraph = defaultdict(list)
        for i in range(n_robots):
            for j in range(n_robots):
                if i != j:
                    if (goal_pos[i] == start_pos[j]).all():
                        dgraph[j].append(i)
            if len(dgraph[i]) == 0:
                dgraph[i] = []

        # Order robots
        if (IsEmptyGraph(dgraph)):
            order = []
        else:
            order = kahn_topsort(dgraph)
        orders.append(order)
        print(start_pos)
        print(goal_pos)
        print(dgraph)
        print(order)
        print('.')

    path['o'] = orders
    return path

def FillPoseMsg(point):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "map"
    pose_msg.pose.position.x, pose_msg.pose.position.y = point[0], point[1]

    quat = tf.transformations.quaternion_from_euler(0, 0, point[2])
    pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = quat[0], quat[1], quat[2], quat[3]

    return pose_msg

if __name__ == "__main__":
    filepath = '../paths/decent.p'
    with open(filepath, 'rb') as f:
        path = pickle.load(f)

    n_waypoints = len(path[0])
    n_rob = len(path.keys())

    OrderRobotsOnPath(path)
    for r in range(n_rob):
        path[r] = AddAnglesToPath(path[r])

    print("num_robots:{}".format(n_rob))
    print("n_waypoints:{}", format(n_waypoints))
    for t in range(n_waypoints):
        print("Sending waypoints for step {}".format(t))

        ordering = path['o'][t]
        print(ordering)
        if len(ordering) == 0:  # Send all robots at once
            for r in range(n_rob):
                pass
                # print("Robot {}, go!".format(r))
                # pose_msg = FillPoseMsg(path[r][t])
                # self.waypoint_pubs[r].publish(pose_msg)

            # Wait for them to finish
            # self.n_robots_done = 0
            # while self.n_robots_done is not n_rob:
            #     print("Waiting for robots to finish their waypoint")
            #     update_rate.sleep()
            print("sent commands to all robots")
            # print("All robots done with step {}".format(t))
        else:
            for r in ordering:  # TODO change this
                # pose_msg = FillPoseMsg(path[r][t])
                # self.waypoint_pubs[r].publish(pose_msg)
                # self.n_robots_done = 0
                # while self.n_robots_done is not 1:
                #    print("Waiting for robots to finish their waypoint")
                #    update_rate.sleep()
                print("Robot {}, go!".format(r))
