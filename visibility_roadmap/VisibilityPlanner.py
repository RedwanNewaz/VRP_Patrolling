import numpy as np
from collections import defaultdict
from sklearn.neighbors import KDTree
import networkx as nx
from .MapReader import Obstacles
from .visibility_road_map import VisibilityRoadMap, ObstaclePolygon



class VisibilityPlanner:
    def __init__(self, mapname):
        self.workspace = Obstacles(mapname)
        self.__construct_roadmap(self.workspace)

    def __call__(self, *args, **kwargs):
        assert len(args) == 2 # should contain start and goal location
        assert len(args[0]) == 2 # start location should be 2d
        assert len(args[1]) == 2
        node_path = self.gen_path(args[0], args[1], self.kdt, self.vis_rm.nodes, self.G)
        final_path = [args[0]] + [[p.x, p.y] for n in node_path.values() for p in n] + [args[1]]
        return {next(iter(node_path.keys())):np.array(final_path)}
    @staticmethod
    def gen_path(start, goal, kdt, nodes, graph):
        loc1 = np.array([start])
        loc2 = np.array([goal])
        cost1, neigh1 = kdt.query(loc1, k=2, return_distance=True)
        cost2, neigh2 = kdt.query(loc2, k=2, return_distance=True)

        s = nodes[neigh1[-1][0]]
        g = nodes[neigh2[-1][0]]
        cs = cost1[-1][0]
        cg = cost2[-1][0]
        path = nx.shortest_path(graph, s, g)
        N = len(path) - 1
        dist = cs + cg
        for i in range(N):
            x = np.array((path[i].x, path[i].y))
            y = np.array((path[i + 1].x, path[i + 1].y))
            dist += np.linalg.norm(x - y)
        return {dist: path}
    def __construct_roadmap(self, workspace):

        # start and goal position
        sx, sy = 9.75, -0.10  # [m]
        gx, gy = -14.3, -8.28  # [m]
        robot_radius = 2.0  # [m]

        # generate obstacles
        boxes = workspace.tolist()
        obstacles = [ObstaclePolygon([y[0] for y in box], [y[1] for y in box]) for box in boxes]

        vis_rm = VisibilityRoadMap(robot_radius, do_plot=False)
        rx, ry = vis_rm.planning(sx, sy, gx, gy, obstacles) # populate the road_map_info and nodes

        # construct kd tree and networkx graph
        X = None
        self.G = nx.Graph()
        for i, node in enumerate(vis_rm.nodes):
            npxy = np.array([node.x, node.y])
            X = npxy if X is None else np.vstack((X, npxy))
            for index in vis_rm.road_map_info[i]:# make sure node class has __hash__ definition
                self.G.add_edge(node, vis_rm.nodes[index])
        self.kdt = KDTree(X, leaf_size=30, metric='euclidean')
        self.vis_rm = vis_rm
