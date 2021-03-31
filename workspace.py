from visibility_roadmap import VisibilityPlanner
import numpy as np
from VRP import vrproutes
import networkx as nx
import matplotlib.pyplot as plt

def cost_function(i, j):
    start = roadnet[i]
    goal = roadnet[j]
    result = planner(start, goal)
    return next(iter(result.keys()))


def find_loop_path(result):
    G = nx.DiGraph()
    for i, j in result:
        G.add_edge(i, j)
    for loop, cycle in enumerate(nx.simple_cycles(G)):
        print(loop, cycle)
        yield cycle

if __name__ == '__main__':
    # start, goal = [-2.0, -2.0], [-32.0, 1]
    planner = VisibilityPlanner("map.yaml")
    # result = planner(start, goal)
    # print(result)
    mean1 = [-2.5, -2.5]
    cov1 = [[8.0, 1.0], [1.0, 1.0]]
    mean2 = [-32.5, 0]
    cov2 = [[2.0, 0.3], [0.3, 5]]

    xx1, yy1 = np.random.multivariate_normal(mean1, cov1, 5).T
    xx2, yy2 = np.random.multivariate_normal(mean2, cov2, 5).T
    depo = np.array([-17, 0])
    xx = [depo[0]] +xx1.tolist() + xx2.tolist()
    yy = [depo[1]] + yy1.tolist() + yy2.tolist()
    roadnet = np.array([xx, yy]).T

    result = vrproutes(10, xx, yy, cost_function)
    print(result)
    planner.workspace.plot()
    for i, j in result:
        start = roadnet[i]
        goal = roadnet[j]
        result = planner(start, goal)
        path = next(iter(result.values()))
        plt.plot(path[:, 0], path[:, 1],'--g')


        plt.plot([xx[i], xx[j]], [yy[i], yy[j]], '--k')
        # plt.pause(2)
    # for round in find_loop_path(result):
    #     N = len(round)
    #     for i in range(N):
    #         j = (i+1)%N
    #         start = roadnet[i]
    #         goal = roadnet[j]
    #         result = planner(start, goal)
    #         path = next(iter(result.values()))
    #         plt.plot(path[:, 0], path[:, 1])
    plt.show()




