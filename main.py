from EnvModel import Workspace
from rrt import RRT
from VRP import vrproutes
import json
import math
import numpy as np
import matplotlib.pyplot as plt

paths = {}

def check_collision(node, obstacleList):
    if node is None:
        return False
    for x, y in zip(node.path_x,node.path_y):
        if not workspace(x, y):
            return False
    return True  # safe

def get_path_length(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d
    return le

def path_cost(i, j):
    global paths
    rrt = RRT(
        start=w[i].tolist(),
        goal=w[j].tolist(),
        rand_area=[-40, 13],
        obstacle_list=[],
        check_collision=check_collision
    )
    path = rrt.planning(animation=False)
    paths[(i, j)] = np.array(path)
    cost = 1000
    if path:
        cost = get_path_length(path)
    return cost

def solve():
    result = vrproutes(10, w[:, 0], w[:, 1], path_cost)

    # display and save results
    data = {}
    for i, j in result:
        path = paths[(i, j)]
        data[i] = {j: path.tolist()}
        plt.plot(path[:, 0], path[:, 1], c='g')

    with open("result.json", "w") as file:
        json.dump(data, file, indent=4)


if __name__ == '__main__':
    workspace = Workspace("map.yaml")
    print (workspace(-18,0))
    workspace.plot()
    workspace.gen_samples()

    # generate vehicle routes
    all_routes = workspace.gen_samples()
    depo = np.array([-17, 0])
    plt.scatter(depo[0], depo[1], color='g')
    w = np.vstack((depo, all_routes[0]))
    solve()



    plt.axis([-37.8, 12.25, -12.25, 11.25])
    plt.show()
