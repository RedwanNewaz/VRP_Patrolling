import math
import numpy as np
import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from scipy.stats import multivariate_normal
from rrt import RRT
from VRP import vrproutes
import json

np.random.seed(100)
class Workspace:
    def __init__(self,name):
        with open(name) as file:
            self.env = yaml.load(file, Loader=yaml.FullLoader)
        self.rectangles = []
        for i in range(1, 17):
            center = self.env["Containers"][i]
            w = self.env["Containers"]["Width"]
            h = self.env["Containers"]["Height"]
            self.rectangles.append([center[0],center[1], center[0]+w,center[1]+h])
        w = self.env["Office"]["Width"]
        h = self.env["Office"]["Height"]
        x, y = self.env["Office"][1][2], self.env["Office"][1][3]
        self.rectangles.append([x,y, x+w, y+h])
        self.circles=[]
        for i in range(1, 5):
            self.circles.append([self.env["Pillar"][i][0], self.env["Pillar"][i][1]])
        self.radius=self.env["Pillar"]["Radius"]
        self.mu1 = self.env['Pdf'][1]['mean']
        self.mu2 = self.env['Pdf'][2]['mean']
        self.cov1 = self.env['Pdf'][1]['cov']
        self.cov2 = self.env['Pdf'][2]['cov']

    @staticmethod
    def get_samples(mu, cov, num_samples = 50):
        x, y = np.random.multivariate_normal(mu, cov, num_samples).T
        return np.array([x, y]).T

    @staticmethod
    def patrolling_locations(X, num_routes, result, count=0):
        row, col = X.shape
        if row < num_routes:
            return
        indexes = np.arange(row)
        samples = np.random.choice(indexes, num_routes)
        # print(count, row, X[samples].shape)
        result[count] = X[samples]
        mask = np.ones(row, dtype=bool)
        mask[samples] = False
        data = X[mask]
        Workspace.patrolling_locations(data, num_routes, result, count + 1)

    def __call__(self, x,y):
        p = np.array([x,y])
        for circle in self.circles:
            if np.linalg.norm(np.array(circle)-p)<self.radius:
                return False
        for box in self.rectangles:
            if p[0]>=box[0] and p[1]>=box[1] and p[0]<=box[2] and p[1]<=box[3]:
                return False
        # -37.8, 12.25, -12.25, 11.25
        if p[0] > -37.8 and p[1] > -12.25 and p[0] < 12.25 and p[1] < 11.25:
            return True
        else:
            return False
        # return True

    def plot(self):
        def draw_rect(rect, color='gray', alpha = 0.7):
            w = rect[2] - rect[0]
            h = rect[3] - rect[1]
            center = tuple(rect[:2])
            ax.add_patch(Rectangle(center,
                                   w, h,
                                   fc=color,
                                   ec=color,
                                   lw=1, alpha=alpha))
        # rectangles boxes
        fig, ax = plt.subplots()
        for rect in self.rectangles[:-1]:
            draw_rect(rect, 'green')
        draw_rect(self.rectangles[-1], 'gray')

        for circle in self.circles:
            ax.add_patch(Circle(tuple(circle), self.radius))

        # multivariate gaussian plots
        x, y = np.mgrid[-37.8:12.25:1, -12.25:12.25:1]
        pos = np.dstack((x, y))

        rv  = multivariate_normal(self.mu1, self.cov1)
        rv1 = multivariate_normal(self.mu2, self.cov2)
        ax.contourf(x, y, rv.pdf(pos)+rv1.pdf(pos))

        # samples
        X1 = self.get_samples(mu=self.mu1, cov=self.cov1, num_samples=50)
        X2 = self.get_samples(mu=self.mu2, cov=self.cov2, num_samples=50)
        X = np.vstack((X1, X2))
        # ax.scatter(X[:,0], X[:, 1], color='r', s= 1 )

        # example route
        result = {}
        self.patrolling_locations(X, 10, result)
        X = result[0]
        ax.scatter(X[:, 0], X[:, 1], color='r', s=1)
        # print(result.keys())

        #depo = np.array([0, 0])
        #patrolnodes = np.vstack((depo, result[0]))
        #print(len(patrolnodes))
        #vrproutes(10, patrolnodes[:, 0], patrolnodes[:, 1])
        self.route = X


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

if __name__ == '__main__':
    workspace = Workspace("map.yaml")
    print (workspace(-18,0))
    workspace.plot()


    p1 = workspace.route[-1, :]
    p2 = workspace.route[-6, :]

    p = np.vstack((p1, p2))
    plt.scatter(p[:,0], p[:, 1], color = 'r')



    # ====Search Path with RRT====
    # Set Initial parameters

    n = 10
    N = [i for i in range(1, n + 1)]
    V = [0] + N
    cost ={}
    A = [(i, j) for i in V for j in V if i != j]


    depo = np.array([-17, 0])
    plt.scatter(depo[0], depo[1], color = 'g')
    w = np.vstack((depo, workspace.route))
    print(w.shape)

    paths={}
    for i, j in A:
        rrt = RRT(
            start=w[i].tolist(),
            goal=w[j].tolist(),
            rand_area=[-40, 13],
            obstacle_list=[],
            check_collision=check_collision
        )
        path = rrt.planning(animation=False)
        paths[(i,j)] = np.array(path)
        if not path:
            cost[(i,j)] = 1000
        else:
            cost[(i,j)] = get_path_length(path)
    #print (cost)
    result=vrproutes(10, w[:, 0], w[:, 1], cost)
    #print(result)
    data={}
    for i, j in result:
        path = paths[(i,j)]
        data[i]={j: path.tolist()}
        #path = np.vstack(depo,path, depo)
        #print(len(path))
        plt.plot(path[:,0], path[:,1], c='g')
    plt.axis([-37.8, 12.25, -12.25, 11.25])
    with open ("result.json", "w") as file:
        json.dump(data, file, indent=4)

    plt.show()

