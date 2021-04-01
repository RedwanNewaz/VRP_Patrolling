from visibility_roadmap import VisibilityPlanner
import numpy as np
from VRP import vrproutes
import networkx as nx
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import math


np.random.seed(123)
def cost_function(i, j):
    start = roadnet[i]
    goal = roadnet[j]
    result = planner(start, goal)
    return next(iter(result.keys()))

def get_traj(path):
    dr = (np.diff(path[:, 0]) ** 2 + np.diff(path[:, 1]) ** 2) ** .5  # segment lengths
    r = np.zeros_like(path[:, 0])
    r[1:] = np.cumsum(dr)  # integrate path

    r_int = np.linspace(0, r.max(), 100)  # regular spaced path
    x_int = np.interp(r_int, r, path[:, 0])  # interpolate
    y_int = np.interp(r_int, r, path[:, 1])
    return np.array([x_int, y_int]).T


class GaussianEllipse:
    def __init__(self, mean, cov, n_std=3):
        self.mean = mean
        self.cov = cov
        self.x, self.y = mean
        eigval, eigvec = np.linalg.eig(cov)
        bigind = 0 if eigval[0] >= eigval[1] else 1
        # print(bigind)
        self.angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
        self.D = 2*np.sqrt(cov[0, 0]) * n_std
        # calculating the stdandard deviation of y ...
        self.d = 2*np.sqrt(cov[1, 1]) * n_std

        if not bigind:
            self.D, self.d = self.d, self.D


    def __call__(self, xp, yp):
        return self.pointInEllipse(self.x, self.y, xp, yp, self.d, self.D, self.angle)

    @staticmethod
    def pointInEllipse(x, y, xp, yp, d, D, angle):
        # tests if a point[xp,yp] is within
        # boundaries defined by the ellipse
        # of center[x,y], diameter d D, and tilted at angle

        cosa = math.cos(angle)
        sina = math.sin(angle)
        dd = d / 2 * d / 2
        DD = D / 2 * D / 2

        a = math.pow(cosa * (xp - x) + sina * (yp - y), 2)
        b = math.pow(sina * (xp - x) - cosa * (yp - y), 2)
        ellipse = (a / dd) + (b / DD)

        if ellipse <= 1:
            return True
        else:
            return False

    def samples(self, N):
        xx, yy = np.random.multivariate_normal(self.mean, self.cov, N).T
        return np.array((xx, yy)).T

def draw_ellipse(mean, cov, ax, n_std = 3.0, ):
    cov = np.array(cov)
    mean_x, mean_y = mean
    pearson = cov[0, 1] / np.sqrt(cov[0, 0] * cov[1, 1])
    # Using a special case to obtain the eigenvalues of this
    # two-dimensionl dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                      facecolor='white', edgecolor='red', alpha = 0.4)
    scale_x = np.sqrt(cov[0, 0]) * n_std
    # calculating the stdandard deviation of y ...
    scale_y = np.sqrt(cov[1, 1]) * n_std

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(mean_x, mean_y)

    ellipse.set_transform(transf + ax.transData)
    ax.add_patch(ellipse)
    return GaussianEllipse(mean, cov)

def main():

    xx1, yy1 = np.random.multivariate_normal(mean1, cov1, 5).T
    xx2, yy2 = np.random.multivariate_normal(mean2, cov2, 5).T
    depo = np.array([-17, 0])
    xx = [depo[0]] +xx1.tolist() + xx2.tolist()
    yy = [depo[1]] + yy1.tolist() + yy2.tolist()
    roadnet = np.array([xx, yy]).T

    result = vrproutes(10, xx, yy, cost_function)

    for i, j in result:
        start = roadnet[i]
        goal = roadnet[j]
        result = planner(start, goal)
        path = next(iter(result.values()))
        #FIXME sort path
        for t in get_traj(path):
            plt.clf()
            planner.workspace.plot()
            plt.plot(path[:, 0], path[:, 1], '--g')
            plt.scatter(t[0], t[1])
            plt.pause(0.0001)

    plt.show()

def get_measuremets(ge1, ge2, point, sensing_radius = 1):
    def collect_samples():
        if ge1(point[0], point[1]):
            # print('ge1')
            points = ge1.samples(100)
            for p in points:
                if np.linalg.norm(p - point) <= sensing_radius:
                    yield p
        elif ge2(point[0], point[1]):
            # print('ge2')
            points = ge2.samples(100)
            for p in points:
                if np.linalg.norm(p - point) <= sensing_radius:
                    yield p

    z = list(collect_samples())
    return np.array(z)



if __name__ == '__main__':
    planner = VisibilityPlanner("map.yaml")
    mean1 = [-2.5, -2.5]
    cov1 = [[8.0, 1.0], [1.0, 1.0]]
    mean2 = [-32.5, 0]
    cov2 = [[2.0, 0.3], [0.3, 5]]






    # route optimization

    xx1, yy1 = np.random.multivariate_normal(mean1, cov1, 5).T
    xx2, yy2 = np.random.multivariate_normal(mean2, cov2, 5).T
    depo = np.array([-17, 0])
    xx = [depo[0]] + xx1.tolist() + xx2.tolist()
    yy = [depo[1]] + yy1.tolist() + yy2.tolist()
    roadnet = np.array([xx, yy]).T

    result = vrproutes(10, xx, yy, cost_function)
    Z = None
    for i, j in result:
        start = roadnet[i]
        goal = roadnet[j]
        result = planner(start, goal)
        path = next(iter(result.values()))
        # FIXME sort path

        for t in get_traj(path):
            plt.clf()
            ax = plt.gca()
            ge1 = draw_ellipse(mean1, cov1, ax)
            ge2 = draw_ellipse(mean2, cov2, ax)
            planner.workspace.plot()
            z = get_measuremets(ge1, ge2, t)
            if len(z)>1 or Z is not None:
                if len(z)>1:
                    Z = z if Z is None else np.vstack((Z, z))
                ax.scatter(Z[:, 0], Z[:, 1], s=2)
            plt.plot(path[:, 0], path[:, 1], '--g')
            plt.scatter(t[0], t[1])
            plt.pause(0.0001)

    plt.show()



    # robot = np.array((2.7, -1.9))
    # z= get_measuremets(ge1, ge2, robot)
    #
    # ax.scatter(robot[0], robot[1])
    #
    # ax.scatter(z[:, 0], z[:, 1], s = 5)
    # planner.workspace.plot()
    #
    #
    #
    # # 12.5,12.53,-37.75,-12.2
    # plt.axis([-40, 12, -12, 13])
    # plt.show()





