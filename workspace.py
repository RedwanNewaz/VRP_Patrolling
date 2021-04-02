from visibility_roadmap import VisibilityPlanner
import numpy as np
from VRP import vrproutes
import networkx as nx
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import math
from env_model import GaussianEllipse as ge

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



def draw_ellipse(mean, cov, ax, n_std = 3.0, ):
    cov = np.array(cov)
    mean_x, mean_y = mean
    pearson = cov[0, 1] / np.sqrt(cov[0, 0] * cov[1, 1])
    # Using a special case to obtain the eigenvalues of this
    # two-dimensionl dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                      facecolor='none', edgecolor='red')
    scale_x = np.sqrt(cov[0, 0]) * n_std
    # calculating the stdandard deviation of y ...
    scale_y = np.sqrt(cov[1, 1]) * n_std

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(mean_x, mean_y)

    ellipse.set_transform(transf + ax.transData)
    ax.add_patch(ellipse)


if __name__ == '__main__':
    planner = VisibilityPlanner("map.yaml")
    mean1 = np.array([-2.5, -2.5])
    cov1 =  np.array([[8.0, 1.0], [1.0, 1.0]])
    mean2 = np.array([-32.5, 0])
    cov2 =  np.array([[2.0, 0.3], [0.3, 5]])






    # route optimization

    xx1, yy1 = np.random.multivariate_normal(mean1, cov1, 5).T
    xx2, yy2 = np.random.multivariate_normal(mean2, cov2, 5).T
    # depo = np.array([-17, 0])
    depo = np.array([-15.5, -2.06])
    xx = [depo[0]] + xx1.tolist() + xx2.tolist()
    yy = [depo[1]] + yy1.tolist() + yy2.tolist()
    roadnet = np.array([xx, yy]).T

    result = vrproutes(10, xx, yy, cost_function)
    Z = None
    envs = ge(mean1, cov1)
    envs.append(mean2, cov2)
    for i, j in result:
        start = roadnet[i]
        goal = roadnet[j]
        result = planner(start, goal)
        path = next(iter(result.values()))
        # FIXME sort path

        for t in get_traj(path):
            z = envs.get_measuremets(t)
            if len(z) > 1 or Z is not None:
                if len(z) > 1:
                    Z = z if Z is None else np.vstack((Z, z))
        plt.plot(path[:, 0], path[:, 1], '--g')


    planner.workspace.plot()
    plt.savefig('results/routes.eps', format = 'eps')
    ax = plt.gca()
    draw_ellipse(mean1, cov1, ax)
    draw_ellipse(mean2, cov2, ax)
    ax.scatter(Z[:, 0], Z[:, 1], s=1)
    # plt.savefig('results/measurements.eps', format='eps')
    plt.show()





