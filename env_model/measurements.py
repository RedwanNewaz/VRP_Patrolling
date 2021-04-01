import numpy as np
import math

class GaussianEllipse:
    __others = []
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

    def append(self, mean, cov, n_std=3):
        self.__others.append(GaussianEllipse(mean, cov, n_std))


    def get_measuremets(self, point, sensing_radius = 1, num_samples = 100):
        assert isinstance(point, np.ndarray)
        def collect_samples(model):
            if model(point[0], point[1]):
                points = model.samples(num_samples)
                for p in points:
                    if np.linalg.norm(p - point) <= sensing_radius:
                        yield p
        models = self.__others + [self]
        Z = [p for m in models for p in collect_samples(m) ]
        return np.array(Z)
