import yaml
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import numpy as np

class RectangleObstacle:
    def __init__(self, origin, width, height, offset):
        self.origin = origin
        self.width = width
        self.height = height
        self.offset = offset
        self.center = ((origin[0]+width/2), (origin[1]+height/2))

    def plot(self):
        self.offset = 0
        xy = ((self.origin[0]-self.offset), (self.origin[1]-self.offset))
        width = self.width + self.offset
        height = self.height + self.offset
        color = 'gray'
        return Rectangle(xy, width=width, height=height, alpha = 0.7, fc=color,ec=color,)
    def coords(self, addOffset=True):
        offset = 0
        if addOffset:
            offset = self.offset
        xy = ((self.origin[0]-offset), (self.origin[1]-offset))
        w = self.width + offset
        h = self.height + offset
        return [xy, (xy[0]+w, xy[1]), (xy[0]+w, xy[1]+h), (xy[0], xy[1]+h)]
class Obstacles:
    def __init__(self, name):
        with open(name) as file:
            self.env = yaml.load(file, Loader=yaml.FullLoader)
        self.rect_obstacles = []
        for i in range(1, 17):
            origin = tuple(self.env["Containers"][i])
            w = self.env["Containers"]["Width"]
            h = self.env["Containers"]["Height"]
            self.rect_obstacles.append(RectangleObstacle(origin, w, h, 1))
        #add big office
        w = self.env["Office"]["Width"]
        h = self.env["Office"]["Height"]
        # print(w, h)
        x, y = self.env["Office"][1][2], self.env["Office"][1][3]
        # print(x,y, w, h)
        self.rect_obstacles.append(RectangleObstacle((x, y), w, h, 1))
        for i in range(1, 5):
            origin = (self.env["Pillar"][i][0], self.env["Pillar"][i][1])
            self.rect_obstacles.append(RectangleObstacle(origin, 1, 1, 1) )

    def plot(self):
        ax = plt.gca()
        for rect in self.rect_obstacles:
            ax.add_patch(rect.plot())

    def tolist(self):
        return [item .coords(addOffset=False) for item in self.rect_obstacles]

    def getPolygon(self):
        # creating polygons using Polygon()
        for item in self.rect_obstacles:
            p1, p2, p3, p4 = map(Point,item.coords(addOffset=False))
            yield Polygon(p1, p2, p3, p4)

