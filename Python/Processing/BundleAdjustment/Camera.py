import numpy as np

from TransformUtils import *

class Camera:
    def __init__(self):
        self.fx = 1
        self.fy = 1
        self.width = 2
        self.height = 2
        self.min_x = -1
        self.min_y = -1
        self.cx = 400
        self.cy = 300
        self.transform = np.eye(4)

    def Project(self, homo_point):
        cam_point = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]]) @ self.transform @ homo_point
        cam_point /= cam_point[2]

        # x = (cam_point[0] * self.fx) + self.cx
        # y = (cam_point[1] * self.fy) + self.cy


        x = (cam_point[0])
        y = (cam_point[1])

        return np.array([x, y])

    def SetTransform(self, transform):
        self.transform = transform