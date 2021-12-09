import numpy as np

from TransformUtils import *

class Camera:
    def __init__(self):
        self.fx = 800
        self.fy = 800
        self.width = 800
        self.height = 600
        self.cx = 400
        self.cy = 300
        self.transform = get_translation_xyz(0,0,4) @ get_rotation_y(0.5)

    def Project(self, homo_point):
        cam_point = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]]) @ self.transform @ homo_point
        # print("CamPoint: ", cam_point)
        cam_point /= cam_point[2]
        # print("ImagePoint: ", cam_point)
        x = (cam_point[0] * self.fx) + self.cx
        y = (cam_point[1] * self.fy) + self.cy
        return np.array([x, y])