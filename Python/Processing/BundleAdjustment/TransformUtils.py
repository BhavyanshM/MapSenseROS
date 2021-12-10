import numpy as np
from geometry import *

def get_rotation_x(radians):
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(radians), np.sin(radians), 0],
        [0, -np.sin(radians), np.cos(radians), 0],
        [0, 0, 0, 1]
    ])


def get_rotation_y(radians):
    return np.array([
        [np.cos(radians), 0, -np.sin(radians), 0],
        [0, 1, 0, 0],
        [np.sin(radians), 0, np.cos(radians), 0],
        [0, 0, 0, 1]
    ])

def get_rotation_z(radians):
    return np.array([
        [np.cos(radians), np.sin(radians), 0, 0],
        [-np.sin(radians), np.cos(radians), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def get_translation_xyz(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1,  z],
        [0, 0, 0,  1]
    ])

# References for SE(3) and SO(3) groups and their Lie Algebra:
# Link: https://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf
# Link: https://ethaneade.com/lie.pdf

def SO3Exp(rvec):
    theta = np.linalg.norm(rvec)
    matrix = np.array([[0, -rvec[2], rvec[1]],[rvec[2], 0, -rvec[0]],[-rvec[1], rvec[0], 0]])
    return np.eye(3) + np.sin(theta)/theta * matrix + (1 - np.cos(theta))/(theta * theta) * matrix * matrix

def SO3LogUp(rotation):
    psi = np.arccos((np.trace(rotation) - 1) / 2)
    return psi * np.array(rotation - rotation.T) / (2 * np.sin(psi))

def SO3Log(rotation):
    psi = np.arccos((np.trace(rotation) - 1) / 2)
    return psi * np.array([rotation[2,1] - rotation[1,2], rotation[0,2] - rotation[2,0], rotation[1,0] - rotation[0,1]]) / (2 * np.sin(psi))

def so3_up(vec):
    return np.array([[0, -vec[2], vec[1]],[vec[2], 0, -vec[0]],[-vec[1], vec[0], 0]])

def so3_down(matrix):
    return np.array([-matrix[1,2], matrix[0,2], -matrix[0,1]])

def SE3Exp(pose):
    so3_mat = so3_up(pose[:3])
    R = SO3Exp(pose)
    theta = np.linalg.norm(pose[:3])
    V = np.eye(3) + ((1 - np.cos(theta))/(theta*theta)) * so3_mat + ((theta - np.sin(theta))/(theta*theta*theta)) * (so3_mat @ so3_mat)
    t = V @ pose[3:6]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def SE3Log(transform):
    omega = SO3Log(transform[:3,:3])
    theta = np.linalg.norm(omega)
    Omega = np.array([[0,-omega[2],omega[1]],[omega[2], 0, -omega[0]],[-omega[1], omega[0], 0]])
    V_inv = np.eye(3) - 1/2 * Omega + 1/(theta*theta) * (1 - np.sin(theta) * theta / (1 - np.cos(theta))) * (Omega @ Omega)
    t = V_inv @ transform[:3, 3]
    pose = np.array([omega[0], omega[1], omega[2], t[0], t[1], t[2]])
    return pose

