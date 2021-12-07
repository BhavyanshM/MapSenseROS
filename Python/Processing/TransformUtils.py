import numpy as np


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