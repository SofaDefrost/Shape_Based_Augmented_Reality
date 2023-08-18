import numpy as np
import copy


def create_rot_matrix_x(angle):
    angle = np.radians(angle)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]]) # en x
    return transform_matrix

def create_rot_matrix_y(angle):
    angle = np.radians(angle)
    transform_matrix = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]]) # matrice transformation en y
    return transform_matrix

def create_rot_matrix_z(angle):
    angle = np.radians(angle)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]]) # matrice en z
    return transform_matrix


        
