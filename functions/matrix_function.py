import numpy as np
import copy
import functions.icp as cp


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
    transform_matrix = np.asarray([[np.cos(angle), -np.sin(angle), 0, 0], [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # matrice en z
    return transform_matrix


def get_pose_matrix(source,target):
    print("Carry out the first ICP execution to obtain the best suitable initial matrix that has the lowest cost.")
    M_icp_1, cost = cp.run_icp_1(source_path = source,target_path = target,pc_after_multiple_icp = "null") 
    print("The best matrix is:", M_icp_1, "with a low cost of:",cost )
    M_icp_1_t = np.transpose(M_icp_1)
    matrix = M_icp_1_t @ create_rot_matrix_x(-90) 
    matrix # version avec matrice 1er ICP

    return matrix

