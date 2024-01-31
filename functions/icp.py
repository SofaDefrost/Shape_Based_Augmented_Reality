import numpy as np

from typing import Tuple
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

from functions import matrix_operations as tf
from Python_3D_Toolbox_for_Realsense.functions import processing_point_cloud as pc


def find_transform_matrix_to_align_points_using_icp(source_points, target_points):
    """
    Find the transformation matrix to align source points to target points using the ICP (Iterative Closest Point) algorithm.

    Args:
    - source_points (np.ndarray): Source point cloud.
    - target_points (np.ndarray): Target point cloud.

    Returns:
    - Tuple[np.ndarray, float]: Tuple containing the cumulative transformation matrix and the final cost of alignment.
    """
    transform_matrix = np.identity(4)

    source_temp = source_points

    source_temp_shaped = np.column_stack((source_temp, np.ones(len(
        source_temp))))

    source_points = np.array([(float(x), float(y), float(z)) for (
        x, y, z, t) in [transform_matrix @ p for p in source_temp_shaped]], dtype=np.float64)

    curr_iteration = 0
    cost_change_threshold = 0.001
    curr_cost = 1000
    prev_cost = 10000
    transform_matrix_cumulee = np.identity(4)
    while True:
        new_source_points, _ = pc.find_nearest_neighbors_between_pc(
            source_temp, target_points, 1)

        source_centroid = np.mean(new_source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)
        source_repos = np.asarray(
            [new_source_points[ind] - source_centroid for ind in range(len(new_source_points))])
        target_repos = np.asarray(
            [target_points[ind] - target_centroid for ind in range(len(target_points))])

        cov_mat = target_repos.transpose() @ source_repos
        U, X, Vt = np.linalg.svd(cov_mat)
        R = U @ Vt
        t = target_centroid - R @ source_centroid
        t = np.reshape(t, (1, 3))
        curr_cost = np.linalg.norm(target_repos - (R @ source_repos.T).T)

        rotation_matrix = R[:3, :3]
        rotation = Rot.from_matrix(rotation_matrix)

        if prev_cost - curr_cost > cost_change_threshold:
            prev_cost = curr_cost
            transform_matrix = np.hstack((R, t.T))
            transform_matrix = np.vstack(
                (transform_matrix, np.array([0, 0, 0, 1])))
            source_temp_shaped = np.column_stack(
                (source_temp, np.ones(len(source_temp))))
            source_points = np.array([(float(x), float(y), float(z)) for (x, y, z, t) in [
                                     transform_matrix @ p for p in source_temp_shaped]], dtype=np.float64)
            curr_iteration += 1
            transform_matrix_cumulee = transform_matrix_cumulee @ transform_matrix
        else:
            transform_matrix = np.hstack((R, t.T))
            transform_matrix = np.vstack(
                (transform_matrix, np.array([0, 0, 0, 1])))
            source_temp_shaped = np.column_stack(
                (source_temp, np.ones(len(source_temp))))
            source_points = np.array([(float(x), float(y), float(z)) for (x, y, z, t) in [
                                     transform_matrix @ p for p in source_temp_shaped]], dtype=np.float64)
            break

    return transform_matrix_cumulee, curr_cost


def get_cost_to_align_points(points_source: np.ndarray, points_target: np.ndarray, angle_x: float, angle_y: float, angle_z: float) -> Tuple[np.ndarray, float]:
    """
    Get the cost and transformation matrix to align a source set of points to a target set of points.

    Args:
        points_source (np.ndarray): Source set of 3D coordinates.
        points_target (np.ndarray): Target set of 3D coordinates.
        angle_x (float): Rotation angle around the x-axis in degrees.
        angle_y (float): Rotation angle around the y-axis in degrees.
        angle_z (float): Rotation angle around the z-axis in degrees.

    Returns:
        Tuple[np.ndarray, float]: Transformation matrix and alignment cost.
    """
    M_x = tf.rotation_matrix_x(np.radians(angle_x))
    M_y = tf.rotation_matrix_y(np.radians(angle_y))
    M_z = tf.rotation_matrix_z(np.radians(angle_z))
    transform_matrix = M_x @ M_y @ M_z

    source_rotated = np.array([np.dot(point, transform_matrix)
                              for point in points_source])

    source_kdtree = cKDTree(source_rotated)

    # Find nearest neighbors for each point in the target point cloud
    distances, _ = source_kdtree.query(points_target, k=1)
    cost = np.mean(distances)
    return transform_matrix, cost


def find_the_best_pre_rotation_to_align_points(points_source: np.ndarray, points_target: np.ndarray, range_angle_x=[-180, 180, 10], range_angle_y=[-180, 180, 10], range_angle_z=[-180, 180, 10]) -> np.ndarray:
    """
    Find the best pre-rotation matrix to align a source set of points to a target set of points.

    Args:
    - points_source (np.ndarray): Source set of 3D coordinates.
    - points_target (np.ndarray): Target set of 3D coordinates.
    - range_angle_x (list, optional): Range of angles for rotation around the X-axis (default: [-180, 180, 10]).
    - range_angle_y (list, optional): Range of angles for rotation around the Y-axis (default: [-180, 180, 10]).
    - range_angle_z (list, optional): Range of angles for rotation around the Z-axis (default: [-180, 180, 10]).

    Returns:
    - np.ndarray: Best pre-rotation matrix.
    """
    # Initialize the best cost.
    best_cost = np.inf
    best_angle = [0, 0, 0]
    for angle_x in range(range_angle_x[0], range_angle_x[1]+1, range_angle_x[2]):
        for angle_y in range(range_angle_y[0], range_angle_y[1]+1, range_angle_y[2]):
            for angle_z in range(range_angle_z[0], range_angle_z[1]+1, range_angle_z[2]):
                transform_matrix, cost = get_cost_to_align_points(
                    points_source, points_target, angle_x, angle_y, angle_z)
                if cost < best_cost:
                    best_transform_matrix = transform_matrix
                    best_cost = cost
                    best_angle = [angle_x, angle_y, angle_z]
    return best_transform_matrix, best_angle
