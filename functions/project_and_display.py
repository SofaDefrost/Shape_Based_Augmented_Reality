#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 24 11:10:07 2023

"""

import numpy as np
import cv2

from scipy.spatial import cKDTree

from typing import Tuple

def find_nearest_neighbors(source_points: np.ndarray, target_points: np.ndarray, nearest_neigh_num: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    Find the nearest neighbors in the source point cloud for each point in the target point cloud.

    Args:
    - source_points (np.ndarray): Source point cloud.
    - target_points (np.ndarray): Target point cloud.
    - nearest_neigh_num (int): Number of nearest neighbors to find.

    Returns:
    - Tuple[np.ndarray, np.ndarray]: Nearest neighbor points and their indices in the source point cloud.
    """
    source_points=np.array([np.array([point[0],point[1],point[2]]) for point in source_points])

    # Create a KD-Tree from the source points
    source_kdtree = cKDTree(source_points)

    # Find nearest neighbors for each point in the target point cloud
    distances, indices = source_kdtree.query(target_points, k=nearest_neigh_num)

    # Retrieve the nearest neighbor points
    nearest_neighbors = source_points[indices]
    
    return nearest_neighbors, np.array(indices)


def project_3D_model_on_pc_using_closest_points_identification(points_model_3D: np.ndarray,colors_model_3D: np.ndarray, points_pc: np.ndarray,colors_pc: np.ndarray,projection_matrix: np.ndarray) -> np.ndarray:
    """
    Project a 3D model onto a point cloud using identification of closest points.

    Parameters:
    - points_model_3D (np.ndarray): 3D points of the model.
    - colors_model_3D (np.ndarray): Colors associated with the 3D model points.
    - points_pc (np.ndarray): 3D points of the point cloud.
    - colors_pc (np.ndarray): Colors associated with the point cloud points.
    - projection_matrix (np.ndarray): Projection matrix for transforming 3D model points.

    Returns:
    - np.ndarray: Updated colors of the point cloud after projecting the 3D model onto it.
    """
    # Transform 3D model points using the projection matrix
    model_3D_points_after_projection = np.array([(float(x), float(y), float(z)) for (
    x, y, z,t) in [projection_matrix @ p for p in np.column_stack((points_model_3D, np.ones(len(
    points_model_3D))))]], dtype=np.float64)
    
    # Find the indices of the closest points in the point cloud for each transformed model point
    _,indices_of_closest_points=find_nearest_neighbors(points_pc,model_3D_points_after_projection,1)

    # Update the colors of the identified points in the point cloud with the colors of the 3D model
    colors_projection = np.asarray([p for p in colors_pc])
    
    i = 0
    for indice in indices_of_closest_points:
        if len(colors_model_3D) == 0:
            # Blue (color in case the 3D model is without color)
            couleur_objet = np.array([0, 0, 255])
        else:
            couleur_objet = colors_model_3D[i]
            i += 1

        # Update the color of the identified point
        colors_projection[indice] = couleur_objet

        # Make the color change more visible in the surrounding points
        for offset in [-1, 0, 1, -640, -640 - 1, -640 + 1, 640, 640 - 1, 640 + 1]:
            colors_projection[indice + offset] = couleur_objet

    return colors_projection


def project_3D_model_on_pc_using_closest_points_and_indices(points_model_3D: np.ndarray,colors_model_3D: np.ndarray,points_pc: np.ndarray,colors_pc: np.ndarray,table_of_indice_pc: np.ndarray,projection_matrix: np.ndarray) -> np.ndarray:
    """
    Project a 3D model onto a point cloud using identification of closest points and indices.

    Parameters:
    - points_model_3D (np.ndarray): 3D points of the model.
    - colors_model_3D (np.ndarray): Colors associated with the 3D model points.
    - points_pc (np.ndarray): 3D points of the point cloud.
    - colors_pc (np.ndarray): Colors associated with the point cloud points.
    - table_of_indice_pc (np.ndarray): Array of indices corresponding to the positions of points
      in the initial point cloud.
    - projection_matrix (np.ndarray): Projection matrix for transforming 3D model points.

    Returns:
    - np.ndarray: Updated colors of the initial point cloud after projecting the 3D model onto it.
    """
    # Transform 3D model points using the projection matrix
    model_3d_points_after_projections = np.array(
        [(float(x), float(y), float(z)) for (x, y, z, t) in [
            projection_matrix @ p for p in np.column_stack(
                (points_model_3D, np.ones(len(points_model_3D))))
        ]],
        dtype=np.float64
    )

    # Find the indices of the closest points in the point cloud for each transformed model point
    _,indices_of_closest_points = find_nearest_neighbors(
        points_pc, model_3d_points_after_projections[:, :3],1)

    # Retrieve the indices of the points in the initial point cloud
    indices_in_initial_pc = [table_of_indice_pc[i]
                             for i in indices_of_closest_points]

    colors_projection = np.asarray([p for p in colors_pc])

    # Update the colors of the initial point cloud with the colors of the 3D model
    i = 0
    for indice in indices_in_initial_pc:
        if len(colors_model_3D) == 0:
            # Blue (color in case the 3D model is without color)
            couleur_objet = np.array([0, 0, 255])
        else:
            couleur_objet = colors_model_3D[i]
            i += 1

        # Update the color of the identified point
        colors_projection[indice] = couleur_objet

        # Make the color change more visible in the surrounding points
        for offset in [-1, 0, 1, -640, -640 - 1, -640 + 1, 640, 640 - 1, 640 + 1]:
            colors_projection[indice + offset] = couleur_objet

    return colors_projection

def project_3D_model_on_pc(frame: np.ndarray, points: np.ndarray, colors: np.ndarray, projection_matrix: np.ndarray) -> np.ndarray:
    """
    Project a 3D model onto a 2D image frame using a projection matrix.

    Parameters:
    - frame (np.ndarray): Image frame.
    - points (np.ndarray): 3D points of the model.
    - colors (np.ndarray): Colors associated with the 3D model points.
    - projection_matrix (np.ndarray): Projection matrix for transforming 3D model points.

    Returns:
    - np.ndarray: Image frame with the 3D model projected onto it.
    """
    # Transformation of 3D model points
    ones_column = np.ones((points.shape[0], 1))
    homogeneous_points = np.hstack((points, ones_column))
    projected_points = np.dot(projection_matrix, homogeneous_points.T).T

    # Conversion of projected 3D coordinates to 2D coordinates
    projected_points[:, 0] /= projected_points[:, 2]
    projected_points[:, 1] /= projected_points[:, 2]

    # Display projected points on the image with the actual colors
    for i, p in enumerate(projected_points.astype(int)):
        if 0 <= p[0] < frame.shape[1] and 0 <= p[1] < frame.shape[0]:
            color = colors[i]
            color = (int(color[2]), int(color[1]), int(color[0]))  # Convert RGB
            cv2.circle(frame, (p[0], p[1]), 1, color, -1)

    return frame
