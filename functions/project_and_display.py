#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 24 11:10:07 2023

"""

import numpy as np
import cv2



def project_and_display(frame, obj, projection, colors):
    """
  Function to project a 3D object onto an image with the real colors of the object.

  :param frame: The image frame onto which the 3D object is projected.
  :param obj: The 3D object to be projected.
  :param projection: The projection matrix for transforming 3D points to 2D image plane.
   :param colors: The actual colors corresponding to the vertices of the 3D object.
   :return: The input frame with the projected points displayed using the real colors.
"""
    vertices = np.array(obj.vertices)  # Conversion de la liste en un tableau NumPy
    # # Transformation des points 3D de l'objet
    ones_column = np.ones((vertices.shape[0], 1))
    homogeneous_vertices = np.hstack((vertices, ones_column))
    projected_points = np.dot(homogeneous_vertices, projection.T)

    # Conversion des coordonnées 3D projetées en coordonnées 2D
    projected_points[:, 0] /= projected_points[:, 2]
    projected_points[:, 1] /= projected_points[:, 2]

   # Display projected points on the image with the actual colors
    for i, p in enumerate(projected_points.astype(int)):
        if 0 <= p[0] < frame.shape[1] and 0 <= p[1] < frame.shape[0]:
            color = tuple(colors[i] * 255)   # Convert color to scale 0-255
            cv2.circle(frame, (p[0], p[1]), 1, color, -1)

    return frame