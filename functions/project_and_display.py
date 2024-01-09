#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 24 11:10:07 2023

"""

import numpy as np
import cv2


def project_and_display(frame, points, colors,projection):

    # # Transformation des points 3D de l'objet
    ones_column = np.ones((points.shape[0], 1))
    homogeneous_points = np.hstack((points, ones_column))
    projected_points = np.dot(projection, homogeneous_points.T).T

    # Conversion des coordonnées 3D projetées en coordonnées 2D
    projected_points[:, 0] /= projected_points[:, 2]
    projected_points[:, 1] /= projected_points[:, 2]

   # Display projected points on the image with the actual colors
    for i, p in enumerate(projected_points.astype(int)):
        if 0 <= p[0] < frame.shape[1] and 0 <= p[1] < frame.shape[0]:
            color = tuple(colors[i] * 255)   # Convert color to scale 0-255
            color = (color[2], color[1], color[0])  # Conversion RGB
            cv2.circle(frame, (p[0], p[1]), 1, color, -1)

    return frame
