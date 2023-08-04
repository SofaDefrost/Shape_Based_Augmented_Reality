#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 24 11:10:07 2023

"""

import numpy as np
import cv2

# def project_and_display(frame, obj, projection, colors):
#     vertices = obj.vertices
#     scale_matrix = np.eye(3)*100   # Ajustez l'échelle selon le besoin

#     # Transformation des points 3D de l'objet
#     projected_points = np.dot(vertices, projection[:, :3].T) + projection[:, 3]
#     projected_points = np.dot(projected_points, scale_matrix)

#     # Conversion des coordonnées 3D projetées en coordonnées 2D
#     projected_points[:, :2] /= projected_points[:, 2:]

#     # Affichage des points projetés sur l'image avec les couleurs réelles
#     for i, p in enumerate(projected_points.astype(int)):
#         color = tuple(colors[i])  # Récupérer la couleur du vertex
#         cv2.circle(frame, (p[0], p[1]), 1, color, -1)

#     return frame
import numpy as np
import cv2

def project_and_display(frame, obj, projection, h, w):
    vertices = np.array(obj.vertices)  # Conversion de la liste en un tableau NumPy

    # Transformation des points 3D de l'objet
    ones_column = np.ones((vertices.shape[0], 1))
    homogeneous_vertices = np.hstack((vertices, ones_column))
    projected_points = np.dot(homogeneous_vertices, projection.T)

    # Conversion des coordonnées 3D projetées en coordonnées 2D
    projected_points[:, 0] /= projected_points[:, 2]
    projected_points[:, 1] /= projected_points[:, 2]

    # Affichage des points projetés sur l'image
    for p in projected_points.astype(int):
        cv2.circle(frame, (p[0], p[1]), 1, (0, 0, 255), -1)

    return frame




