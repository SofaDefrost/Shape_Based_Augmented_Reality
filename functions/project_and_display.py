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
# import numpy as np
# import cv2

# def project_and_display(frame, obj, projection, h, w):
#     vertices = np.array(obj.vertices)  # Conversion de la liste en un tableau NumPy

#     # Transformation des points 3D de l'objet
#     ones_column = np.ones((vertices.shape[0], 1))
#     homogeneous_vertices = np.hstack((vertices, ones_column))
#     projected_points = np.dot(homogeneous_vertices, projection.T)

#     # Conversion des coordonnées 3D projetées en coordonnées 2D
#     projected_points[:, 0] /= projected_points[:, 2]
#     projected_points[:, 1] /= projected_points[:, 2]

#     # Affichage des points projetés sur l'image
#     for p in projected_points.astype(int):
#         cv2.circle(frame, (p[0], p[1]), 1, (0, 0, 255), -1)

#     return frame

import cv2
import numpy as np

def project_and_display(frame, obj, projection, h, w, scale_factor=0.000023):
    """
    Fonction pour projeter et afficher les points 3D d'un objet sur une image.

    :param frame: L'image (matrice numpy) sur laquelle projeter les points.
    :param obj: L'objet 3D contenant les coordonnées des sommets (vertices) de l'objet.
    :param projection: La matrice de projection (3x4 numpy array) pour la projection des points 3D sur l'image.
    :param h: Hauteur de l'image.
    :param w: Largeur de l'image.
    :param scale_factor: Facteur d'échelle pour ajuster l'échelle des points 3D projetés. (facultatif)

    :return: L'image avec les points 3D projetés affichés.
    """
    vertices = obj.vertices

    # Ajuster l'échelle des points 3D projetés selon le besoin
    scale_matrix = np.eye(3) * scale_factor

    # Transformation des points 3D de l'objet
    projected_points = np.dot(vertices, projection[:, :3].T) + projection[:, 3]
    projected_points = np.dot(projected_points, scale_matrix)

    # Conversion des coordonnées 3D projetées en coordonnées 2D
    projected_points[:, :2] /= projected_points[:, 2:]

    # Affichage des points projetés sur l'image
    for p in projected_points.astype(int):
        if 0 <= p[0] < w and 0 <= p[1] < h:
            cv2.circle(frame, (p[0], p[1]), 1, (255, 255, 255), -1)

    return frame



