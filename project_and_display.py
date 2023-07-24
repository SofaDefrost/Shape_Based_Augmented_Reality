#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 24 11:10:07 2023

"""

import numpy as np
import cv2

def project_and_display(frame, obj, projection, colors):
    vertices = obj.vertices
    scale_matrix = np.eye(3)   # Ajustez l'échelle selon le besoin

    # Transformation des points 3D de l'objet
    projected_points = np.dot(vertices, projection[:, :3].T) + projection[:, 3]
    projected_points = np.dot(projected_points, scale_matrix)

    # Conversion des coordonnées 3D projetées en coordonnées 2D
    projected_points[:, :2] /= projected_points[:, 2:]

    # Affichage des points projetés sur l'image avec les couleurs réelles
    for i, p in enumerate(projected_points.astype(int)):
        color = tuple(colors[i])  # Récupérer la couleur du vertex
        cv2.circle(frame, (p[0], p[1]), 1, color, -1)

    return frame