#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 17 15:52:12 2023

@author: tinhinane
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

def angles(transform_matrix):
    # Extraire la partie de rotation de la matrice (3x3)
    rotation_matrix = transform_matrix[:3, :3]

    # Convertir la matrice de rotation en objet Rotation
    rotation = R.from_matrix(rotation_matrix)

    # Obtenir les angles d'Euler (en radians) sous forme d'un vecteur (roll, pitch, yaw)
    euler_angles = rotation.as_euler('zyx', degrees=True)  # 'zyx' signifie que les rotations sont appliquées dans l'ordre ZYX
    
    

    # Extraction des angles individuels
    angle_z, angle_y, angle_x = euler_angles
    angle_zz = -angle_z
    angle_yy = -angle_y

    # Matrices de rotation individuelles
    Mx = np.array([[1, 0, 0, 0],
                   [0, np.cos(angle_x), -np.sin(angle_x), 0],
                   [0, np.sin(angle_x), np.cos(angle_x), 0],
                   [0, 0, 0, 1]])

    My = np.array([[np.cos(angle_yy), 0, np.sin(angle_yy), 0],
                   [0, 1, 0, 0],
                   [-np.sin(angle_yy), 0, np.cos(angle_yy), 0],
                   [0, 0, 0, 1]])

    Mz = np.array([[np.cos(angle_zz), -np.sin(angle_zz), 0, 0],
                   [np.sin(angle_zz), np.cos(angle_zz), 0, 0],
                   [0, 0, 0, 0],
                   [0, 0, 0, 1]])

    # Matrice de transformation finale
    matrix_1 = np.dot(Mx,My, Mz)

    return matrix_1