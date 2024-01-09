#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 24 11:10:07 2023

"""

import numpy as np
import cv2

from scipy.spatial import cKDTree

def project_and_display_Thibaud_V1(POINTS_MODEL_3D,COLORS_MODEL_3D,POINTS_PC,COULEURS_PC,PROJECTION_MATRIX):
    # L'idée dans cette version est de ne pas faire de projection
    # mais plutot d'utiliser le nuage de point initialement capturé par la caméra :
    # On va déterminer les nouvelles coordonées de notre objet 3D
    # et chercher les POINTS correspondant dans le nuage de la caméra.
    # On viendra alors modifier la couleur de ces POINTS

    MODEL_3D_POINTS_AFTER_ICP = np.array([(float(x), float(y), float(z)) for (
    x, y, z,t) in [PROJECTION_MATRIX @ p for p in np.column_stack((POINTS_MODEL_3D, np.ones(len(
    POINTS_MODEL_3D))))]], dtype=np.float64)
    # On cherche maintenant à superposer les deux nuages de POINTS
    # Pour cela on utilise des arbres KD
    tree = cKDTree(POINTS_PC)

    # Liste pour stocker les indices des POINTS les plus proches dans le second nuage
    indices_des_plus_proches = []

    # Pour chaque point dans le premier nuage
    for point in MODEL_3D_POINTS_AFTER_ICP:
        # On recherche le point le plus proche dans le second nuage
        distance, indice_plus_proche = tree.query(point)

        if True:  # distance < 0.003:
            # On concerve l'indice du point le plus proche
            indices_des_plus_proches.append(indice_plus_proche)

    # On modifie les COULEURS des POINTS trouvés dans l'étape précédente
    # c'est la ou se situe notre objet donc on va l'indiquer avec la couleur de l'objet en question
    i = 0

    COULEURS_PROJECTION_V1 = np.asarray([p for p in COULEURS_PC])

    for indice in indices_des_plus_proches:
        if len(COLORS_MODEL_3D) == 0:
            # Bleu (couleur dans le cas ou le modèle 3D est sans couleur)
            couleur_objet = np.array([0, 0, 255])
        else:
            couleur_objet = COLORS_MODEL_3D[i]
            i += 1
        COULEURS_PROJECTION_V1[indice] = couleur_objet
        # On fait un peu autour pour que ce soit plus visible
        COULEURS_PROJECTION_V1[indice+1] = couleur_objet
        COULEURS_PROJECTION_V1[indice-1] = couleur_objet
        COULEURS_PROJECTION_V1[indice+640] = couleur_objet
        COULEURS_PROJECTION_V1[indice+640+1] = couleur_objet
        COULEURS_PROJECTION_V1[indice+640-1] = couleur_objet
        COULEURS_PROJECTION_V1[indice-640-1] = couleur_objet
        COULEURS_PROJECTION_V1[indice-640] = couleur_objet
        COULEURS_PROJECTION_V1[indice-640+1] = couleur_objet

    return COULEURS_PROJECTION_V1

def project_and_display_Thibaud_V2(POINTS_MODEL_3D,COLORS_MODEL_3D,POINTS_PC,COULEURS_PC,TABLEAU_INDICE_PC,PROJECTION_MATRIX):
    # L'idée dans cette version est de ne pas faire de projection
    # mais plutot d'utiliser le nuage de point initialement capturé par la caméra :
    # On va déterminer les nouvelles coordonées de notre objet 3D
    # et chercher points correspondants dans le nuage de la caméra (une fois le filtrage fini).
    # On viendra alors modifier la couleur des POINTS correspondants dans l'acquisition initiale

    MODEL_3D_POINTS_AFTER_ICP = np.array([(float(x), float(y), float(z)) for (
    x, y, z,t) in [PROJECTION_MATRIX @ p for p in np.column_stack((POINTS_MODEL_3D, np.ones(len(
    POINTS_MODEL_3D))))]], dtype=np.float64)

    # On cherche maintenant à superposer les deux nuages (model_3D_POINTS sur points_full_filtres)
    # Pour cela on utilise des arbres KD
    tree = cKDTree(POINTS_PC)

    # Liste pour stocker les positions (indices) des POINTS du deuxième nuage (points_full_filtres)
    # les plus proches de ceux dans le premier nuage (model_3D_POINTS)
    # model_3D_POINTS[i]=points_full_filtres[indices_des_plus_proches[i]]
    indices_des_plus_proches = []

    # Pour chaque point dans le premier nuage (model_3D_POINTS)
    for point in MODEL_3D_POINTS_AFTER_ICP:
        # On recherche le point le plus proche dans le second nuage
        distance, indice_plus_proche = tree.query(point)

        if True:  # distance < 0.003:
            # On conserve l'indice du point le plus proche
            indices_des_plus_proches.append(indice_plus_proche)

    # On récupère la position (indices) des POINTS (points_full_filtres) dans le nuage de point initial
    # TABLEAU_INDICE_FILTRE_BRUIT est le tableau des indices des position des POINTS filtrés
    # dans le nuage de POINTS de l'acquisition initiale
    indice_dans_pc_initial = [TABLEAU_INDICE_PC[i]
                            for i in indices_des_plus_proches]

    COULEURS_PROJECTION_V2 = np.asarray([p for p in COULEURS_PC])

    # On fait les modifications de COULEURS de l'acquisition initiale
    i = 0
    for indice in indice_dans_pc_initial:
        if len(COLORS_MODEL_3D) == 0:
            # Bleu (couleur dans le cas ou le modèle 3D est sans couleur)
            couleur_objet = np.array([0, 0, 255])
        else:
            couleur_objet = COLORS_MODEL_3D[i]
            i += 1
        COULEURS_PROJECTION_V2[indice] = couleur_objet
        # On fait un peu autour pour que ce soit plus visible
        COULEURS_PROJECTION_V2[indice+1] = couleur_objet
        COULEURS_PROJECTION_V2[indice-1] = couleur_objet
        COULEURS_PROJECTION_V2[indice+640] = couleur_objet
        COULEURS_PROJECTION_V2[indice+640+1] = couleur_objet
        COULEURS_PROJECTION_V2[indice+640-1] = couleur_objet
        COULEURS_PROJECTION_V2[indice-640-1] = couleur_objet
        COULEURS_PROJECTION_V2[indice-640] = couleur_objet
        COULEURS_PROJECTION_V2[indice-640+1] = couleur_objet
        
    return COULEURS_PROJECTION_V2

def project_and_display_Tinhinane(frame, points, colors,projection):

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
            color = colors[i]
            color = (color[2], color[1], color[0])  # Conversion RGB
            cv2.circle(frame, (p[0], p[1]), 1, color, -1)

    return frame
