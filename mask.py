#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 21 17:15:54 2023

@author: tinhinane
"""
import open3d as o3d
import numpy as np

def mask(point_cloud_name, filtered_cloud_name, threshold, color_phase):
    """
    Cette fonction permet de filtrer un nuage de points en fonction d'une valeur seuil
    appliquée sur les couleurs des points ou les coordonnées x, y, z.

    :param point_cloud: Le nom du fichier du nuage de points généré par la fonction acquisition.
    :param output_file: Le nom du fichier dans lequel sera enregistré le nuage de points
                        après l'application du masque.
    :param threshold: Le seuil pour filtrer les points selon la couleur ou les coordonnées.
    :param color_phase: Sélectionner quelle phase des couleurs conserver,
                        soit dans le rouge (intervalle [0-1]) et dans la colonne 0.
    """
    cloud = o3d.io.read_point_cloud(point_cloud_name)
    points = np.asarray(cloud.points)
    colors = np.asarray(cloud.colors)
    
    # Indices des canaux de couleur (rouge=0, vert=1, bleu=2)
    r = 0
    g = 1
    b = 2

    if color_phase == 'rouge':
        mask = colors[:, r] < threshold
    elif color_phase == 'vert':
        mask = colors[:, g] < threshold
    elif color_phase == 'bleu':
        mask = colors[:, b] < threshold
    else:
        raise ValueError("La phase de couleur sélectionnée n'est pas valide.")

    # Filtrage des points et des couleurs
    points = points[mask]
    colors = colors[mask]

    # Création d'un nouveau nuage de points avec les points filtrés
    filtered_cloud = o3d.geometry.PointCloud()
    filtered_cloud.points = o3d.utility.Vector3dVector(points)
    filtered_cloud.colors = o3d.utility.Vector3dVector(colors)

    # Sauvegarde du nuage de points filtré
    o3d.io.write_point_cloud(filtered_cloud_name, filtered_cloud)

    