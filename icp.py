#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 23 09:08:24 2023

@author: tinhinane
"""

import numpy as np
import copy
import open3d as o3d

def draw_registration_result(source, target, transformation):
    """
    Fonction pour visualiser le résultat de l'alignement entre le nuage de points source et le nuage de points cible.

    :param source: Le nuage de points source (objet open3d.geometry.PointCloud).
    :param target: Le nuage de points cible (objet open3d.geometry.PointCloud).
    :param transformation: La matrice de transformation (4x4 numpy array) pour aligner le nuage de points source sur le cible.
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp], zoom=2, front=[0.5, -0.1, -0.1], lookat=[0, 0, 0], up=[-0.3402, -0.9189, -0.1996])

def find_nearest_neighbors(source_pc, target_pc, nearest_neigh_num):
    """
    Fonction pour trouver les points les plus proches dans le nuage de points source pour chaque point du nuage de points cible.

    :param source_pc: Le nuage de points source (objet open3d.geometry.PointCloud).
    :param target_pc: Le nuage de points cible (objet open3d.geometry.PointCloud).
    :param nearest_neigh_num: Le nombre de points les plus proches à rechercher pour chaque point cible.

    :return: Un tableau numpy contenant les points les plus proches du nuage de points source pour chaque point du nuage de points cible.
    """
    point_cloud_tree = o3d.geometry.KDTreeFlann(source_pc)
    points_arr = []
    for point in target_pc.points:
        [_, idx, _] = point_cloud_tree.search_knn_vector_3d(point, nearest_neigh_num)
        points_arr.append(source_pc.points[idx[0]])
    return np.asarray(points_arr)

def icp(source, target):
    """
    Fonction principale pour l'algorithme ICP (Iterative Closest Point).

    :param source: Le nuage de points source (objet open3d.geometry.PointCloud).
    :param target: Le nuage de points cible (objet open3d.geometry.PointCloud).

    :return: La matrice de transformation finale (4x4 numpy array) pour aligner le nuage de points source sur le cible.
    """
    # Initialisation des couleurs des nuages de points pour la visualisation.
    source.paint_uniform_color([0.5, 0.5, 0.5])
    target.paint_uniform_color([0, 0, 1])
    target_points = np.asarray(target.points)
    
    # Initialisation des variables pour le suivi des itérations et du coût.
    curr_iteration = 0
    cost_change_threshold = 0.001
    curr_cost = 1000
    prev_cost = 10000

    # Définition d'une fonction locale pour calculer le coût actuel et mettre à jour la transformation si nécessaire.
    def calculate_curr_low(transform_matrix):
        nonlocal prev_cost, curr_iteration
        if ((prev_cost - curr_cost) > cost_change_threshold):
            prev_cost = curr_cost
            target.transform(transform_matrix)
            curr_iteration += 1
        return curr_iteration

    # Première transformation : Translation initiale.
    transform_matrix = np.asarray([[0.862, 0.011, -0.507, 0.5], [-0.139, 0.967, -0.215, 0.7], [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    calculate_curr_low(transform_matrix)

    # Deuxième transformation : Rotation sur l'axe X (avec 45 degrés).
    angle = np.radians(45)
    transform_matrix_1 = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
    calculate_curr_low(transform_matrix_1)

    # Troisième transformation : Rotation sur l'axe Y (avec 45 degrés).
    angle = np.radians(45)
    transform_matrix_2 = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])
    calculate_curr_low(transform_matrix_2)

    # Quatrième transformation : Rotation sur l'axe Z (avec 45 degrés).
    angle = np.radians(45)
    transform_matrix_3 = np.asarray([[np.cos(angle), -np.sin(angle), 0, 0], [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    calculate_curr_low(transform_matrix_3)

    # Sélection de la meilleure transformation parmi les trois rotations.
    if calculate_curr_low(transform_matrix_1) < calculate_curr_low(transform_matrix_2) or calculate_curr_low(transform_matrix_1) < calculate_curr_low(transform_matrix_3):
        transform_matrix = transform_matrix_1
    elif calculate_curr_low(transform_matrix_2) < calculate_curr_low(transform_matrix_1) or calculate_curr_low(transform_matrix_2) < calculate_curr_low(transform_matrix_3):
        transform_matrix = transform_matrix_2
    else:
        transform_matrix = transform_matrix_3

    # Réinitialisation des itérations et du coût pour l'algorithme ICP principal.
    curr_iteration = 0
    cost_change_threshold = 0.001
    curr_cost = 1000
    prev_cost = 10000

    while True:
        # Trouver les points les plus proches du nuage de points source pour chaque point du nuage de points cible.
        new_source_points = find_nearest_neighbors(source, target, 1)

        # Calculer les centroïdes des nuages de points source transformé et cible.
        source_centroid = np.mean(new_source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)

        # Calculer les positions relatives des points par rapport à leurs centroïdes respectifs.
        source_repos = np.asarray([new_source_points[ind] - source_centroid for ind in range(len(new_source_points))])
        target_repos = np.asarray([target_points[ind] - target_centroid for ind in range(len(target_points))])

        # Calculer la matrice de covariance et effectuer la décomposition SVD pour obtenir la meilleure rotation.
        cov_mat = target_repos.transpose() @ source_repos
        U, X, Vt = np.linalg.svd(cov_mat)
        R = U @ Vt

        # Calculer la translation optimale.
        t = target_centroid - R @ source_centroid
        t = np.reshape(t, (1, 3))

        # Calculer le coût actuel de l'itération.
        curr_cost = np.linalg.norm(target_repos - (R @ source_repos.T).T)
        print("Curr_cost=", curr_cost)

        # Vérifier si la convergence est atteinte (différence de coût inférieure au seuil).
        if ((prev_cost - curr_cost) > cost_change_threshold):
            prev_cost = curr_cost
            # Mettre à jour la matrice de transformation avec la rotation et la translation.
            transform_matrix_target = np.hstack((R, t.T))
            transform_matrix_target = np.vstack((transform_matrix_target, np.array([0, 0, 0, 1])))
            # Appliquer la transformation au nuage de points source.
            source.transform(transform_matrix_target)
            curr_iteration += 1
        else:
            break

    print("\nIteration=", curr_iteration)
    # Visualiser le résultat final de l'alignement.
    draw_registration_result(source, target, transform_matrix)
    # Renvoyer la matrice de transformation finale.
    return transform_matrix_target ,cost #ajoute le coût dans les sorties

def multiplpe_icp(source, target):
    
    # init_target = target
    
    # Deuxième transformation : Rotation sur l'axe X (avec 45 degrés).
    angle = np.radians(45)
    transform_matrix_1 = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
    target.transform(transform_matrix)
    transform_matrix_target_1,cost_1 = icp(source,target)

    # Troisième transformation : Rotation sur l'axe Y (avec 45 degrés).
    angle = np.radians(45)
    transform_matrix_2 = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])
    target.transform(transform_matrix)
    transform_matrix_target_2,cost_2 = icp(source,target)

    # Quatrième transformation : Rotation sur l'axe Z (avec 45 degrés).
    angle = np.radians(45)
    transform_matrix_3 = np.asarray([[np.cos(angle), -np.sin(angle), 0, 0], [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    target.transform(transform_matrix)
    transform_matrix_target_3,cost_3 = icp(source,target)
    
    # ajouter comparaison
    transform_matrix_target = transform_matrix_targe_3
    
    return transform_matrix_target
    


def run_icp(source_path, target_path):
    """
    Fonction pour exécuter l'algorithme ICP avec deux chemins de fichiers de nuages de points en entrée.

    :param source_path: Le chemin du fichier du nuage de points source au format PLY.
    :param target_path: Le chemin du fichier du nuage de points cible au format PLY.

    :return: La matrice de transformation finale (4x4 numpy array) pour aligner le nuage de points source sur le cible.
    """
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)
    # Appeler la fonction ICP pour aligner les nuages de points source et cible.
    return icp(source, target)
    # return multiple_icp(source, target)

