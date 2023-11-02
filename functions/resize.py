#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 21 18:08:11 2023

@author: Tinhinane and Thibaud
"""
import trimesh
import open3d as o3d
import numpy as np

def compute_max_distance(point_cloud):
    # Récupérer les coordonnées des points sous forme de tableau NumPy
    points = np.asarray(point_cloud.points)

    if len(points) < 2:
        return 0.0

    # Calculer toutes les distances entre les points en une seule opération
    pairwise_distances = np.linalg.norm(points[:, np.newaxis] - points, axis=2)

    # Ignorer les distances entre un point et lui-même (diagonale)
    np.fill_diagonal(pairwise_distances, 0.0)

    # Trouver la distance maximale
    max_distance = np.max(pairwise_distances)

    return max_distance

def Resize_pas_auto(output_file, pc_resized, scaling_factor):
    """
    Resize the point cloud mesh by applying a scaling factor to its vertices.

    :param output_file: The filename of the point cloud mesh in PLY format.
    :param pc_resized: The filename to save the resized point cloud mesh.
    :param scaling_factor: The scaling factor to resize the vertices of the mesh.
    """
    # Load the PLY file and create a Trimesh object containing the point cloud data.
    mesh = trimesh.load(output_file)
    
    # Apply scaling by multiplying the vertices' coordinates with the scaling factor.
    scaled_vertices = mesh.vertices * scaling_factor
    
    # Create a new Trimesh with the scaled vertices and original faces (if available).
    scaled_mesh = trimesh.Trimesh(scaled_vertices, faces=mesh.faces if hasattr(mesh, 'faces') else None)
    
    # Export the resized mesh to the specified file.
    scaled_mesh.export(pc_resized)


def resize_auto(pc_filtred_name, model_3D_name, point_cloud_resizing):
    """
    Cette fonction redimensionne un nuage de points 3D à partir de deux fichiers au format PLY,
    en utilisant le rapport des distances maximales entre les points des deux nuages de points.

    :param pc_filtred_name: Le nom du fichier du nuage de points filtré après l'application du masque au format PLY.
    :param model_3D_name: Le nom du fichier du modèle 3D au format PLY.
    :param point_cloud_resizing: Le nom du fichier dans lequel le nouveau nuage de points redimensionné
                                 sera enregistré au format PLY.
    """
    # Charger les nuages de points filtré et modèle 3D
    pc_filtred = o3d.io.read_point_cloud(pc_filtred_name)
    model_3D = o3d.io.read_point_cloud(model_3D_name)
    
    # Calculer les distances maximales des deux nuages de points
    max_dist_pc_filtred = compute_max_distance(pc_filtred)
    max_dist_model_3D = compute_max_distance(model_3D)

    # Calculer le facteur de redimensionnement basé sur les distances maximales
    scaling_factor = max_dist_pc_filtred / max_dist_model_3D
    Resize_pas_auto(model_3D_name,point_cloud_resizing,scaling_factor)


