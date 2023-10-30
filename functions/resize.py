#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 21 18:08:11 2023

@author: tinhinane
"""
import trimesh
import open3d as o3d
import numpy as np
import math

def distance_max(model):
    """
This function calculates the maximum distance between points in a point cloud.

:param model: The filename of the point cloud in PLY format.
:return: The maximum distance between points in the point cloud.
  
 """

    cloud = o3d.io.read_point_cloud(model)
    points = np.asarray(cloud.points)

    max_distance = 0.0

    for i in range(len(points)):
        for j in range(len(points)):
            dx = points[j][0] - points[i][0]
            dy = points[j][1] - points[i][1]
            dz = points[j][2] - points[i][2]
            dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            if dist > max_distance:
                max_distance = dist

    print("La distance maximale du nuage de points:", max_distance)
    return max_distance#!/usr/bin/env python3

def compute_max_distance(point_cloud):
    # Récupérer les coordonnées des points sous forme de tableau NumPy
    points = np.asarray(point_cloud.points)

    max_distance = 0.0
    num_points = len(points)

    for i in range(num_points):
        for j in range(i + 1, num_points):
            distance = np.linalg.norm(points[i] - points[j])
            if distance > max_distance:
                max_distance = distance

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
    scaling_factor = max_dist_model_3D / max_dist_pc_filtred

    # Redimensionner le nuage de points filtré
    pc_filtred.scale(scaling_factor, center=pc_filtred.get_center())

    # Enregistrer le nuage de points redimensionné
    o3d.io.write_point_cloud(point_cloud_resizing, pc_filtred)

