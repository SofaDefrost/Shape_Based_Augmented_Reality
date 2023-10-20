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

    :param pc_filtred_name: Le nom du fichier du nuage de points filtré aprés l'application du masque au format PLY.
    :param model_3D_name: Le nom du fichier du modèle 3D au format PLY.
    :param point_cloud_resizing: Le nom du fichier dans lequel le nouveau nuage de points redimensionné
                                 sera enregistré au format PLY.
    """
    # Chargement du modèle 3D représenté par le fichier model_3D_name à l'aide de la bibliothèque trimesh.
    mesh = trimesh.load(model_3D_name)

    # Calcul des distances maximales entre les points des nuages de points model_3D_name et pc_reposing_name.
    l1 = distance_max(model_3D_name)
    l2 = distance_max(pc_filtred_name)

    # Redimensionnement du nuage de points model_3D_name en multipliant chaque coordonnée de vertex
    # par le rapport des distances maximales l1/l2. Cela ajuste la taille de l'objet.
    scaled_vertices = mesh.vertices * (l1 / l2)

    # Création d'un nouveau Trimesh à partir des nouvelles coordonnées de vertex scaled_vertices
    # et des faces originales du nuage de points d'origine mesh.faces. Les faces sont stockées dans la propriété faces de l'objet Trimesh.
    scaled_mesh = trimesh.Trimesh(scaled_vertices, faces=mesh.faces if hasattr(mesh, 'faces') else None)

    # Exportation du nouveau nuage de points redimensionné dans le fichier point_cloud_resizing au format PLY.
    scaled_mesh.export(point_cloud_resizing)
