#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 21 18:08:11 2023

@author: tinhinane
"""
import trimesh
import functions.distance_max as dm

def resize(pc_filtred_name, model_3D_name, point_cloud_resizing):
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
    l1 = dm.distance_max(model_3D_name)
    l2 = dm.distance_max(pc_filtred_name)

    # Redimensionnement du nuage de points model_3D_name en multipliant chaque coordonnée de vertex
    # par le rapport des distances maximales l1/l2. Cela ajuste la taille de l'objet.
    scaled_vertices = mesh.vertices * (l1 / l2)

    # Création d'un nouveau Trimesh à partir des nouvelles coordonnées de vertex scaled_vertices
    # et des faces originales du nuage de points d'origine mesh.faces. Les faces sont stockées dans la propriété faces de l'objet Trimesh.
    scaled_mesh = trimesh.Trimesh(scaled_vertices, faces=mesh.faces if hasattr(mesh, 'faces') else None)

    # Exportation du nouveau nuage de points redimensionné dans le fichier point_cloud_resizing au format PLY.
    scaled_mesh.export(point_cloud_resizing)
