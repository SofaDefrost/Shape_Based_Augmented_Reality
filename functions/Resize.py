#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  7 09:49:42 2023

@author: tinhinane
"""
import trimesh
def Resize(output_file, pc_resized, facteur_echelle):
    mesh = trimesh.load(output_file)
    #cette ligne charge le fichier PLY situé à l'emplacement /home/tinhinane/programme python nuage de points/Etoileplus.ply et crée un objet Trimesh qui contient les données du nuage de points.
    scaled_vertices = mesh.vertices *facteur_echelle# multiplie la taille par 2 #multiplie chaque coordonnée de vertex du nuage de points par 20, ce qui augmente la taille de l'objet. Les coordonnées de vertex sont stockées dans la propriété vertices de l'objet Trimesh.
    scaled_mesh = trimesh.Trimesh(scaled_vertices, faces=mesh.faces if hasattr(mesh, 'faces') else None)#pour  créer un nouveau Trimesh à partir des nouvelles coordonnées de vertex et des faces originales du nuage de points. Les faces sont stockées dans la propriété faces de l'objet Trimesh.
    scaled_mesh.export(pc_resized)
