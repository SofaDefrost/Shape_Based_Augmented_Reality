#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 29 11:45:56 2023

@author: tinhinane
"""

import mask as msk
import icp
import translation_m as tm
import repose as rp
import resize as rz
import point_cloud as pc
import numpy as np
import cv2
from objloader_simple import *
import open3d as o3d
import aspose.threed as a3d
import time


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

nom = "fleur_7"
point_cloud_name = nom + ".ply"
output_file_name = nom + "_m.ply"
image_name = nom + ".jpeg"

point_cloud = "fleur_7.ply"
output_file = "fleur_m_7.ply"
tpl = "FleurDeLisThing.ply"
tpl_resized = "tpl_resize.ply"
pc_reposed = "fleur_rep.ply"

time.delay(20)

frame = cv2.imread("fleur_6.jpeg")

#pc.run_realSense(point_cloud,frame)

# Masquage du nuage de points
msk.mask(point_cloud, output_file)

# Redimensionnement du nuage de points masqué
rz.resize(tpl,  tpl_resized)

# Repositionnement du nuage de points redimensionné
translation_vector = rp.repose(output_file, pc_reposed)
translation_vector[0] = -translation_vector[0]
translation_vector[1] =translation_vector[1]

# Estimation de la matrice de transformation ICP
transform_matrix = icp.run_icp('tpl_resize.ply', "fleur_rep.ply")

result_icp = "result_icp_6.ply"


# output_obj_file = "fleur_obj.obj"

# ply_to_obj(result_icp, output_obj_file)


# "tpl_resize.ply"
# 'fleur_rep.ply')
Mr = transform_matrix

Mt = tm.translation_matrix(translation_vector)
E = np.dot(Mr, Mt)

# scene = a3d.Scene.from_file(result_icp)
# scene.save('result_icp_6.obj')
# Chargement de l'objet 3D
obj = OBJ('result_icp_6.obj', swapyz=True)
# Matrice de calibration de la caméra D415
calibration_matrix = np.array([[629.538, 0, 320.679, 0], [0, 629.538, 234.088, 0], [0, 0, 1, 0]])

# Matrice de calibration de la caméra D405
#calibration_matrix = np.array( [[382.437, 0, 319.688, 0], [0,	382.437,	240.882, 0], [0, 0, 1, 0]])
# Trans=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])# Matrice identitée
# application de rotation -90° selon x
Mat_90 = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
# application de mat de trans 180° selon Z
#Mat_180_z = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
#Mat_90_z = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
# Projection de l'objet 3D sur l'image et affichage du résultat

proj= np.dot( calibration_matrix, E)
#proj_1= np.dot(proj, Mat_90)
#projection= np.dot(proj_1, Mat_90_z)

projection = np.dot(proj, Mat_90)

#projection = np.dot(proj_1, Mat_180_z)

# angle = np.radians(-45)
# r_y= np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])
# Mat_180_z=np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]])#application de mat de trans 180° selon Z

# projec=np.dot(proj, Trans)
# pr1=np.dot(proj,Trans1)
pcd = o3d.io.read_point_cloud("result_icp_6.ply")
vertex_colors = np.asarray(pcd.colors)

h, w, _ = frame.shape

cv2.imshow("frame_avant", frame)
while True:
    # cv2.imshow("frame_avant", frame)
    frame_apres = project_and_display(frame, obj, projection, vertex_colors)
    cv2.imshow("frame_apres", frame_apres)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
