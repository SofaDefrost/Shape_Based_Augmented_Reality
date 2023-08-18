#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 18 11:24:07 2023

@author: tinhinane
"""

import numpy as np
import cv2
import open3d as o3d
import functions.mask as msk
import functions.icp as cp
import functions.translation_m as tm
import functions.repose as rp
import functions.Resize as rz
import functions.angles as an
#import functions.resize as rzz
import realsense.acquisition as aq
from functions.objloader_simple import OBJ
import functions.project_and_display as proj
import functions.ply2obj as po


#Charger le model 3D
name_model_3D = "data_acquisition/nom_modèle_3d.ply"

# Récupération du nuage de points en utilisant la Realsense
name = "data_acquisition/nom"
name_pc = name + '.ply'
color_image_name = name + '.png'

# Appeler la fonction run_acquisition pour récupérer le nuage de points

aq.run_acquisition(name_pc, color_image_name)

color_image= cv2.imread(color_image_name)

# Application du masque
# donner le nom pour le fichier nouveau après l'application du masque

pc_masked_name = name + '_masked.ply'  # donner le nom pour le fichier nouveau après l'application du masque
threshold = 0.155 # donner une valeur pour fixer un seuil de couleur
color_phase= "blue"
msk.mask(name_pc, pc_masked_name, threshold, color_phase)

# Application de redimensionnement
name_3D="data_acquisition/model_3D"
model_3D_resized_name =name_3D + '_resized.ply'
scaling_factor= 0.00099
rz.Resize(name_model_3D, model_3D_resized_name,scaling_factor)
# rzz.resize(model_3D_masked_name,pc_masked_name,model_3D_resized_name) # Call the function to perform automatic resizing



# # Application de repositionnement
pc_reposed_name = name + '_reposed.ply'
translation_vector = rp.repose(pc_masked_name, pc_reposed_name)
translation_vector[0] = -translation_vector[0]
translation_vector[1] = translation_vector[1]
translation_vector[2] = translation_vector[2]

Mt = tm.translation_matrix(translation_vector)  # Matrice de translation
Mt_t= np.transpose(Mt)

 # Application de l'icp  avec  plusieurs matrices de transformation et d'enregister le fichier qui a le plus petit cout 
pc_after_multiple_icp="data_acquisition/pc_after_multiple_icp.ply" 
print("Carry out the first ICP execution to obtain the best suitable initial matrix that has the lowest cost.")
M_icp_1, cost=cp.run_icp_1(model_3D_resized_name,pc_reposed_name,pc_after_multiple_icp) 
print("The best matrix is:", M_icp_1, "with a low cost of:",cost )
print("Please wait a moment for ICP_2 to execute!!")
M_icp_2, _=cp.run_icp_2(pc_reposed_name, pc_after_multiple_icp)

M_icp_2_t= np.transpose(M_icp_2)
M_icp_1_t=np.transpose(M_icp_1)

# Matrice de calibration de la caméra realsense D415
# M_in = np.array([[629.538, 0, 320.679, 0], [0, 629.538, 234.088, 0], [0, 0, 1, 0]])  # Matrice intrinsèque
#Matrice de calibration de la caméra realsense D405
M_in = np.array([[382.437, 0, 319.688, 0], [0, 382.437, 240.882, 0], [0, 0, 1, 0]])  # Matrice intrinsèquqe
M_ex=   M_icp_1 @ M_icp_2
#M_ex =  M_icp_1_t @ M_icp_2_t
matrix= an.angles(M_ex)

# # M_ex =  np.transpose(M_ex)
# M_exx=  Mt @ M_ex 
# matrix = np.array([[-0.38488, 0, -0.922966, 0],
#                     [0.89589, 0.240441, -0.373589, 0],
#                     [0.221919, -0.970664, -0.0925412, 0],
#                     [0, 0, 0, 1]])

# matrix_t = np.transpose(matrix)

angle = np.radians(-90)
Mat_90 = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])


# Matrice de projection ==> Matrice extrinsèque transposée * Matrice intrinsèque
# Proj_1= M_in @ M_exx
Proj_1=Mt @ matrix
Projection= M_in @  Proj_1 

#Appel à la fonction permettant de convertir le fichier template.ply redimensionné au format .obj
obj_file_name= name_3D +'.obj'
po.convert_ply_to_obj(model_3D_resized_name, obj_file_name)
# Chargement du fichier obj

obj = OBJ(obj_file_name, swapyz=True)

# # Affichage
h, w, _ = color_image.shape
cv2.imshow("frame_avant", color_image)

#recuperer les couleurs du l'objet 3D
# color_3D_Model = o3d.io.read_point_cloud(model_3D_resized_name)
# vertex_colors = np.asarray(color_3D_Model.colors)


while True:

     frame_apres = proj.project_and_display_without_colors(color_image,obj, Projection, h, w)
    #Appel à la fonction permettant de projeter l'objet 3D avec ses couleurs spécifiques
     #frame_apres = proj.project_and_display(color_image,obj, Projection, vertex_colors )
     cv2.imshow("frame_apres", frame_apres)

     if cv2.waitKey(1) & 0xFF == ord('q'):        
         break

cv2.destroyAllWindows()