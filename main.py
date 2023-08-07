#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 23 09:45:05 2023

@author: tinhinane
"""

""" 
Voici la correction des erreurs dans le texte :

L'utilisateur doit :
- Vérifier que tous les fichiers existent dans le même dossier.
- Charger le chemin du modèle 3D, de l'image
- Donner un nom au nuage de points capturé par la Realsense.
- Fixer le seuil de couleur et la phase de la couleur.
- Vérifier le type de la caméra utilisée afin d'utiliser la matrice de calibration spécifique à cette caméra.

Dans ce main il fait appel à toutes les fonctions qui permet de faire:
    
   Voici les corrections apportées au texte en soulignant les erreurs :

Dans ce programme, il fait appel à plusieurs fonctions qui permettent de réaliser les étapes suivantes :

1- Acquisition : Cette fonction permet de récupérer le nuage de points en branchant la caméra RealSense. Pour l'utiliser,
 l'utilisateur doit d'abord initialiser le nom du fichier du nuage de points et le nom de l'image optique. Ensuite, 
 il faut appeler la fonction `run_acquisition` en lui fournissant les paramètres suivants : `file_name_point_cloud` (nom du fichier du nuage de points) et `name_image_2D`.

2- Masque : Cette fonction permet de filtrer le nuage de points et de sélectionner uniquement la couleur de l'objet. Elle prend les paramètres suivants :

- `point_cloud_name` : Le nom du fichier du nuage de points généré par la fonction acquisition.
- `filtered_cloud_name` : Le nom du fichier dans lequel le nuage de points sera enregistré après l'application du masque.
- `threshold` : Le seuil utilisé pour filtrer les points selon la couleur ou les coordonnées.
- `color_phase` : Permet de sélectionner quelle phase des couleurs conserver, soit dans le rouge (intervalle [0-1]) et dans la colonne 0.

3- Repositionnement : Cette fonction "repose" les points du nuage de points pour les repositionner correctement. (Remarque : La description ne mentionne pas de paramètres spécifiques pour cette fonction.)

4- Distance maximale : Cette fonction "max_distance" calcule la distance maximale entre le modèle 3D et le nuage de points filtré et repositionné, puis récupère cette valeur. (Remarque : La description ne mentionne pas de paramètres spécifiques pour cette fonction.)

5- Redimensionnement : Cette fonction "resize" permet de redimensionner le modèle 3D et le nuage de points filtré et repositionné. L'utilisateur doit fournir les paramètres suivants :

- `pc_reposing_name` : Le nom du fichier du nuage de points repositionné au format PLY.
- `model_3D_name` : Le nom du fichier du modèle 3D au format PLY.
- `point_cloud_resizing` : Le nom du fichier dans lequel le nouveau nuage de points redimensionné sera enregistré au format PLY.

6- ICP : L'utilisateur doit appeler la fonction `run_icp` en fournissant les paramètres suivants : 
    le nom du fichier du nuage de points repositionné et le modèle 3D redimensionné. 


 
    """

import functions.mask as msk
import functions.icp as cp
import functions.translation_m as tm
import functions.repose as rp
import functions.Resize as rz
#import functions.acquisition as aq
import numpy as np
import cv2
from functions.objloader_simple import OBJ
import open3d as o3d
# import time as tm
import functions.project_and_display as proj
import functions.project_object_onto_image as project



#Charger le model 3D
name_model_3D = "data_exemple/FleurDeLisThing.ply"

# Récupération du nuage de points en utilisant la Realsense
name = "data_exemple/fleur_9"
name_pc = name + '.ply'
color_image_name = name + '.png'
# Appeler la fonction run_acquisition pour récupérer le nuage de points
#aq.run_acquisition(name_pc, color_image_name)
color_image= cv2.imread(color_image_name)
# Application du masque
# donner le nom pour le fichier nouveau après l'application du masque

pc_masked_name = name + '_masked.ply'  # donner le nom pour le fichier nouveau après l'application du masque
threshold = 0.155 # donner une valeur pour fixer un seuil de couleur
color_phase= "bleu"
msk.mask(name_pc, pc_masked_name, threshold, color_phase)

# Application de redimensionnement
name_3D="data_exemple/model_3D"
model_3D_resized_name =name_3D + '_resized.ply'
facteur_echelle= 0.00099
rz.Resize(name_model_3D, model_3D_resized_name,facteur_echelle)

# # Application de repositionnement
pc_reposed_name = name + '_reposed.ply'
translation_vector = rp.repose(pc_masked_name, pc_reposed_name)
translation_vector[0] =-translation_vector[0]
translation_vector[1] =translation_vector[1]
translation_vector[2] =translation_vector[2]

Mt = tm.translation_matrix(translation_vector)  # Matrice de translation
Mt_t= np.transpose(Mt)
angle = np.radians(90)
Mat_90 = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), np.sin(angle), 0], [0, -np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
 # Application de l'icp  avec  plusieurs matrices de transformation et d'enregister le fichier qui a le plus petit cout 
 
M_icp_1, _=cp.run_icp_1(model_3D_resized_name,pc_reposed_name) 

M_icp_2, _=cp.run_icp_2(pc_reposed_name, model_3D_resized_name)
# M_icp_2= np.asarray([[0.862, 0.011, -0.507, 0.5], [-0.139, 0.967, -0.215, 0.7], [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
M_icp_2_t= np.transpose(M_icp_2)
M_icp_1_t=np.transpose(M_icp_1)

# # Matrice de calibration de la caméra realsense D415
#M_in = np.array([[629.538, 0, 320.679, 0], [0, 629.538, 234.088, 0], [0, 0, 1, 0]])  # Matrice intrinsèque
M_ex= M_icp_1 @M_icp_2_t

M_exx=  Mt @ M_ex 
#Matrice de calibration de la caméra realsense D405
M_in = np.array([[382.437, 0, 319.688, 0], [0, 382.437, 240.882, 0], [0, 0, 1, 0]])  # Matrice intrinsèquqe




# Matrice de projection ==> Matrice extrinsèque transposée * Matrice intrinsèque


Proj_1= M_in @ M_exx

Projection=  Proj_1 

# Chargement du fichier obj

obj = OBJ("data_exemple/fleur_3_resized.obj", swapyz=True)

# # Affichage
h, w, _ = color_image.shape
cv2.imshow("frame_avant", color_image)

#recuperer les couleurs de model 3D
# color_3D_Model = o3d.io.read_point_cloud("data_exemple/fleur_icp.ply")
# vertex_colors = np.asarray(color_3D_Model.colors)


while True:

     frame_apres = proj.project_and_display(color_image,obj, Projection,h,w)
     
     cv2.imshow("frame_apres", frame_apres)

     if cv2.waitKey(1) & 0xFF == ord('q'):        
         break

cv2.destroyAllWindows()




               


