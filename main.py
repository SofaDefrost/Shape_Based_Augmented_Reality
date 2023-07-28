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

6- ICP : L'utilisateur doit appeler la fonction `run_icp` en fournissant les paramètres suivants : le nom du fichier du nuage de points repositionné et le modèle 3D redimensionné. (Remarque : La description ne mentionne pas les détails spécifiques de la fonction `run_icp`, comme ses paramètres exacts.)

 
    """

import functions.mask as msk
import functions.icp as cp
import functions.translation_m as tm
import functions.repose as rp
import functions.Resize as rz
# import functions.acquisition as aq
import numpy as np
import cv2
from functions.objloader_simple import OBJ
import open3d as o3d
#import time as tm
import functions.project_and_display as proj
import os



#Charger le model 3D
name_model_3D = "data_exemple/FleurDeLisThing.ply"

# Récupération du nuage de points en utilisant la Realsense
name = "data_exemple/fleur_3"
name_pc = name + '.ply'
name_image_2D =cv2.imread( name + '.png')
# Appeler la fonction run_acquisition pour récupérer le nuage de points
#aq.run_acquisition(name_pc, name_image_2D)

# Application du masque
# donner le nom pour le fichier nouveau après l'application du masque

pc_masked_name = name + '_masked.ply'  # donner le nom pour le fichier nouveau après l'application du masque
threshold = 0.222 # donner une valeur pour fixer un seuil de couleur
color_phase= "bleu"
msk.mask(name_pc, pc_masked_name, threshold, color_phase)

# Application de redimensionnement
name_3D="data_exemple/model_3D"
model_3D_resized_name =name_3D + '_resized.ply'
facteur_echelle= 0.00077
rz.Resize(name_model_3D, model_3D_resized_name,facteur_echelle)

# # Application de repositionnement
pc_reposed_name = name + '_reposed.ply'
translation_vector = rp.repose(pc_masked_name, pc_reposed_name)
Mt = tm.translation_matrix(translation_vector)  # Matrice de translation

# # Application de l'icp
# cp.run_icp_1(model_3D_resized_name,pc_reposed_name)
Mr = cp.run_icp_2(pc_reposed_name, model_3D_resized_name)  # Matrice de rotation

# # Matrice extrinsèque
M_ex = np.dot(Mt, Mr)

# # Matrice extrinsèque transposée
# M_ex_t = np.transpose(M_ex)

# # Matrice de calibration de la caméra realsense D415
# M_in = np.array([[629.538, 0, 320.679, 0], [0, 629.538, 234.088, 0], [0, 0, 1, 0]])  # Matrice intrinsèque

#Matrice de calibration de la caméra realsense D405
M_in = np.array([[382.437, 0, 319.688, 0], [0, 382.437, 240.882, 0], [0, 0, 1, 0]])  # Matrice intrinsèque

#Matrice de transformation de 90° selon X
Mat_90 = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])

# # Matrice de projection ==> Matrice extrinsèque transposée * Matrice intrinsèque
Proj_1 = np.dot( M_in,M_ex)

# # Matrice de projection * la matrice de transformation 90°
Projection = np.dot(Proj_1, Mat_90)

# Chargement du fichier obj

obj = OBJ("data_exemple/fleur_3_resized.obj", swapyz=True)

# # Affichage
h, w, _ = name_image_2D.shape
cv2.imshow("frame_avant", name_image_2D)

#recuperer les couleurs de model 3D
color_3D_Model = o3d.io.read_point_cloud("data_exemple/fleur_icp.ply")
vertex_colors = np.asarray(color_3D_Model.colors)


while True:

     frame_apres = proj.project_and_display(name_image_2D, obj, Projection,vertex_colors)
     cv2.imshow("frame_apres", frame_apres)

     if cv2.waitKey(1) & 0xFF == ord('q'):        
         break

cv2.destroyAllWindows()




               


