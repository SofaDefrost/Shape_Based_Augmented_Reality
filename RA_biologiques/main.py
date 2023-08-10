#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 23 09:45:05 2023

@author: tinhinane
"""

""" 


L'utilisateur doit :
    Important:
- **Pendant la capture, l'utilisateur devra appuyer sur la touche S du clavier et la touche Q pour arrêter la capture.**
- Vérifier que tous les fichiers existent dans le même dossier.
- Charger le chemin du modèle 3D et de l'image, et assurez-vous que le modèle 3D est repositionner au centre et filtré.
- Donner un nom au nuage de points capturé par la caméra Realsense.
- Fixer le seuil de couleur et la phase de la couleur.
- Vérifier le type de caméra utilisé afin d'utiliser la matrice de calibration spécifique à cette caméra.

Dans ce programme, il fait appel à plusieurs fonctions qui permettent de réaliser les étapes suivantes :

1- Acquisition : Cette fonction permet de récupérer le nuage de points en connectant la caméra RealSense. Pour l'utiliser,
 l'utilisateur doit d'abord initialiser le nom du fichier du nuage de points et le nom de l'image optique. Ensuite, 
 il faut appeler la fonction `run_acquisition` en lui fournissant les paramètres suivants : `nom_fichier_nuage_points` (nom du fichier du nuage de points) et `nom_image_2D`.

2- Masque : Cette fonction permet de filtrer le nuage de points et de sélectionner uniquement la couleur de l'objet. Elle prend les paramètres suivants :

- `nom_nuage_points` : Le nom du fichier du nuage de points généré par la fonction d'acquisition.
- `nom_nuage_points_filtre` : Le nom du fichier dans lequel le nuage de points sera enregistré après l'application du masque.
- `seuil` : Le seuil utilisé pour filtrer les points en fonction de la couleur ou des coordonnées.
- `phase_couleur` : Permet de sélectionner quelle phase des couleurs conserver, soit dans le rouge (intervalle [0-1]) et dans la colonne 0.

3- Repositionnement : Cette fonction "repose" les points du nuage de points pour les repositionner correctement. (Remarque : La description ne mentionne pas de paramètres spécifiques pour cette fonction.)

4- Distance maximale : Cette fonction "max_distance" calcule la distance maximale entre le modèle 3D et le nuage de points filtré et repositionné, puis récupère cette valeur. (Remarque : La description ne mentionne pas de paramètres spécifiques pour cette fonction.)

5- Redimensionnement : Cette fonction "resize" permet de redimensionner le modèle 3D et le nuage de points filtré et repositionné. L'utilisateur doit fournir les paramètres suivants :

- `nom_nuage_points_repositionnes` : Le nom du fichier du nuage de points repositionnés au format PLY.
- `nom_modele_3D` : Le nom du fichier du modèle 3D au format PLY.
- `nuage_points_redimensionne` : Le nom du fichier dans lequel le nouveau nuage de points redimensionné sera enregistré au format PLY.

6- ICP : L'utilisateur doit appeler la fonction `run_icp` en fournissant les paramètres suivants : 
    le nom du fichier du nuage de points repositionnés et le modèle 3D redimensionné.

 
    """
import numpy as np
import cv2
import open3d as o3d
import functions.mask as msk
import functions.icp as cp
import functions.translation_m as tm
import functions.repose as rp
import functions.Resize as rz
import functions.resize as rzz
#import functions.acquisition as aq
from functions.objloader_simple import OBJ
import functions.project_and_display as proj
import functions.project_object_onto_image as project
import functions.ply2obj as po


#Charger le model 3D
name_model_3D = "data_exemple/foie_3D.ply"

# Récupération du nuage de points en utilisant la Realsense
name = "data_exemple/foie"
name_pc = name + '.ply'
color_image_name = name + '.png'

# Appeler la fonction run_acquisition pour récupérer le nuage de points

#aq.run_acquisition(name_pc, color_image_name)

color_image= cv2.imread(color_image_name)

# Application du masque
# donner le nom pour le fichier nouveau après l'application du masque

pc_masked_name = name + '_masked.ply'  # donner le nom pour le fichier nouveau après l'application du masque
threshold_1 = 0.32 # donner une valeur pour fixer un seuil de couleur
color_phase_1= "red"
msk.mask(name_pc, pc_masked_name, threshold_1, color_phase_1)

#Application du masque sur le modèle 3_D
model_3D_masked_name=name_model_3D+ "masked.ply"
msk.mask(name_model_3D,model_3D_masked_name, threshold=0.8, color_phase= "red")
# Application de redimensionnement
name_3D="data_exemple/model_3D"
model_3D_resized_name =name_3D + '_resized.ply'
# scaling_factor= 0.00099
# rz.Resize(model_3D_masked_name, model_3D_resized_name,scaling_factor)
rzz.resize(pc_masked_name,model_3D_masked_name,model_3D_resized_name)
# # Application de repositionnement
pc_reposed_name = name + '_reposed.ply'
translation_vector = rp.repose(pc_masked_name, pc_reposed_name)
translation_vector[0] = -translation_vector[0]
translation_vector[1] = translation_vector[1]
translation_vector[2] = translation_vector[2]

Mt = tm.translation_matrix(translation_vector)  # Matrice de translation
Mt_t= np.transpose(Mt)

 # Application de l'icp  avec  plusieurs matrices de transformation et d'enregister le fichier qui a le plus petit cout 
pc_after_multiple_icp="data_exemple/pc_after_multiple_icp.ply" 
M_icp_1, _=cp.run_icp_1(model_3D_resized_name,pc_reposed_name,pc_after_multiple_icp) 
print("Veuillez patienter un instant pour que l'icp_2 s'exécute.")
M_icp_2, _=cp.run_icp_2(pc_reposed_name, pc_after_multiple_icp)


M_icp_2_t= np.transpose(M_icp_2)
M_icp_1_t=np.transpose(M_icp_1)

# # Matrice de calibration de la caméra realsense D415
#M_in = np.array([[629.538, 0, 320.679, 0], [0, 629.538, 234.088, 0], [0, 0, 1, 0]])  # Matrice intrinsèque

M_ex =  M_icp_1_t @ M_icp_2_t

M_ex=  M_icp_1 @ M_icp_2
M_ex = np.transpose(M_ex)

M_exx=   Mt @ M_ex
#Matrice de calibration de la caméra realsense D405
M_in = np.array([[382.437, 0, 319.688, 0], [0, 382.437, 240.882, 0], [0, 0, 1, 0]])  # Matrice intrinsèquqe

# Matrice de projection ==> Matrice extrinsèque transposée * Matrice intrinsèque
Proj_1= M_in @ M_exx
angle = np.radians(90)

# Mat_90 = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
# Projection=  Proj_1 @ Mat_90
Projection=  Proj_1

#Appel à la fonction permettant de convertir le fichier template.ply redimensionné au format .obj
obj_file_name= name_3D +'.obj'
po.convert_ply_to_obj(model_3D_resized_name, obj_file_name)
# Chargement du fichier obj

obj = OBJ(obj_file_name, swapyz=True)

# # Affichage
h, w, _ = color_image.shape
cv2.imshow("frame_avant", color_image)

#recuperer les couleurs de model 3D
color_3D_Model = o3d.io.read_point_cloud("data_exemple/foie_3D.ply")
vertex_colors = np.asarray(color_3D_Model.colors)


while True:

     frame_apres = proj.project_and_display(color_image,obj, Projection,vertex_colors)
     
     cv2.imshow("frame_apres", frame_apres)

     if cv2.waitKey(1) & 0xFF == ord('q'):        
         break

cv2.destroyAllWindows()




               


