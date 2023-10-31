#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 23 09:45:05 2023
Updated on Wed Oct 23 11:44:38 2023

@author: Tinhinane and Thibaud
"""

""" 


L'utilisateur doit prendre en compte les points suivants :

**Important**:
- **Pendant la capture, l'utilisateur devra appuyer sur la touche S du clavier et la touche Q pour arrêter la capture.**
- Assurez-vous que tous les fichiers existent dans le même dossier.
- Indiquez le chemin vers le modèle 3D ainsi que l'image.
- Nommez le nuage de points acquis par la caméra Realsense.
- Définissez le seuil de couleur et la phase de couleur.
- Vérifiez le type de caméra utilisé pour pouvoir appliquer la matrice de calibration spécifique à cette caméra.
- Pour arrêter l'exécution, il suffit de cliquer sur la touche Q.'

Ce programme utilise plusieurs fonctions pour effectuer les étapes suivantes :

1. **Acquisition** : Cette fonction permet de récupérer le nuage de points en connectant la caméra RealSense. Pour l'utiliser, 
l'utilisateur doit d'abord initialiser le nom du fichier du nuage de points et le nom de l'image optique. 
Ensuite, appelez la fonction `run_acquisition` en fournissant les paramètres suivants : 
    `nom_fichier_nuage_points` (nom du fichier du nuage de points) et `nom_image_2D`.

2. **Masquage** : Cette fonction permet de filtrer le nuage de points et de sélectionner uniquement la couleur de l'objet. Elle prend les paramètres suivants :

- `nom_nuage_points` : Nom du fichier du nuage de points généré par la fonction d'acquisition.
- `nom_nuage_points_filtre` : Nom du fichier dans lequel le nuage de points sera enregistré après l'application du masque.
- `seuil` : Seuil utilisé pour filtrer les points en fonction de la couleur ou des coordonnées.
- `phase_couleur` : Permet de sélectionner quelle phase des couleurs conserver, soit dans le rouge (intervalle [0-1]) et dans la colonne 0.

3. **Repositionnement** : Cette fonction "repose" les points du nuage de points pour les repositionner correctement. (Remarque : La description ne mentionne pas de paramètres spécifiques pour cette fonction.)

4. **Redimensionnement automatique** (resize avec 'r' en minuscule) : Cette fonction "resize" permet de redimensionner le modèle 3D et le nuage de points filtré et repositionné. L'utilisateur doit fournir les paramètres suivants :

- `nom_nuage_points_repositionne` : Nom du fichier du nuage de points repositionnés au format PLY.
- `nom_modele_3D` : Nom du fichier du modèle 3D au format PLY.
- `nuage_points_redimensionne` : Nom du fichier dans lequel le nouveau nuage de points redimensionné sera enregistré au format PLY.

 **Distance maximale** : Cette fonction "max_distance" calcule la distance maximale 
entre le modèle 3D et le nuage de points filtré et repositionné, puis récupère cette valeur. 


4. **Redimensionnement avec seuil** (redimensionnement avec 'R' en majuscule) :
    - `nom_nuage_points_repositionne`
    - Nom du fichier de sortie après le redimensionnement
    - Facteur de redimensionnement

    
***Remarque*** : Si l'utilisateur souhaite éviter de refaire l'acquisition à chaque fois, 
    il peut commenter la ligne "import functions.acquisition as aq" et utiliser "aq.run_acquisition(name_pc, color_image_name)". 
    De plus, si l'objet 3D possède ses propres couleurs spécifiques, l'utilisateur devra appeler ou décommenter l'appel à la fonction project_and_display.
    Dans le cas contraire, il devra décommenter la ligne contenant la fonction project_and_display_without_colors.
    """
import numpy as np
import cv2
import copy
import open3d as o3d
import functions.mask as msk
import functions.icp as cp
import functions.translation_m as tm
import functions.repose as rp
import functions.resize as rz
import functions.filter_referential  as an
#import functions.resize as rzz
import realsense.acquisition as aq
from functions.objloader_simple import OBJ
import functions.project_and_display as proj
import functions.ply2obj as po
import realsense.utils.hsv as apply_hsv
import realsense.utils.interface_hsv_image as get_filtre_hsv
import realsense.utils.convert as cv
import functions.recover_realsense_matrix as rc
import functions.transformations as tf
import crop_points_cloud as cr
from realsense.utils import filtrage_bruit as bruit
from realsense.utils import realsense_pc as rpc

# Je ne sais pas pourquoi mais si ces fonctions ne sont pas définie dans ce fichier ça ne marche pas

def transformation_matrix_to_euler_xyz(transformation_matrix):
    # Extraire la sous-matrice de rotation 3x3
    rotation_matrix = transformation_matrix[:3, :3]
    
    # Utiliser la fonction euler_from_matrix pour obtenir les angles d'Euler en XYZ
    euler_angles = tf.euler_from_matrix(rotation_matrix, 'sxyz')  # 'sxyz' order for XYZ Euler angles
    
    return euler_angles

def matrix_from_angles(angle_x, angle_y, angle_z):
    rotation_matrix = np.eye(4)
    rotation_matrix[:3, :3] = tf.euler_matrix(angle_x, angle_y, angle_z, 'sxyz')[:3, :3]
    return rotation_matrix

############### Loading ####################

# Charger le model 3D
name_model_3D = "data_exemple/FleurDeLisThing.ply"

###########################################################

################### Acquisition ###########################

# Récupération du nuage de points en utilisant la Realsense
name = "data_exemple/fleur"
name_pc = name + '.ply'
color_image_name = name + '.png'

# Appeler la fonction points_and_colors_realsense pour récupérer le nuage de points et les couleurs

aq.run_acquisition(name_pc, color_image_name)

color_image= cv2.imread(color_image_name)

###########################################################

# #################### Selectionner Zone ####################
# Provoque des problèmes

# points_crop,couleurs_crop=cr.crop_points_cloud(color_image_name,points,colors)

# ###########################################################

###################### Masquage ###########################

# Détermination du masque

points,couleurs=cv.ply_to_points_and_colors(name_pc)
mask_hsv=get_filtre_hsv.interface_hsv_image(color_image_name)

# Application du masque
# donner le nom pour le fichier nouveau après l'application du masque

pc_masked_name = name + '_masked.ply'  # donner le nom pour le fichier nouveau après l'application du masque

points_filtrés,colors= apply_hsv.mask(points,couleurs,mask_hsv)

###########################################################

####################### Filtrage Bruit #####################
# Peut être pas super utile...

name_bruit=name+'_filtre_bruit.ply'
point_filtre_bruit=bruit.interface_de_filtrage_de_points(points_filtrés,points_filtrés)[0]
cv.create_ply_file_without_colors(point_filtre_bruit,name_bruit)

###########################################################


######################### Redimensionnement du modèle 3D ##################################

# Application de redimensionnement
name_3D=name+"_model_3D"
model_3D_resized_name =name_3D + '_resized.ply'
scaling_factor = 0.001 # 0.00099 #0.0011
rz.Resize_pas_auto(name_model_3D, model_3D_resized_name,scaling_factor)
# rz.resize_auto(name_bruit,name_model_3D,model_3D_resized_name) # Call the function to perform automatic resizing

###########################################################

################### Repositionnment du repère de la caméra dans celui de l'objet #####################

# # Application de repositionnement
pc_reposed_name = name + '_reposed.ply'
translation_vector = rp.repose(name_bruit, pc_reposed_name)
translation_vector[0] = -translation_vector[0]
translation_vector[1] = translation_vector[1]
translation_vector[2] = translation_vector[2]

Mt = tm.translation_matrix(translation_vector)  # Matrice de translation
# print("Matrice de translation:",Mt)

###########################################################

################ Matrice de pré-rotation ###################

M_icp_1=cp.find_the_best_pre_rotation(model_3D_resized_name,pc_reposed_name)
model_3D_after_pre_rotations=name+"_after_pre_rotations.ply"
p,c=cv.ply_to_points_and_colors(model_3D_resized_name)
source_rotated=[np.dot(point,M_icp_1) for point in p]
cv.create_ply_file_without_colors(source_rotated,model_3D_after_pre_rotations)

M_icp_1 = np.hstack((M_icp_1, np.array([[0], [0], [0]])))
M_icp_1 = np.vstack((M_icp_1, np.array([0, 0, 0, 1])))

###########################################################

###################### Matrice ICP #########################

print("Please wait a moment for ICP to execute!!")
M_icp_2, _=cp.run_icp(model_3D_after_pre_rotations,pc_reposed_name) # Pour la version avec pré-rotation
# M_icp_2, _=cp.run_icp(model_3D_resized_name,pc_reposed_name) # Pour la version sans pré-rotation
# print("M_icp :",M_icp_2)

# On ajuste la matrice dICP dans le repère de la caméra
angles_ICP2=transformation_matrix_to_euler_xyz(M_icp_2)
print("Voici les angles de l'ICP : ",angles_ICP2)

x=-angles_ICP2[0]
y=-angles_ICP2[1] # Utile ?
z=angles_ICP2[2]

M_icp_2_inv = np.linalg.inv(matrix_from_angles(x,y,z)) #  Important de calculer l'inverse parce que nous on veut faire bouger le modèle de CAO sur le nuage de points (et pas l'inverse !)


###########################################################

########## Calcul des points de projections ###############

# Matrice de projection ==> Matrice intrinsèque * Matrice extrinsèque 
# Matrice extrinsèque ==> ensemble des modifications (translations et rotations) à appliquer au modèle CAO

#### Matrice de calibration (Matrice intrinsèque) ####

calibration_matrix = rc.recover_matrix_calib()
M_in = np.hstack((calibration_matrix, np.zeros((3, 1))))
# M_in = np.array([[382.437, 0, 319.688, 0], [0, 382.437, 240.882, 0], [0, 0, 1, 0]])  # Matrice intrinsèquqe Tinhinane
M_in = np.array([[423.84763, 0, 319.688, 0], [0,423.84763, 240.97697, 0], [0, 0, 1, 0]])  # Matrice intrinsèquqe Tinhinane remaster à la main

#### Matrice pour replaquer le modèle 3D ####
# (Initialement le modéle n'est pas dans la position que l'on souhaite)

angle = np.radians(-90)
Mat_x = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
angle = np.radians(180)
Mat_y = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])

#### Calcul final de la projection ####

Projection= M_in @ Mt @ M_icp_1 @ M_icp_2_inv @ Mat_y @ Mat_x 

###########################################################

################# Affiche et exporte #######################

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
    # Appel à la fonction permettant de projeter l'objet 3D avec ses couleurs spécifiques
    # frame_apres = proj.project_and_display(color_image,obj, Projection, vertex_colors )
    cv2.imshow("frame_apres", frame_apres)
    if cv2.waitKey(1) & 0xFF == ord('q'):        
        break

cv2.destroyAllWindows()




               


