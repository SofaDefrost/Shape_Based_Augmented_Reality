#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 23 09:45:05 2023

@author: tinhinane
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
import open3d as o3d
import functions.mask as msk
import functions.icp as cp
import functions.translation_m as tm
import functions.repose as rp
import functions.Resize as rz
#import functions.resize as rzz
#import functions.acquisition as aq
from functions.objloader_simple import OBJ
import functions.project_and_display as proj
import functions.project_object_onto_image as project
import functions.ply2obj as po


#Charger le model 3D
name_model_3D = "data_exemple/FleurDeLisThing.ply"

# Récupération du nuage de points en utilisant la Realsense
name = "data_exemple/fleur_5"
name_pc = name + '.ply'
color_image_name = name + '.png'

# Appeler la fonction run_acquisition pour récupérer le nuage de points

#aq.run_acquisition(name_pc, color_image_name)

color_image= cv2.imread(color_image_name)

# Application du masque
# donner le nom pour le fichier nouveau après l'application du masque

pc_masked_name = name + '_masked.ply'  # donner le nom pour le fichier nouveau après l'application du masque
threshold = 0.155 # donner une valeur pour fixer un seuil de couleur
color_phase= "blue"
msk.mask(name_pc, pc_masked_name, threshold, color_phase)

# Application de redimensionnement
name_3D="data_exemple/model_3D"
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




               


