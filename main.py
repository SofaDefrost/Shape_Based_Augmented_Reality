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

2. **Masquage** : Cette fonction permet de filtrer le nuage de points selon un filtre hsv. Elle prend les paramètres suivants :

- `points` : Liste des coordonées du nuages de points
- `couleurs` : Liste des couleurs associées à ce nuage de points
- `mask` : Masque hsv à appliquer 

3. **Filtrage Bruit** : Cette fonction permet de filtrer le nuage de points pour éviter le bruit environnant. La fonction crée une interface TKinter qui permet de selectionner la taille du rayon de filtrage (par rapport au centre de masse) :

- `points` : Liste des coordonées du nuages de points
- `couleurs` : Liste des couleurs associées à ce nuage de points

4. **Repositionnement** : Cette fonction "repose" les points du nuage de points pour les repositionner correctement. (Remarque : La description ne mentionne pas de paramètres spécifiques pour cette fonction.)

5. **Redimensionnement automatique** (resize_auto) : Cette fonction permet de redimensionner le modèle 3D et le nuage de points filtré et repositionné. L'utilisateur doit fournir les paramètres suivants :

- `nom_nuage_points_repositionne` : Nom du fichier du nuage de points repositionnés au format PLY.
- `nom_modele_3D` : Nom du fichier du modèle 3D au format PLY.
- `nuage_points_redimensionne` : Nom du fichier dans lequel le nouveau nuage de points redimensionné sera enregistré au format PLY.



6. **Redimensionnement avec seuil** (Resize_pas_auto) :
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


import functions.icp as cp
import functions.translation_m as tm
import functions.repose as rp
import functions.resize as rz
import realsense.acquisition as aq
from functions.objloader_simple import OBJ
import functions.project_and_display as proj
import functions.ply2obj as po
import realsense.utils.hsv as apply_hsv
import realsense.utils.interface_hsv_image as get_filtre_hsv
import realsense.utils.convert as cv
import functions.recover_realsense_matrix as rc
import functions.transformations as tf
import realsense.utils.crop_points_cloud as cr
from realsense.utils import filtrage_bruit as bruit
from realsense.utils import realsense_pc as rpc
from realsense.utils import reduction_densite_pc as dens
from scipy.spatial import cKDTree

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
name = "data_exemple/fleure"

# name_model_3D = "labo_biologie/2eme_semaine/foie_L.ply"
# name="labo_biologie/2eme_semaine/_foie_deuxieme_jour__Thibaud0"

# Marche bien (ne pas changer les paramètres) : 
# _foie_deuxieme_jour__Thibaud4 (icp ok et affichage ok)
# _foie_deuxieme_jour__Thibaud14 (icp bien mais affichage limite)
# _foie_deuxieme_jour__Thibaud20" (icp excellent mais affichage limite)
# _foie_deuxieme_jour_dedos__Thibaud3 (icp et affichage très bon)
# _foie_deuxieme_jour_dedos__Thibaud9 (galère à faire converger correctement (il fait bien meurtrir le fichier avec le masque hsv) mais sinon icp et affichage bon )
# _foie_deuxieme_jour_dedos__Thibaud10 (icp et affichage bien)

###########################################################

################### Acquisition ###########################

name_pc = name + '.ply'
color_image_name = name + '.png'

# Récupération du nuage de points en utilisant la Realsense
# Appeler une fonction d'acquisition pour récupérer le nuage de points et les couleurs

### Version Tinhinane (penser à également décommenter la partie pour la projection)

# aq.run_acquisition(name_pc, color_image_name)

### Version Thibaud (penser à également décommenter la partie pour la projection)

# points_acquisition_originale,couleurs_acquisition_originale=aq.points_and_colors_realsense(color_image_name)
# couleurs_acquisition_originale=rpc.colors_relasense_sofa(couleurs_acquisition_originale)
# cv.create_ply_file(points_acquisition_originale,couleurs_acquisition_originale,name_pc)

color_image= cv2.imread(color_image_name)
points,couleurs=cv.ply_to_points_and_colors(name_pc)

###########################################################

##################### Selectionner Zone ####################

# Fonctionne uniquement avec la version de Thibaud + doit raccorder aux restes du code  

points_crop,couleurs_crop=cr.crop_points_cloud(color_image_name,points,couleurs,640)

############################################################

###################### Masquage ###########################

# Détermination du masque

mask_hsv=get_filtre_hsv.interface_hsv_image(color_image_name)

# Application du masque

pc_masked_name = name + '_masked.ply'  # donner le nom pour le fichier nouveau après l'application du masque
points_filtrés,colors= apply_hsv.mask(points_crop,couleurs_crop,mask_hsv)

###########################################################

####################### Filtrage Bruit #####################

name_bruit=name+'_filtre_bruit.ply'
point_filtre_bruit=bruit.interface_de_filtrage_de_points(points_filtrés,points_filtrés)[0]
cv.create_ply_file_without_colors(point_filtre_bruit,name_bruit)

###########################################################

######################### Redimensionnement du modèle 3D ##################################

name_3D=name+"_model_3D"
model_3D_resized_name =name_3D + '_resized.ply'

### Pour le resize pas auto 

# Application de redimensionnement
# scaling_factor = 0.00099 #0.0011
# rz.Resize_pas_auto(name_model_3D, model_3D_resized_name,scaling_factor)

### Pour le resize auto
name_model_3D_reduit_densite=name_model_3D
nuage_de_point_trop_gros=True

# On divise le nombre de points par deux jusqu'à ce que ce soit suffisant pour l'algo de resize automatique
while nuage_de_point_trop_gros:
    try:
        rz.resize_auto(name_bruit,name_model_3D_reduit_densite,model_3D_resized_name) # Call the function to perform automatic resizing
        nuage_de_point_trop_gros=False
    except Exception as e:
        # Code à exécuter en cas d'erreur
        print("Trop de points dans le nuage de point pour la fonction de resize automatique, on re-essaie en divisant le nombre de points du nuage par deux !")
        name_model_3D_reduit_densite=name_3D+"_reduit_densite.ply"
        dens.reduction_densite_pc(name_model_3D,name_model_3D_reduit_densite,0.5)
        name_model_3D=name_model_3D_reduit_densite

# On s'assure également que le nuage de point récupéré par la caméra soit d'une taille simailaire à notre modèle 3D

# point_model_reduit,_=cv.ply_to_points_and_colors(name_model_3D_reduit_densite)
# point_bruits,_=cv.ply_to_points_and_colors(name_bruit)
# name_bruit_reduit=name_bruit

# while len(point_bruits) > 2*len(point_model_reduit):
#     name_bruit_reduit=name+"bruit_reduit.ply"
#     dens.reduction_densite_pc(name_bruit,name_bruit_reduit,0.5)
#     name_bruit=name_bruit_reduit
#     point_bruits,_=cv.ply_to_points_and_colors(name_bruit)

###########################################################

################### Repositionnment du repère de la caméra dans celui de l'objet #####################

# # Application de repositionnement
pc_reposed_name = name + '_reposed.ply'
translation_vector = rp.repose(name_bruit, pc_reposed_name)
translation_vector[0] = translation_vector[0]
translation_vector[1] = translation_vector[1]
translation_vector[2] = translation_vector[2]

Mt = tm.translation_matrix(translation_vector)  # Matrice de translation
# print("Matrice de translation:",Mt)

###########################################################

################ Remise en place du modèle 3D #############

# ## Que pour le foie

# # On inverse suivant y le sens du modèle 3D (parce que il n'est pas dans le bon sens)

# angle = np.radians(180)
# Mat_y = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])

# # On récupère les points de notre modèle 3D et on applique les transformations (rotation et translations)
# model_3D_resized_name_points,model_3D_resized_name_coulors=cv.ply_to_points_and_colors(model_3D_resized_name)
# model_3D_resized_name_points = np.column_stack((model_3D_resized_name_points, np.ones(len(model_3D_resized_name_points)))) # On met au bon format les points (on rajoute une coordonnée de 1)

# M=Mat_y

# model_3D_resized_name_points=[M @ p for p in model_3D_resized_name_points]
# cv.create_ply_file_without_colors(model_3D_resized_name_points,model_3D_resized_name)
# model_3D_points,_=cv.ply_to_points_and_colors(model_3D_resized_name)

###########################################################

################ Matrice de pré-rotation ###################

M_icp_1=cp.find_the_best_pre_rotation(model_3D_resized_name,pc_reposed_name)
model_3D_after_pre_rotations=name+"_after_pre_rotations.ply"
p,c=cv.ply_to_points_and_colors(model_3D_resized_name)
source_rotated=[np.dot(point,M_icp_1) for point in p]
cv.create_ply_file_without_colors(source_rotated,model_3D_after_pre_rotations)

M_icp_1 = np.hstack((M_icp_1, np.array([[0], [0], [0]])))
M_icp_1 = np.vstack((M_icp_1, np.array([0, 0, 0, 1])))

M_icp_1_inv = np.linalg.inv(M_icp_1)

###########################################################

###################### Matrice ICP #########################

print("Please wait a moment for ICP to execute!!")
M_icp_2, _=cp.run_icp(model_3D_after_pre_rotations,pc_reposed_name) # Pour la version avec pré-rotation
# M_icp_2, _=cp.run_icp(model_3D_resized_name,pc_reposed_name) # Pour la version sans pré-rotation
# print("M_icp :",M_icp_2)

# On ajuste la matrice d'ICP dans le repère de la caméra
angles_ICP2=transformation_matrix_to_euler_xyz(M_icp_2)
print("Voici les angles de l'ICP : ",angles_ICP2)

# Version Thibaud
x=-angles_ICP2[0] # Dépend de la version choisie (Thibaud ou Tinhinane)
y=angles_ICP2[1] # Idem
z=-angles_ICP2[2] # Idem


M_icp_2_inv = np.linalg.inv(matrix_from_angles(x,y,z)) #  Important de calculer l'inverse parce que nous on veut faire bouger le modèle de CAO sur le nuage de points (et pas l'inverse !)

###########################################################

################# Affiche et exporte Thibaud #######################

# ## L'idée dans cette version est de ne pas faire de projection mais plutot d'utiliser le nuage de point initialement capturé par la caméra : 
# ## On va déterminer les nouvelles coordonées de notre objet 3D et chercher les points correspondant dans le nuage de la caméra.
# ## On viendra alors modifier la couleur de ces points  

# # On récupère les points et les couleurs de notre nuage de points (et on les convertit au bon format)
# points_acquisition_originale,couleurs_acquisition_originale=cv.ply_to_points_and_colors(name_pc)
# points_acquisition_originale = [tuple(row) for row in points_acquisition_originale]
# points_acquisition_originale = np.array([(float(x), float(y), float(z)) for (x, y, z) in points_acquisition_originale], dtype=np.float64)
# couleurs_acquisition_originale = couleurs_acquisition_originale.astype(int)

# # On récupère les points de notre modèle 3D et on applique les transformations (rotation et translations)
# model_3D_resized_name_points,model_3D_resized_name_coulors=cv.ply_to_points_and_colors(model_3D_resized_name)
# model_3D_resized_name_points = np.column_stack((model_3D_resized_name_points, np.ones(len(model_3D_resized_name_points)))) # On met au bon format les points (on rajoute une coordonnée de 1)

# M=Mt @ M_icp_1_inv @ M_icp_2_inv # Matrice de "projection"

# name_transformed_model=name+"_transformed_3D_model.ply"

# model_3D_resized_name_points=[M @ p for p in model_3D_resized_name_points]
# cv.create_ply_file_without_colors(model_3D_resized_name_points,name_transformed_model)
# model_3D_points,_=cv.ply_to_points_and_colors(name_transformed_model)

# # On cherche maintenant à superposer les deux nuages de points 
# # Pour cela on utilise des arbres KD

# tree = cKDTree(points_acquisition_originale)

# # Liste pour stocker les indices des points les plus proches dans le second nuage
# indices_des_plus_proches = []

# # Pour chaque point dans le premier nuage
# for point in model_3D_points:
#     # On recherche le point le plus proche dans le second nuage
#     distance, indice_plus_proche = tree.query(point)
    
#     if True: #distance < 0.003:
#         # On concerve l'indice du point le plus proche
#         indices_des_plus_proches.append(indice_plus_proche)

# # On modifie les couleurs des points trouvés dans l'étape précédente (c'est la ou se situe notre objet donc on va l'indiquer avec une couleurs spéciale ici bleue)
# print(model_3D_resized_name_coulors)
# print(len(model_3D_resized_name_coulors))
# for indice in indices_des_plus_proches:
#     print(indice)
#     if len(model_3D_resized_name_coulors)==0:
#         couleur_objet=np.array([0,0,255]) # Bleu (couleur de base)
#     else:
#         couleur_objet=model_3D_resized_name_coulors[indice]
#     couleurs_acquisition_originale[indice]=couleur_objet
#     # On fait un peu autour pour que ce soit plus visible
#     couleurs_acquisition_originale[indice+1]=couleur_objet
#     couleurs_acquisition_originale[indice-1]=couleur_objet
#     couleurs_acquisition_originale[indice+640]=couleur_objet
#     couleurs_acquisition_originale[indice+640+1]=couleur_objet
#     couleurs_acquisition_originale[indice+640-1]=couleur_objet
#     couleurs_acquisition_originale[indice-640-1]=couleur_objet
#     couleurs_acquisition_originale[indice-640]=couleur_objet
#     couleurs_acquisition_originale[indice-640+1]=couleur_objet

# # On enregistre
# cv.creer_image_a_partir_de_liste(couleurs_acquisition_originale,640,480,name+"projection.png")
# color_image1= cv2.imread(name+"projection.png")

# # On affiche
# while True:
#     cv2.imshow("projection",color_image1)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cv2.destroyAllWindows()

#####################################################################

################# Affiche et exporte Tinhinane ######################

# ## Calcul des points de projections

# Matrice de projection ==> Matrice intrinsèque * Matrice extrinsèque 
# Matrice extrinsèque ==> ensemble des modifications (translations et rotations) à appliquer au modèle CAO

## Matrice de calibration (Matrice intrinsèque) ####

calibration_matrix = rc.recover_matrix_calib()
M_in = np.hstack((calibration_matrix, np.zeros((3, 1))))
M_in = np.vstack((M_in, np.array([0, 0, 0, 1])))
# M_in = np.array([[382.437, 0, 319.688, 0], [0, 382.437, 240.882, 0], [0, 0, 1, 0]])  # Matrice intrinsèquqe Tinhinane je pense à supprimer
# M_in = np.array([[423.84763, 0, 319.688, 0], [0,423.84763, 240.97697, 0], [0, 0, 1, 0]])  # Matrice intrinsèquqe Tinhinane remaster à la main je pense à supprimer

#### Matrice pour replaquer le modèle 3D ####
# (Initialement le modéle n'est pas dans la position que l'on souhaite)

angle = np.radians(-90)
Mat_x = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
angle = np.radians(180)
Mat_y = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])

#### Calcul final de la projection ####

Projection= M_in @ Mt @ M_icp_1_inv @ M_icp_2_inv @ Mat_y @ Mat_x 

#Appel à la fonction permettant de convertir le fichier template.ply redimensionné au format .obj
obj_file_name= name_3D +'.obj'
po.convert_ply_to_obj(model_3D_resized_name, obj_file_name)
# Chargement du fichier obj

obj = OBJ(obj_file_name, swapyz=True)

# # Affichage
h, w, _ = color_image.shape
cv2.imshow("frame_avant", color_image)
#recuperer les couleurs du l'objet 3D
color_3D_Model = o3d.io.read_point_cloud(model_3D_resized_name)
vertex_colors = np.asarray(color_3D_Model.colors)
if len(vertex_colors)==0:
    vertex_colors=[[0, 0, 255] for i in range(np.asarray(color_3D_Model.points))]

while True:

    # frame_apres = proj.project_and_display_without_colors(color_image,obj, Projection, h, w)
    # Appel à la fonction permettant de projeter l'objet 3D avec ses couleurs spécifiques
    frame_apres = proj.project_and_display(color_image,obj, Projection, vertex_colors)
    cv2.imshow("frame_apres", frame_apres)
    if cv2.waitKey(1) & 0xFF == ord('q'):        
        break

cv2.destroyAllWindows()




               


