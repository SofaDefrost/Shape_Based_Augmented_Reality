import os

import numpy as np
import cv2
import open3d as o3d
from scipy.spatial import cKDTree

from functions.objloader_simple import OBJ

import functions.icp as cp
import functions.translation_m as tm
import functions.recover_realsense_matrix as rc
import functions.transformations as tf
import functions.project_and_display as proj
import functions.ply2obj as po

from realsense import acquisition_realsense as aq
from realsense.utils import processing_ply as pp
from realsense.utils import processing_array as pa
from realsense.utils import display_function_Tkinter as Tk
from realsense.utils import processing_img as pi

# Je ne sais pas pourquoi mais si ces fonctions ne sont pas définie dans ce fichier ça ne marche pas


def transformation_matrix_to_euler_xyz(transformation_matrix):
    """
    Convertit une matrice de transformation en angles d'Euler selon l'ordre XYZ.

    Args:
        transformation_matrix (numpy.ndarray): Matrice de transformation 4x4.

    Returns:
        Tuple[float, float, float]: Angles d'Euler (radians) selon l'ordre XYZ.
    """
    # Extraire la sous-matrice de rotation 3x3
    rotation_matrix = transformation_matrix[:3, :3]

    # Utiliser la fonction euler_from_matrix pour obtenir les angles d'Euler en XYZ
    # 'sxyz' order for XYZ Euler angles
    euler_angles = tf.euler_from_matrix(rotation_matrix, 'sxyz')

    return euler_angles


def matrix_from_angles(angle_x, angle_y, angle_z):
    """
    Crée une matrice de rotation à partir des angles d'Euler en XYZ.

    Args:
        angle_x (float): Angle d'Euler autour de l'axe X en radians.
        angle_y (float): Angle d'Euler autour de l'axe Y en radians.
        angle_z (float): Angle d'Euler autour de l'axe Z en radians.

    Returns:
        numpy.ndarray: Matrice de rotation 4x4.
    """
    rotation_matrix = np.eye(4)
    rotation_matrix[:3, :3] = tf.euler_matrix(
        angle_x, angle_y, angle_z, 'sxyz')[:3, :3]
    return rotation_matrix

############### Loading ####################

os.system('cls' if os.name == 'nt' else 'clear')

# Charger le model 3D

NAME_MODEL_3D = "data_exemple/FleurDeLisColored.ply"
NAME = "data_exemple/debug"

# NAME_MODEL_3D = "labo_biologie/2eme_semaine/foie_V_couleurs_h.ply"
# NAME = "labo_biologie/2eme_semaine/_foie_deuxieme_jour_dedos__Thibaud0"

# Marche bien (ne pas changer les paramètres) :
# Globalement si ça ne marche pas c'est juste parce que l'acquisition a merdé

# _foie_deuxieme_jour__Thibaud0 (marche trop bien)
# _foie_deuxieme_jour__Thibaud1 (marche trop bien)
# _foie_deuxieme_jour__Thibaud2 (marche trop bien)
# _foie_deuxieme_jour__Thibaud3 (marche trop bien)
# _foie_deuxieme_jour__Thibaud4 (marche trop bien)
# _foie_deuxieme_jour__Thibaud5 (marche trop bien)
# _foie_deuxieme_jour__Thibaud6 (marche trop bien)
# _foie_deuxieme_jour__Thibaud14 (marche trop bien)
# _foie_deuxieme_jour__Thibaud16 (marche ok)
# _foie_deuxieme_jour__Thibaud20 (marche trop bien)

# _foie_deuxieme_jour_dedos__Thibaud0 (marche trop bien)
# _foie_deuxieme_jour_dedos__Thibaud1 (marche trop bien)
# _foie_deuxieme_jour_dedos__Thibaud3 (marche trop bien)
# _foie_deuxieme_jour_dedos__Thibaud4 (marche trop bien)
# _foie_deuxieme_jour_dedos__Thibaud9 (marche trop bien)
# _foie_deuxieme_jour_dedos__Thibaud10 (marche trop bien)

###########################################################

################### Acquisition ###########################

print("Acquisition en cours...")

NAME_PC = NAME + '.ply'
COLOR_IMAGE_NAME = NAME + '.png'

# Récupération du nuage de POINTS en utilisant la Realsense
# Appeler une fonction d'acquisition pour récupérer le nuage de POINTS et les COULEURS

# Version Tinhinane (penser à également décommenter la partie pour la projection)

# aq.save_ply_from_realsense_with_interface(NAME_PC, COLOR_IMAGE_NAME)

# Version Thibaud (penser à également décommenter la partie pour la projection)

# aq.save_ply_from_realsense(NAME_PC,COLOR_IMAGE_NAME)

COLOR_IMAGE = cv2.imread(COLOR_IMAGE_NAME)
POINTS, COULEURS = pp.get_points_and_colors_of_ply(NAME_PC)

TABLEAU_INDICE = [i for i in range(len(POINTS))]

print("Acquisition terminée...")
###########################################################

##################### Selectionner Zone ####################

print("Selectionnez la zone à traiter :")

# Fonctionne uniquement avec la version de Thibaud + doit raccorder aux restes du code

POINTS_CROP, COULEURS_CROP, TABLEAU_INDICE_CROP = pa.crop_pc_from_zone_selection(POINTS,COULEURS,(640,480),TABLEAU_INDICE)

############################################################

###################### Masquage ###########################

print("Determination du masque hsv :")

# Détermination du masque

MASK_HSV = pi.get_hsv_mask_with_sliders(COLOR_IMAGE_NAME)

# Application du masque

POINTS_FILTRES_HSV, COULEURS_FILTRES_HSV, TABLEAU_INDICE_HSV = pa.apply_hsv_mask_to_pc(POINTS_CROP,COULEURS_CROP,MASK_HSV,TABLEAU_INDICE_CROP)

###########################################################

####################### Filtrage Bruit #####################

print("Supression du bruit de la caméra :")
print("Jouez avec le curseur pour augmenter le rayon de suppresion (centre = centre de masse) ")

NAME_BRUIT = NAME+'_filtre_bruit.ply'

POINT_FILRE_BRUIT, _, TABLEAU_INDICE_FILTRE_BRUIT = pa.filter_array_with_sphere_on_barycentre_with_interface(POINTS_FILTRES_HSV, COULEURS_FILTRES_HSV, TABLEAU_INDICE_HSV)
pp.save_ply_file(NAME_BRUIT,POINT_FILRE_BRUIT)

###########################################################

######################### Redimensionnement du modèle 3D ##################################

print("Redmimensionnement en cours...")

NAME_3D = NAME + "_model_3D"
MODEL_3D_RESIZED_NAME = NAME_3D + '_resized.ply'

# Pour le resize pas auto

# Application de redimensionnement
# SCALING_FACTOR = 0.00099 #0.0011
# pp.resize_ply_with_scaling_factor(NAME_MODEL_3D, MODEL_3D_RESIZED_NAME,SCALING_FACTOR)

# Pour le resize auto
NAME_MODEL_3D_REDUIT_DENSITE = NAME_MODEL_3D
NUAGE_DE_POINTS_TROP_GROS = True

# On divise le nombre de point par deux jusqu'à ce que ce soit suffisant pour l'algo de resize auto
while NUAGE_DE_POINTS_TROP_GROS:
    try:
        # Call the function to perform automatic resizing
        pp.resize_ply_to_another_one(NAME_MODEL_3D_REDUIT_DENSITE,NAME_BRUIT,
                       MODEL_3D_RESIZED_NAME)
        NUAGE_DE_POINTS_TROP_GROS = False
    except Exception as e:
        # Code à exécuter en cas d'erreur
        print(
            "Trop de points dans le nuage de point pour la fonction de resize automatique")
        print("on re-essaie en divisant le nombre de points du nuage par deux !")
        NAME_MODEL_3D_REDUIT_DENSITE = NAME_3D + "_reduit_densite.ply"
        pp.reduce_density_of_ply(NAME_MODEL_3D, NAME_MODEL_3D_REDUIT_DENSITE, 0.5)
        NAME_MODEL_3D = NAME_MODEL_3D_REDUIT_DENSITE

print("Redmimensionnement terminé")

###########################################################

################## Repositionnment du repère de la caméra dans celui de l'objet ####################

# # Application de repositionnement
PC_REPOSED_NAME = NAME + '_reposed.ply'
pp.centering_ply_on_mean_points(NAME_BRUIT, PC_REPOSED_NAME)
translation_vector = pp.get_mean_point_of_ply(NAME_BRUIT)
translation_vector[0] = translation_vector[0]
translation_vector[1] = translation_vector[1]
translation_vector[2] = translation_vector[2]

Mt = tm.translation_matrix(translation_vector)  # Matrice de translation
# print("Matrice de translation:",Mt)

print("Repositionnement terminé")

###########################################################

################ Remise en place du modèle 3D #############

# Que pour le foie

# On inverse suivant y le sens du modèle 3D (parce que il n'est pas dans le bon sens)
# En théorie on a pas beoisn de le faire si les pré-rotation parcourent l'ensemble des rotation possible
# (Ii pas le cas car tests (ie je veux que ça aille plus vite)))

angle = np.radians(180)
Mat_y = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [
                   0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])

# On récupère les points de notre modèle 3D et on applique les rotations et translations
model_3D_resized_name_points, model_3D_resized_name_coulors = pp.get_points_and_colors_of_ply(
    MODEL_3D_RESIZED_NAME)
# On met au bon format les POINTS (on rajoute une coordonnée de 1)
model_3D_resized_name_points = np.column_stack((model_3D_resized_name_points, np.ones(len(
    model_3D_resized_name_points))))

M = Mat_y

model_3D_resized_name_points = [M @ p for p in model_3D_resized_name_points]

pp.save_ply_file(MODEL_3D_RESIZED_NAME, model_3D_resized_name_points,model_3D_resized_name_coulors)

###########################################################

################ Matrice de pré-rotation ###################

print("On cherche la bonne pré-rotation à appliquer : ")

POINTS_MODEL_3D_RESIZED_NAME,_=pp.get_points_and_colors_of_ply(MODEL_3D_RESIZED_NAME)
POINTS_PC_REPOSED_NAME,_=pp.get_points_and_colors_of_ply(PC_REPOSED_NAME)
M_icp_1 = cp.find_the_best_pre_rotation(POINTS_MODEL_3D_RESIZED_NAME, POINTS_PC_REPOSED_NAME)
MODEL_3D_AFTER_PRE_ROTATIONS = NAME + "_after_pre_rotations.ply"
p, c = pp.get_points_and_colors_of_ply(MODEL_3D_RESIZED_NAME)
source_rotated = [np.dot(point, M_icp_1) for point in p]
pp.save_ply_file(MODEL_3D_AFTER_PRE_ROTATIONS,source_rotated)

M_icp_1 = np.hstack((M_icp_1, np.array([[0], [0], [0]])))
M_icp_1 = np.vstack((M_icp_1, np.array([0, 0, 0, 1])))

M_ICP_1_INV = np.linalg.inv(M_icp_1)

print("Pré-rotation trouvée")

# ###########################################################

# ###################### Matrice ICP #########################

print("Calcul de l'ICP :")
# Pour la version avec pré-rotation
p1,_=pp.get_points_and_colors_of_ply(MODEL_3D_AFTER_PRE_ROTATIONS)
p2,_=pp.get_points_and_colors_of_ply(PC_REPOSED_NAME)
M_icp_2, _ = cp.icp(p1, p2)
# M_icp_2, _=cp.run_icp(MODEL_3D_RESIZED_NAME,PC_REPOSED_NAME) # Pour la version sans pré-rotation
# print("M_icp :",M_icp_2)

# On ajuste la matrice d'ICP dans le repère de la caméra
angles_ICP2 = transformation_matrix_to_euler_xyz(M_icp_2)
print("Voici les angles de l'ICP : ", angles_ICP2)

x = -angles_ICP2[0]
y = angles_ICP2[1]
z = -angles_ICP2[2]

# Important de calculer l'inverse
# Parce que nous on veut faire bouger le modèle de CAO sur le nuage de points (et pas l'inverse !)
M_icp_2_inv = np.linalg.inv(matrix_from_angles(x, y, z))

# ###########################################################

# ################# Affiche et exporte Thibaud version 1 (à supprimer ?) #######################

# L'idée dans cette version est de ne pas faire de projection
# mais plutot d'utiliser le nuage de point initialement capturé par la caméra :
# On va déterminer les nouvelles coordonées de notre objet 3D
# et chercher les POINTS correspondant dans le nuage de la caméra.
# On viendra alors modifier la couleur de ces POINTS

# On récupère les POINTS et les COULEURS de notre nuage de POINTS (et on les convertit au bon format)
points_acquisition_originale, couleurs_acquisition_originale = pp.get_points_and_colors_of_ply(
    NAME_PC)
points_acquisition_originale = [tuple(row)
                                for row in points_acquisition_originale]
points_acquisition_originale = np.array([(float(x), float(y), float(z)) for (
    x, y, z) in points_acquisition_originale], dtype=np.float64)
couleurs_acquisition_originale = couleurs_acquisition_originale.astype(int)

# On récupère les POINTS de notre modèle 3D et on applique les rotations et translations
model_3D_resized_name_points, model_3D_resized_name_coulors = pp.get_points_and_colors_of_ply(
    MODEL_3D_RESIZED_NAME)
# On met au bon format les POINTS (on rajoute une coordonnée de 1)
model_3D_resized_name_points = np.column_stack((model_3D_resized_name_points, np.ones(len(
    model_3D_resized_name_points))))

M = Mt @ M_ICP_1_INV @ M_icp_2_inv  # Matrice de "projection"

NAME_TRANSFORMED_MODEL = NAME + "_transformed_3D_model.ply"

model_3D_resized_name_points = [M @ p for p in model_3D_resized_name_points]

pp.save_ply_file(NAME_TRANSFORMED_MODEL,model_3D_resized_name_points,model_3D_resized_name_coulors)

model_3D_POINTS, _ = pp.get_points_and_colors_of_ply(NAME_TRANSFORMED_MODEL)

# On cherche maintenant à superposer les deux nuages de POINTS
# Pour cela on utilise des arbres KD

tree = cKDTree(points_acquisition_originale)

# Liste pour stocker les indices des POINTS les plus proches dans le second nuage
indices_des_plus_proches = []

# Pour chaque point dans le premier nuage
for point in model_3D_POINTS:
    # On recherche le point le plus proche dans le second nuage
    distance, indice_plus_proche = tree.query(point)

    if True:  # distance < 0.003:
        # On concerve l'indice du point le plus proche
        indices_des_plus_proches.append(indice_plus_proche)

# On modifie les COULEURS des POINTS trouvés dans l'étape précédente
# c'est la ou se situe notre objet donc on va l'indiquer avec la couleur de l'objet en question
i = 0
for indice in indices_des_plus_proches:
    if len(model_3D_resized_name_coulors) == 0:
        # Bleu (couleur dans le cas ou le modèle 3D est sans couleur)
        couleur_objet = np.array([0, 0, 255])
    else:
        couleur_objet = model_3D_resized_name_coulors[i]
        i += 1
    couleurs_acquisition_originale[indice] = couleur_objet
    # On fait un peu autour pour que ce soit plus visible
    couleurs_acquisition_originale[indice+1] = couleur_objet
    couleurs_acquisition_originale[indice-1] = couleur_objet
    couleurs_acquisition_originale[indice+640] = couleur_objet
    couleurs_acquisition_originale[indice+640+1] = couleur_objet
    couleurs_acquisition_originale[indice+640-1] = couleur_objet
    couleurs_acquisition_originale[indice-640-1] = couleur_objet
    couleurs_acquisition_originale[indice-640] = couleur_objet
    couleurs_acquisition_originale[indice-640+1] = couleur_objet

# On enregistre
pi.save_image_from_array(couleurs_acquisition_originale, NAME + "projection_Thibaud.png", (640,480))

### ????
POINTS_PROJECTION_V1, COULEURS_PORJECTION_V1, _ = pa.filter_array_with_sphere_on_barycentre(points_acquisition_originale,0.5, couleurs_acquisition_originale)
### ???

pp.save_ply_file(NAME+"_projection.ply",POINTS_PROJECTION_V1,COULEURS_PORJECTION_V1)

color_image1 = cv2.imread(NAME + "projection_Thibaud.png")

# On affiche
print("Résultat final !")
while True:
    cv2.imshow("Affichage_Thibaud_V1", color_image1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

#####################################################################

################# Affiche et exporte Thibaud version affinée #######################

# L'idée dans cette version est de ne pas faire de projection
# mais plutot d'utiliser le nuage de point initialement capturé par la caméra :
# On va déterminer les nouvelles coordonées de notre objet 3D
# et chercher points correspondants dans le nuage de la caméra (une fois le filtrage fini).
# On viendra alors modifier la couleur des POINTS correspondants dans l'acquisition initiale

# On récupère les points de notre nuage de en sortie de nos étapes de filtrages
# (et on les convertit au bon format)
points_full_filtres, _ = pp.get_points_and_colors_of_ply(NAME_BRUIT)
points_full_filtres = [tuple(row) for row in points_full_filtres]
points_full_filtres = np.array([(float(x), float(y), float(z)) for (
    x, y, z) in points_full_filtres], dtype=np.float64)

# On récupère les points de notre modèle 3D et on applique les rotations et translations
model_3D_resized_name_points, model_3D_resized_name_coulors = pp.get_points_and_colors_of_ply(
    MODEL_3D_RESIZED_NAME)
# On met au bon format les POINTS (on rajoute une coordonnée de 1)
model_3D_resized_name_points = np.column_stack((model_3D_resized_name_points, np.ones(len(
    model_3D_resized_name_points))))

M = Mt @ M_ICP_1_INV @ M_icp_2_inv  # Matrice de "projection"

NAME_TRANSFORMED_MODEL = NAME + "_transformed_3D_model.ply"

model_3D_resized_name_points = [M @ p for p in model_3D_resized_name_points]

pp.save_ply_file(NAME_TRANSFORMED_MODEL,model_3D_resized_name_points,model_3D_resized_name_coulors)

model_3D_POINTS, _ = pp.get_points_and_colors_of_ply(NAME_TRANSFORMED_MODEL)

# On cherche maintenant à superposer les deux nuages (model_3D_POINTS sur points_full_filtres)
# Pour cela on utilise des arbres KD
tree = cKDTree(points_full_filtres)

# Liste pour stocker les positions (indices) des POINTS du deuxième nuage (points_full_filtres)
# les plus proches de ceux dans le premier nuage (model_3D_POINTS)
# model_3D_POINTS[i]=points_full_filtres[indices_des_plus_proches[i]]
indices_des_plus_proches = []

# Pour chaque point dans le premier nuage (model_3D_POINTS)
for point in model_3D_POINTS:
    # On recherche le point le plus proche dans le second nuage
    distance, indice_plus_proche = tree.query(point)

    if True:  # distance < 0.003:
        # On conserve l'indice du point le plus proche
        indices_des_plus_proches.append(indice_plus_proche)

# On récupère la position (indices) des POINTS (points_full_filtres) dans le nuage de point initial
# TABLEAU_INDICE_FILTRE_BRUIT est le tableau des indices des position des POINTS filtrés
# dans le nuage de POINTS de l'acquisition initiale
indice_dans_pc_initial = [TABLEAU_INDICE_FILTRE_BRUIT[i]
                          for i in indices_des_plus_proches]
# On récupère les COULEURS de notre nuage de point initial (utile pour la projection)
points_acquisition_originale, couleurs_acquisition_originale = pp.get_points_and_colors_of_ply(
    NAME_PC)
couleurs_acquisition_originale = couleurs_acquisition_originale.astype(int)

# On fait les modifications de COULEURS de l'acquisition initiale
i = 0
for indice in indice_dans_pc_initial:
    if len(model_3D_resized_name_coulors) == 0:
        # Bleu (couleur dans le cas ou le modèle 3D est sans couleur)
        couleur_objet = np.array([0, 0, 255])
    else:
        couleur_objet = model_3D_resized_name_coulors[i]
        i += 1
    couleurs_acquisition_originale[indice] = couleur_objet
    # On fait un peu autour pour que ce soit plus visible
    couleurs_acquisition_originale[indice+1] = couleur_objet
    couleurs_acquisition_originale[indice-1] = couleur_objet
    couleurs_acquisition_originale[indice+640] = couleur_objet
    couleurs_acquisition_originale[indice+640+1] = couleur_objet
    couleurs_acquisition_originale[indice+640-1] = couleur_objet
    couleurs_acquisition_originale[indice-640-1] = couleur_objet
    couleurs_acquisition_originale[indice-640] = couleur_objet
    couleurs_acquisition_originale[indice-640+1] = couleur_objet


# On enregistre
pi.save_image_from_array(couleurs_acquisition_originale, NAME + "projection_Thibaud_affinee.png", (640,480))

### ????
POINTS_PROJECTION_V2, COULEURS_PORJECTION_V2, _ = pa.filter_array_with_sphere_on_barycentre(points_acquisition_originale,0.5, couleurs_acquisition_originale)
### ???

pp.save_ply_file(NAME+"_projection_affinee.ply",POINTS_PROJECTION_V2,COULEURS_PORJECTION_V2)


color_image1 = cv2.imread(NAME + "projection_Thibaud_affinee.png")

# On affiche
print("Résultat final !")
while True:
    cv2.imshow("Affichage_Thibaud_V2", color_image1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

#####################################################################

################# Affiche et exporte Tinhinane ######################

# ## Calcul des POINTS de projections

# Matrice de projection ==> Matrice intrinsèque * Matrice extrinsèque
# Matrice extrinsèque ==> ensemble des modifications (translations et rotations) à appliquer au modèle CAO

## Matrice de calibration (Matrice intrinsèque) ####

# calibration_matrix = rc.recover_matrix_calib()
# M_in = np.hstack((calibration_matrix, np.zeros((3, 1))))
# M_in = np.vstack((M_in, np.array([0, 0, 0, 1])))
# Matrice intrinsèquqe Tinhinane je pense à supprimer
M_in = np.array([[382.437, 0, 319.688, 0], [
                0, 382.437, 240.882, 0], [0, 0, 1, 0]])
# M_in = np.array([[423.84763, 0, 319.688, 0], [0,423.84763, 240.97697, 0], [0, 0, 1, 0]])  # Matrice intrinsèquqe Tinhinane remaster à la main je pense à supprimer

#### Calcul final de la projection ####

POINTS_MODEL_3D ,COLORS_MODEL_3D = pp.get_points_and_colors_of_ply(MODEL_3D_RESIZED_NAME)

PROJECTION = M_in @ Mt @ M_ICP_1_INV @ M_icp_2_inv

if len(COLORS_MODEL_3D) == 0:
    COLORS_MODEL_3D = np.asarray(
        [[0., 0., 1.] for i in range(len(np.asarray(POINTS_MODEL_3D)))])

while True:
    # Appel à la fonction permettant de projeter l'objet 3D avec ses COULEURS spécifiques
    frame_apres = proj.project_and_display_Tinhinane(
        COLOR_IMAGE, POINTS_MODEL_3D, COLORS_MODEL_3D, PROJECTION)
    cv2.imshow("Affichage_Tinhinane", frame_apres)
    cv2.imwrite(NAME + "projection_Tinhinane.png", frame_apres)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
