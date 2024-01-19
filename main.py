import os

import numpy as np
import cv2
import time

from functions import icp as cp
from functions import project_and_display as proj
from functions import transformations as tf

from realsense import acquisition_realsense as aq
from realsense import calibration_matrix_realsense as rc
from realsense.functions.utils import array as array
from realsense.functions import processing_ply as ply
from realsense.functions import processing_point_cloud as pc
from realsense.functions import processing_pixel_list as pixels
from realsense.functions import processing_img as img
from realsense.functions import previsualisation_application_function as Tk

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

# NAME_MODEL_3D = "data_exemple/FleurDeLisColored.ply"
# NAME = "data_exemple/debug"

NAME_MODEL_3D = "labo_biologie/2eme_semaine/foie_V_couleurs_h.ply"
NAME = "labo_biologie/2eme_semaine/_foie_deuxieme_jour_dedos__Thibaud10"

# NAME_MODEL_3D = "labo_biologie/3eme_semaine/poulet_2_3D_model.ply"
# NAME = "labo_biologie/3eme_semaine/2_poulet_12"

POINTS_MODEL_3D,COLORS_MODEL_3D = ply.get_points_and_colors(NAME_MODEL_3D)

# Nom des fichiers à enregistrer

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

# pipeline = aq.init_realsense(640,480) # On fait une acquisition
# aq.get_points_and_colors_from_realsense(pipeline) # On fait une acquisition
# POINTS, COULEURS = aq.get_points_and_colors_from_realsense(pipeline,COLOR_IMAGE_NAME) # On fait une acquisition
POINTS, COULEURS =  ply.get_points_and_colors(NAME_PC) # Ou alors, au lieu de faire une acquisition, on récupère les points d'un ply existant

COLOR_IMAGE = img.load(COLOR_IMAGE_NAME)

TABLEAU_INDICE = np.array([i for i in range(len(POINTS))])

print("Acquisition terminée...")

###########################################################

##################### Selectionner Zone ####################

print("Selectionnez la zone à traiter :")

# Fonctionne uniquement avec la version de Thibaud + doit raccorder aux restes du code

POINTS_CROP, COULEURS_CROP, TABLEAU_INDICE_CROP,new_shape = pc.crop_from_zone_selection(points=POINTS,colors=COULEURS,shape=(640,480),tableau_indice=TABLEAU_INDICE)

############################################################


###################### Masquage ###########################

print("Determination du masque hsv :")

# Détermination du masque

MASK_HSV = pixels.get_hsv_mask_with_sliders(array.line_to_3Darray(COULEURS_CROP,new_shape))

# Application du masque

POINTS_FILTRES_HSV, COULEURS_FILTRES_HSV, TABLEAU_INDICE_HSV = pc.apply_hsv_mask(POINTS_CROP,COULEURS_CROP,MASK_HSV,new_shape,TABLEAU_INDICE_CROP)

###########################################################

####################### Filtrage Bruit #####################

print("Supression du bruit de la caméra :")
print("Jouez avec le curseur pour augmenter le rayon de suppresion (centre = centre de masse) ")

radius = Tk.get_parameter_using_preview(POINTS_FILTRES_HSV,pc.filter_with_sphere_on_barycentre,"Radius")

POINT_FILRE_BRUIT, COULEUR_FILRE_BRUIT, TABLEAU_INDICE_FILTRE_BRUIT = pc.filter_with_sphere_on_barycentre(POINTS_FILTRES_HSV,radius, COULEURS_FILTRES_HSV, TABLEAU_INDICE_HSV)

###########################################################

######################### Redimensionnement du modèle 3D ##################################

print("Redmimensionnement en cours...")

NUAGE_DE_POINTS_TROP_GROS = True

# On divise le nombre de point par deux jusqu'à ce que ce soit suffisant pour l'algo de resize auto
while NUAGE_DE_POINTS_TROP_GROS:
    try:
        # Call the function to perform automatic resizing
        POINTS_MODEL_3D_RESIZED = pc.resize_point_cloud_to_another_one(POINTS_MODEL_3D,POINT_FILRE_BRUIT)
        NUAGE_DE_POINTS_TROP_GROS = False
    except Exception as e:
        # Code à exécuter en cas d'erreur
        print(
            "Trop de points dans le nuage de point pour la fonction de resize automatique")
        print("on re-essaie en divisant le nombre de points du nuage par deux !")
        POINTS_MODEL_3D, COLORS_MODEL_3D = pc.reduce_density(POINTS_MODEL_3D,0.5,COLORS_MODEL_3D)

print("Redmimensionnement terminé")

###########################################################

################## Repositionnment du repère de la caméra dans celui de l'objet ####################

# # Application de repositionnement
POINTS_REPOSED = pc.centering_on_mean_points(POINT_FILRE_BRUIT)
translation_vector = pc.get_mean_point(POINT_FILRE_BRUIT)
translation_vector[0] = translation_vector[0]
translation_vector[1] = translation_vector[1]
translation_vector[2] = translation_vector[2]

Mt = tf.translation_matrix(translation_vector)  # Matrice de translation
# print("Matrice de translation:",Mt)

print("Repositionnement terminé")

###########################################################

################ Remise en place du modèle 3D #############

# Que pour les tests

# On inverse suivant y le sens du modèle 3D (parce que il n'est pas dans le bon sens)
# En théorie on a pas beoisn de le faire si les pré-rotation parcourent l'ensemble des rotation possible
# (Ii pas le cas car tests (ie je veux que ça aille plus vite)))

angle = np.radians(180)
Mat_y = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [
                   0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])

POINTS_MODEL_3D_RESIZED= np.array([(float(x), float(y), float(z)) for (
    x, y, z,t) in [Mat_y @ p for p in np.column_stack((POINTS_MODEL_3D_RESIZED, np.ones(len(
    POINTS_MODEL_3D_RESIZED))))]], dtype=np.float64)

###########################################################

################ Matrice de pré-rotation ###################

print("On cherche la bonne pré-rotation à appliquer : ")

POINTS_MODEL_3D_REDUCE_DENSITY,_ = pc.reduce_density(POINTS_MODEL_3D_RESIZED,0.05)

M_icp_1 = cp.find_the_best_pre_rotation_to_align_points(POINTS_MODEL_3D_REDUCE_DENSITY, POINTS_REPOSED,[0, 10, 10],[0, 10, 10],[-180, 180, 10]) # Utile avoir des résultats plus vite

M_icp_1 = np.hstack((M_icp_1, np.array([[0], [0], [0]])))
M_icp_1 = np.vstack((M_icp_1, np.array([0, 0, 0, 1])))

M_ICP_1_INV = np.linalg.inv(M_icp_1)

print("Pré-rotation trouvée")

###########################################################

###################### Matrice ICP #########################

print("Calcul de l'ICP :")

# Pour la version avec pré-rotation

MODEL_3D_POINTS_AFTER_PRE_ROTATION = np.array([(float(x), float(y), float(z)) for (
    x, y, z,t) in [M_ICP_1_INV @ p for p in np.column_stack((POINTS_MODEL_3D_RESIZED, np.ones(len(
    POINTS_MODEL_3D_RESIZED))))]], dtype=np.float64)
    
M_icp_2,_ = cp.find_transform_matrix_to_align_points_using_icp(MODEL_3D_POINTS_AFTER_PRE_ROTATION, POINTS_REPOSED)

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

# ################# Affiche et exporte using closest points identification #######################

M = Mt @ M_ICP_1_INV @ M_icp_2_inv  # Matrice de "projection"

COULEURS_PROJECTION_V1 = proj.project_3D_model_on_pc_using_closest_points_identification(POINTS_MODEL_3D_RESIZED,COLORS_MODEL_3D,POINTS,COULEURS,M)

# On enregistre
img.save(COULEURS_PROJECTION_V1, NAME + "_projection_closest_points.png", (480,640))

ply.save(NAME+"_projection_closest_points.ply",POINTS,COULEURS_PROJECTION_V1)


color_image1=img.load(NAME + "_projection_closest_points.png")

# On affiche
while True:
    cv2.imshow("Projection using closest points identification", color_image1[:, :, ::-1])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

#####################################################################

################# Affiche et exporte using indices identification #######################
COULEURS_PROJECTION_V2 = proj.project_3D_model_on_pc_using_closest_points_and_indices(POINTS_MODEL_3D_RESIZED,COLORS_MODEL_3D,POINT_FILRE_BRUIT,COULEURS,TABLEAU_INDICE_FILTRE_BRUIT,M)

# On enregistre
img.save(COULEURS_PROJECTION_V2, NAME + "_projection_using_indices.png", (480,640))

ply.save(NAME+"_projection_using_indices.ply",POINTS,COULEURS_PROJECTION_V2)

color_image2=img.load(NAME + "_projection_using_indices.png")

# On affiche
while True:
    cv2.imshow("Projection using indices identification", color_image2[:, :, ::-1])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

#####################################################################

################# Affiche et exporte using simple projection ######################

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

PROJECTION = M_in @ Mt @ M_ICP_1_INV @ M_icp_2_inv

if len(COLORS_MODEL_3D) == 0:
    COLORS_MODEL_3D = np.asarray(
        [[0., 0., 1.] for i in range(len(np.asarray(POINTS_MODEL_3D)))])

COLOR_IMAGE=cv2.imread(COLOR_IMAGE_NAME)

while True:
    # Appel à la fonction permettant de projeter l'objet 3D avec ses COULEURS spécifiques
    frame_apres = proj.project_3D_model_on_pc(
        COULEURS, POINTS_MODEL_3D_RESIZED, COLORS_MODEL_3D, PROJECTION)
    cv2.imshow("Projection of 3D model on pc", frame_apres)
    cv2.imwrite(NAME + "projection.png", frame_apres)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
