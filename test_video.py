import numpy as np
import cv2

from functions import icp as cp
from functions import project_and_display as proj
from functions import transformations as tf

from realsense import acquisition_realsense as aq
from realsense import calibration_matrix_realsense as rc
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


# Calcul de la première image

############### Loading ####################

# Charger le model 3D

NAME_MODEL_3D = "data_exemple/FleurDeLisColored.ply"
NAME = "data_exemple/debug"

POINTS_MODEL_3D,COLORS_MODEL_3D = ply.get_points_and_colors(NAME_MODEL_3D)

###########################################################

################### Acquisition ###########################

print("Acquisition en cours...")

NAME_PC = NAME + '.ply'
COLOR_IMAGE_NAME = NAME + '.png'

# Récupération du nuage de points en utilisant la Realsense

pipeline = aq.init_realsense(640,480)

POINTS, COLORS = aq.get_points_and_colors_from_realsense(pipeline) # On fait une acquisition

print("Acquisition terminée...")

###########################################################

##################### Selectionner Zone ####################

print("Selectionnez la zone à traiter :")

# Fonctionne uniquement avec la version de Thibaud + doit raccorder aux restes du code

POINTS_CROP, COULEURS_CROP,_ = pc.crop_from_zone_selection(POINTS,COLORS)

############################################################

###################### Masquage ###########################

print("Determination du masque hsv :")

# Détermination du masque

MASK_HSV = pixels.get_hsv_mask_with_sliders(COLORS)

# Application du masque

POINTS_FILTRES_HSV, COULEURS_FILTRES_HSV,_ = pc.apply_hsv_mask(POINTS_CROP,COULEURS_CROP,MASK_HSV)

###########################################################

####################### Filtrage Bruit #####################

print("Supression du bruit de la caméra :")
print("Jouez avec le curseur pour augmenter le rayon de suppresion (centre = centre de masse) ")

radius = Tk.get_parameter_using_preview(POINTS_FILTRES_HSV,pc.filter_with_sphere_on_barycentre,"Radius")

POINT_FILRE_BRUIT, COULEUR_FILRE_BRUIT, _ = pc.filter_with_sphere_on_barycentre(POINTS_FILTRES_HSV,radius, COULEURS_FILTRES_HSV)

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

################ Matrice de pré-rotation ###################

print("On cherche la bonne pré-rotation à appliquer : ")

M_icp_1 = cp.find_the_best_pre_rotation_to_align_points(POINTS_MODEL_3D_RESIZED, POINTS_REPOSED)

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



# Calcul de toute les autres

while True:
       
    points,colors_image=aq.get_points_and_colors_from_realsense(pipeline)
    
    cv2.imshow("Color Image", colors_image[:, :, ::-1])
    
    if cv2.waitKey(1) & 0xFF == 27:
        break

cv2.destroyAllWindows()