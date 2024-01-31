import numpy as np
import cv2
import time

from functions import icp as cp
from functions import project_and_display as proj
from functions import transformations as tf

from Python_3D_Toolbox_for_Realsense import acquisition_realsense as aq
from Python_3D_Toolbox_for_Realsense import calibration_matrix_realsense as rc
from Python_3D_Toolbox_for_Realsense.functions import processing_ply as ply
from Python_3D_Toolbox_for_Realsense.functions import processing_point_cloud as pc
from Python_3D_Toolbox_for_Realsense.functions import processing_pixel_list as pixels
from Python_3D_Toolbox_for_Realsense.functions import processing_img as img
from Python_3D_Toolbox_for_Realsense.functions import previsualisation_application_function as Tk
from Python_3D_Toolbox_for_Realsense.functions.utils import array as array

# Calcul de la première image

############### Loading ####################

# Charger le model 3D

NAME_MODEL_3D = "data_exemple/FleurDeLisColoredReduceDensity.ply"
NAME = "data_exemple/debug"

POINTS_MODEL_3D,COLORS_MODEL_3D = ply.get_points_and_colors(NAME_MODEL_3D)

###########################################################

################### Acquisition ###########################

NAME_PC = NAME + '.ply'
COLOR_IMAGE_NAME = NAME + '.png'

# Récupération du nuage de points en utilisant la Realsense

pipeline = aq.init_realsense(640,480)
# Bon je ne sais pas pourquoi il faut que je le fasse plusiquer fois comme ça mais si je ne fais pas, l'image affichée n'est pas la même tout le long du truc ...
POINTS, COLORS = aq.get_points_and_colors_from_realsense(pipeline) # On fait une acquisition
pixels.display(COLORS,"Display")
POINTS, COLORS = aq.get_points_and_colors_from_realsense(pipeline) # On fait une acquisition
pixels.display(COLORS,"Display")
POINTS, COLORS = aq.get_points_and_colors_from_realsense(pipeline) # On fait une acquisition
pixels.display(COLORS,"Display")

###########################################################

###################### Masquage ###########################

# Détermination du masque

MASK_HSV = pixels.get_hsv_mask_with_sliders(COLORS,(480,640))

# Application du masque

POINTS_FILTRES_HSV, COULEURS_FILTRES_HSV,_ = pc.apply_hsv_mask(POINTS,COLORS,MASK_HSV,(480,640))

###########################################################

####################### Filtrage Bruit #####################

radius = Tk.get_parameter_using_preview(POINTS_FILTRES_HSV,pc.filter_with_sphere_on_barycentre,"Radius")

POINT_FILRE_BRUIT, COULEUR_FILRE_BRUIT, _ = pc.filter_with_sphere_on_barycentre(POINTS_FILTRES_HSV,radius, COULEURS_FILTRES_HSV)

# Eventuellement pour gagner en vitesse
density = 1
while not(1500<len(POINT_FILRE_BRUIT)*density<2000):
    density = density - 0.1
    if density < 0:
        raise ValueError("Error density")

POINT_FILRE_BRUIT,COULEUR_FILRE_BRUIT = pc.reduce_density(POINT_FILRE_BRUIT,density,COULEUR_FILRE_BRUIT)

###########################################################

######################### Redimensionnement du modèle 3D ##################################

NUAGE_DE_POINTS_TROP_GROS = True

# On divise le nombre de point par deux jusqu'à ce que ce soit suffisant pour l'algo de resize auto
while NUAGE_DE_POINTS_TROP_GROS:
    try:
        # Call the function to perform automatic resizing
        POINTS_MODEL_3D_RESIZED = pc.resize_point_cloud_to_another_one(POINTS_MODEL_3D,POINT_FILRE_BRUIT)
        NUAGE_DE_POINTS_TROP_GROS = False
    except Exception as e:
        # Code à exécuter en cas d'erreur
        POINTS_MODEL_3D, COLORS_MODEL_3D = pc.reduce_density(POINTS_MODEL_3D,0.5,COLORS_MODEL_3D)

###########################################################

################## Repositionnment du repère de la caméra dans celui de l'objet ####################

# # Application de repositionnement
# POINTS_REPOSED = pc.centering_on_mean_points(POINT_FILRE_BRUIT)
# translation_vector = pc.get_mean_point(POINT_FILRE_BRUIT)

POINTS_REPOSED = pc.centers_points_on_geometry(POINT_FILRE_BRUIT)
translation_vector = pc.get_center_geometry(POINT_FILRE_BRUIT)

translation_vector[0] = translation_vector[0]
translation_vector[1] = translation_vector[1]
translation_vector[2] = translation_vector[2]

Mt = tf.translation_matrix(translation_vector)  # Matrice de translation

###########################################################

################ Matrice de pré-rotation ###################

M_icp_1,best_angle = cp.find_the_best_pre_rotation_to_align_points(POINTS_MODEL_3D_RESIZED, POINTS_REPOSED,[0, 0, 10],[0, 0, 10],[-180, 180, 20])

M_icp_1 = np.hstack((M_icp_1, np.array([[0], [0], [0]])))
M_icp_1 = np.vstack((M_icp_1, np.array([0, 0, 0, 1])))

M_ICP_1_INV = np.linalg.inv(M_icp_1)

###########################################################

###################### Matrice ICP #########################

# Pour la version avec pré-rotation

MODEL_3D_POINTS_AFTER_PRE_ROTATION = np.array([(float(x), float(y), float(z)) for (
    x, y, z,t) in [M_ICP_1_INV @ p for p in np.column_stack((POINTS_MODEL_3D_RESIZED, np.ones(len(
    POINTS_MODEL_3D_RESIZED))))]], dtype=np.float64)

M_icp_2,_ = cp.find_transform_matrix_to_align_points_using_icp(MODEL_3D_POINTS_AFTER_PRE_ROTATION, POINTS_REPOSED)

# On ajuste la matrice d'ICP dans le repère de la caméra
angles_ICP2 = tf.transformation_matrix_to_euler_xyz(M_icp_2)

x = -angles_ICP2[0]
y = angles_ICP2[1]
z = -angles_ICP2[2]

# Important de calculer l'inverse
# Parce que nous on veut faire bouger le modèle de CAO sur le nuage de points (et pas l'inverse !)
M_icp_2_inv = np.linalg.inv(tf.matrix_from_angles(x, y, z))

# Calcul de toute les autres

################# Affiche et exporte using closest points identification #######################

M = Mt @ M_ICP_1_INV @ M_icp_2_inv  # Matrice de "projection"

COULEURS_PROJECTION_V1 = proj.project_3D_model_on_pc_using_closest_points_identification(POINTS_MODEL_3D_RESIZED,COLORS_MODEL_3D,np.array(POINTS),COLORS,M)

COULEURS_PROJECTION_V1 = array.line_to_2Darray(COULEURS_PROJECTION_V1, (480,640)).astype(np.uint8)

# On affiche
while True:
    cv2.imshow("Projection using closest points identification", COULEURS_PROJECTION_V1[:, :, ::-1])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()


#####################################################################

point_model_3D_resize = np.copy(POINTS_MODEL_3D_RESIZED)

# # Paramètres de la vidéo
# largeur, hauteur = 640, 480
# fps = 30

# # Créer un objet VideoWriter
# video_writer = cv2.VideoWriter('test.mp4', cv2.VideoWriter_fourcc(*'XVID'), fps, (largeur, hauteur))

while True:
    
    temps_debut = time.time()

    # Acquisition   
    
    points,colors=aq.get_points_and_colors_from_realsense(pipeline)
    # Application du masque hsv
    
    points_filtres,colors_filtres,_ = pc.apply_hsv_mask(points,colors,MASK_HSV,(480,640))

    # Filtrage bruit
    
    points_filtres_sphere, colors_filtres_sphere,_ = pc.filter_with_sphere_on_barycentre(points_filtres,radius, colors_filtres)
    
    if not(1500<len(points_filtres_sphere)*density<2000):
        points_filtres_sphere, colors_filtres_sphere = pc.reduce_density(points_filtres_sphere,density,colors_filtres_sphere)

    # Repositionnement
    
    # points_reposed = pc.centering_on_mean_points(points_filtres_sphere)
    # translation_vector = pc.get_mean_point(points_filtres_sphere)
    
    points_reposed = pc.centers_points_on_geometry(points_filtres_sphere)
    translation_vector = pc.get_center_geometry(points_filtres_sphere)
    
    translation_vector[0] = translation_vector[0]
    translation_vector[1] = translation_vector[1]
    translation_vector[2] = translation_vector[2]

    Mt = tf.translation_matrix(translation_vector) 

    # Pré-rotation
    
    M_icp_1,best_angle = cp.find_the_best_pre_rotation_to_align_points(POINTS_MODEL_3D_RESIZED, points_reposed,[best_angle[0]-20, best_angle[0]+20, 20],[best_angle[1]-20, best_angle[1]+20, 20],[best_angle[2]-20, best_angle[2]+20, 20])
    # M_icp_1,best_angle = cp.find_the_best_pre_rotation_to_align_points(POINTS_MODEL_3D_RESIZED, points_reposed,[0, 0, 10],[0, 0, 10],[-180, 180, 20])
    M_icp_1 = np.hstack((M_icp_1, np.array([[0], [0], [0]])))
    M_icp_1 = np.vstack((M_icp_1, np.array([0, 0, 0, 1])))

    M_ICP_1_INV = np.linalg.inv(M_icp_1)
    
    # ICP
    
    MODEL_3D_POINTS_AFTER_PRE_ROTATION = np.array([(float(x), float(y), float(z)) for (
    x, y, z,t) in [M_ICP_1_INV @ p for p in np.column_stack((point_model_3D_resize, np.ones(len(
    point_model_3D_resize))))]], dtype=np.float64)
    
    M_icp_2,_ = cp.find_transform_matrix_to_align_points_using_icp(MODEL_3D_POINTS_AFTER_PRE_ROTATION, points_reposed)

    angles_ICP2 = tf.transformation_matrix_to_euler_xyz(M_icp_2)

    x = -angles_ICP2[0]
    y = angles_ICP2[1]
    z = -angles_ICP2[2]

    M_icp_2_inv = np.linalg.inv(tf.matrix_from_angles(x, y, z))
       
    # Calcul de projection 
        
    M_in = np.array([[382.437, 0, 319.688, 0], [
                0, 382.437, 240.882, 0], [0, 0, 1, 0]])
    
    M_projection = M_in @ Mt @ M_ICP_1_INV @ M_icp_2_inv  # Matrice de "projection"

    colors_image = proj.project_3D_model_on_pc(colors, point_model_3D_resize, COLORS_MODEL_3D, M_projection)

    # Affichage 
     
    cv2.imshow("Color Image", colors_image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        pipeline.stop()
        break
               
    # video_writer.write(colors_image)
    temps_fin = time.time()
    temps_execution = temps_fin - temps_debut
    print(f"Temps d'exécution : {temps_execution} secondes")
    
cv2.destroyAllWindows()