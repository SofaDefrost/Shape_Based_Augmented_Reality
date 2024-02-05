import numpy as np
import cv2
import time

from functions import icp as cp
from functions import project_and_display as proj
from functions import matrix_operations as tf

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

NAME_MODEL_3D = "data_exemple/estomac_3D_model_reduced_density_colored.ply"
NAME = "data_exemple/test_estomac"

POINTS_MODEL_3D,COLORS_MODEL_3D = ply.get_points_and_colors(NAME_MODEL_3D)

if len(COLORS_MODEL_3D) == 0:
    COLORS_MODEL_3D = np.asarray(
        [[0., 0., 255.] for i in range(len(np.asarray(POINTS_MODEL_3D)))])

###########################################################

################### Acquisition ###########################

NAME_PC = NAME + '.ply'
COLOR_IMAGE_NAME = NAME + '.png'

# Récupération du nuage de points en utilisant la Realsense
size_acqui = (1280,720)
pipeline = aq.init_realsense(size_acqui[0],size_acqui[1])

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

MASK_HSV = pixels.get_hsv_mask_with_sliders(COLORS,size_acqui)

# Application du masque

POINTS_FILTRES_HSV, COULEURS_FILTRES_HSV,_ = pc.apply_hsv_mask(POINTS,COLORS,MASK_HSV,size_acqui)

###########################################################

####################### Filtrage Bruit #####################

radius = Tk.get_parameter_using_preview(POINTS_FILTRES_HSV,pc.filter_with_sphere_on_barycentre,"Radius")

POINT_FILRE_BRUIT, COULEUR_FILRE_BRUIT, _ = pc.filter_with_sphere_on_barycentre(POINTS_FILTRES_HSV,radius, COULEURS_FILTRES_HSV)

# Eventuellement pour gagner en vitesse

if (len(POINT_FILRE_BRUIT)>2000):
    POINT_FILRE_BRUIT,COULEUR_FILRE_BRUIT = pc.reduce_density(POINT_FILRE_BRUIT,2000/len(POINT_FILRE_BRUIT),COULEUR_FILRE_BRUIT)

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

M_icp_1,best_angle = cp.find_the_best_pre_rotation_to_align_points(POINTS_MODEL_3D_RESIZED, POINTS_REPOSED,[0, 0, 10],[0, 0, 10],[-180, 180, 10])

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

################# Affiche #######################

calibration_matrix = rc.recover_matrix_calib(size_acqui[0],size_acqui[1])
M_in = np.hstack((calibration_matrix, np.zeros((3, 1))))
M_in = np.vstack((M_in, np.array([0, 0, 0, 1])))

M = M_in @ Mt @ M_ICP_1_INV @ M_icp_2_inv  # Matrice de "projection"

COULEURS_PROJECTION_V1 = proj.project_3D_model_on_pc(COLORS, POINTS_MODEL_3D_RESIZED, COLORS_MODEL_3D, M,size_acqui)
 
while True:
    cv2.imshow("Color Image", COULEURS_PROJECTION_V1)  
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

#####################################################################

point_model_3D_resize = np.copy(POINTS_MODEL_3D_RESIZED)

# # Paramètres de la vidéo
# largeur, hauteur = size_acqui
# fps = 5

# # Créer un objet VideoWriter
# video_writer = cv2.VideoWriter('test.mp4', cv2.VideoWriter_fourcc(*'XVID'), fps, (largeur, hauteur))

pipeline = aq.init_realsense(size_acqui[0],size_acqui[1])

while True:
    
    temps_debut = time.time()

    # Acquisition   
    temps_debut = time.time()
    points,colors=aq.get_points_and_colors_from_realsense(pipeline)
    temps_fin = time.time()
    temps_execution = temps_fin - temps_debut
    print(f"Temps acquisition : {temps_execution} secondes")
    #
    # Trouver un moyen de faire fonctionner un truc comme ça demain ou alors obj : est accelèrer les deux masques (trouver éventuellement une autre manière de comment faire) 
    # if (len(points)>100000):
    #     points, colors = pc.reduce_density(points,100000/len(points),colors)
    # Application du masque hsv
    temps_debut = time.time()
    points_filtres,colors_filtres,_ = pc.apply_hsv_mask(points,colors,MASK_HSV,size_acqui)
    temps_fin = time.time()
    temps_execution = temps_fin - temps_debut
    print(f"Temps mask : {temps_execution} secondes")
    
    # Filtrage bruit
    temps_debut = time.time()
    points_filtres_sphere, colors_filtres_sphere,_ = pc.filter_with_sphere_on_barycentre(points_filtres,radius, colors_filtres)
    
    if (len(points_filtres_sphere)>2000):
        points_filtres_sphere, colors_filtres_sphere = pc.reduce_density(points_filtres_sphere,2000/len(points_filtres_sphere),colors_filtres_sphere)
    temps_fin = time.time()
    temps_execution = temps_fin - temps_debut
    print(f"Temps filtrage bruit : {temps_execution} secondes")
    
    # Repositionnement
    temps_debut = time.time()      
    points_reposed = pc.centers_points_on_geometry(points_filtres_sphere)
    translation_vector = pc.get_center_geometry(points_filtres_sphere)

    Mt = tf.translation_matrix(translation_vector) 
    temps_fin = time.time()
    temps_execution = temps_fin - temps_debut
    print(f"Temps repose : {temps_execution} secondes")
    
    # Pré-rotation
    temps_debut = time.time()    
    M_icp_1,best_angle = cp.find_the_best_pre_rotation_to_align_points(POINTS_MODEL_3D_RESIZED, points_reposed,[best_angle[0]-5, best_angle[0]+5, 5],[best_angle[1]-5, best_angle[1]+5, 5],[best_angle[2]-5, best_angle[2]+5, 5])
    # M_icp_1,best_angle = cp.find_the_best_pre_rotation_to_align_points(POINTS_MODEL_3D_RESIZED, points_reposed,[0, 0, 10],[0, 0, 10],[-180, 180, 20])
    M_icp_1 = np.hstack((M_icp_1, np.array([[0], [0], [0]])))
    M_icp_1 = np.vstack((M_icp_1, np.array([0, 0, 0, 1])))

    M_ICP_1_INV = np.linalg.inv(M_icp_1)
    temps_fin = time.time()
    temps_execution = temps_fin - temps_debut
    print(f"Temps pre-rot : {temps_execution} secondes")
    
    # ICP
    temps_debut = time.time()    
    MODEL_3D_POINTS_AFTER_PRE_ROTATION = np.array([(float(x), float(y), float(z)) for (
    x, y, z,t) in [M_ICP_1_INV @ p for p in np.column_stack((point_model_3D_resize, np.ones(len(
    point_model_3D_resize))))]], dtype=np.float64)
    
    M_icp_2,_ = cp.find_transform_matrix_to_align_points_using_icp(MODEL_3D_POINTS_AFTER_PRE_ROTATION, points_reposed)

    angles_ICP2 = tf.transformation_matrix_to_euler_xyz(M_icp_2)

    x = -angles_ICP2[0]
    y = angles_ICP2[1]
    z = -angles_ICP2[2]

    M_icp_2_inv = np.linalg.inv(tf.matrix_from_angles(x, y, z))
    temps_fin = time.time()
    temps_execution = temps_fin - temps_debut
    print(f"Temps ICP : {temps_execution} secondes")   
    # Calcul de projection 
    temps_debut = time.time()          
    M_projection = M_in @ Mt @ M_ICP_1_INV @ M_icp_2_inv  # Matrice de "projection"

    colors_image = proj.project_3D_model_on_pc(colors, point_model_3D_resize, COLORS_MODEL_3D, M_projection,size_acqui)

    # Affichage 
    temps_debut = time.time()     
    cv2.imshow("Color Image", colors_image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        pipeline.stop()
        cv2.destroyAllWindows()
        break
               
    # video_writer.write(colors_image)
    temps_fin = time.time()
    temps_execution = temps_fin - temps_debut
    print(f"Temps affichage : {temps_execution} secondes")
    