import numpy as np
import cv2
import time
import logging

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

############## First picture ################

# Load the 3D model

name_model_3D = "data_exemple/estomac_3D_model_reduced_density_colored.ply"
name = "data_exemple/test_estomac"

points_model_3D, colors_model_3D = ply.get_points_and_colors(name_model_3D)

if len(colors_model_3D) == 0:
    colors_model_3D = np.asarray(
        [[0., 0., 255.] for i in range(len(np.asarray(points_model_3D)))])

# Acquisition

# Get point cloud with the realsense camera
size_acqui = (1280, 720)
pipeline = aq.init_realsense(size_acqui[0], size_acqui[1])

# Bon je ne sais pas pourquoi il faut que je le fasse plusiquer fois comme ça mais si je ne fais pas, l'image affichée n'est pas la même tout le long du truc ...
points, colors = aq.get_points_and_colors_from_realsense(
    pipeline)  # On fait une acquisition
pixels.display(colors, "Display")
points, colors = aq.get_points_and_colors_from_realsense(
    pipeline)  # On fait une acquisition
pixels.display(colors, "Display")
points, colors = aq.get_points_and_colors_from_realsense(
    pipeline)  # On fait une acquisition
pixels.display(colors, "Display")

# Mask

# Get mask

mask_hsv = pixels.get_hsv_mask_with_sliders(colors, size_acqui)

# Apply mask

points_filtered_hsv, colors_filtered_hsv, _ = pc.apply_hsv_mask(
    points, colors, mask_hsv, size_acqui)

# Remove noisy values

radius = Tk.get_parameter_using_preview(
    points_filtered_hsv, pc.filter_with_sphere_on_barycentre, "Radius")

points_filtered_noise, colors_filtered_noise, _ = pc.filter_with_sphere_on_barycentre(
    points_filtered_hsv, radius, colors_filtered_hsv)

# To gain speed

if (len(points_filtered_noise) > 2000):
    points_filtered_noise, colors_filtered_noise = pc.reduce_density(
        points_filtered_noise, 2000/len(points_filtered_noise), colors_filtered_noise)


# Resizing of 3D model

pc_too_big = True

while pc_too_big:
    # The resize algorithm can crash if we have too many points.
    # We check if it is the case (if yes we reduced by half the number of points in the point cloud)
    try:
        points_model_3D_resized = pc.resize_point_cloud_to_another_one(
            points_model_3D, points_filtered_noise)
        pc_too_big = False
    except Exception as e:
        logging.info(
            f"Too many points in the point cloud {name_model_3D} : we reduce the number of points by half")
        points_model_3D, colors_model_3D = pc.reduce_density(
            points_model_3D, 0.5, colors_model_3D)

# Repose objects

points_reposed = pc.centers_points_on_geometry(points_filtered_noise)
translation_vector = pc.get_center_geometry(points_filtered_noise)

Mt = tf.translation_matrix(translation_vector)  # Matrice de translation

# Pre-rotation matrix

M_pre_rot, best_angle = cp.find_the_best_pre_rotation_to_align_points(
    points_model_3D_resized, points_reposed, [0, 0, 10], [0, 0, 10], [-180, 180, 10])

M_pre_rot = np.hstack((M_pre_rot, np.array([[0], [0], [0]])))
M_pre_rot = np.vstack((M_pre_rot, np.array([0, 0, 0, 1])))

M_pre_rot_inv = np.linalg.inv(M_pre_rot)

# ICP Matrix

model_3D_points_after_pre_rotation = np.array([(float(x), float(y), float(z)) for (
    x, y, z, t) in [M_pre_rot_inv @ p for p in np.column_stack((points_model_3D_resized, np.ones(len(
        points_model_3D_resized))))]], dtype=np.float64)

M_icp, _ = cp.find_transform_matrix_to_align_points_using_icp(
    model_3D_points_after_pre_rotation, points_reposed)

angles_ICP = tf.transformation_matrix_to_euler_xyz(M_icp)

x = -angles_ICP[0]
y = angles_ICP[1]
z = -angles_ICP[2]

M_icp_inv = np.linalg.inv(tf.matrix_from_angles(x, y, z))

# Display

calibration_matrix = rc.recover_matrix_calib(size_acqui[0], size_acqui[1])
M_in = np.hstack((calibration_matrix, np.zeros((3, 1))))
M_in = np.vstack((M_in, np.array([0, 0, 0, 1])))

M = M_in @ Mt @ M_pre_rot_inv @ M_icp_inv

colors_projection = proj.project_3D_model_on_pc(
    colors, points_model_3D_resized, colors_model_3D, M, size_acqui)

while True:
    cv2.imshow("Projection", colors_projection)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

#################### For next pictures ####################

# Video settings
# largeur, hauteur = size_acqui
# fps = 5
# video_writer = cv2.VideoWriter('test.mp4', cv2.VideoWriter_fourcc(*'XVID'), fps, (largeur, hauteur))

# Needed because the code that get the calibration matrix reset the pipeline
pipeline = aq.init_realsense(size_acqui[0], size_acqui[1])

while True:

    temps_debut = time.time()

    # Acquisition
    points, colors = aq.get_points_and_colors_from_realsense(pipeline)

    # Apply mask
    points_filtres, colors_filtres, _ = pc.apply_hsv_mask(
        points, colors, mask_hsv, size_acqui)

    # Remove noisy data
    points_filtres_sphere, colors_filtres_sphere, _ = pc.filter_with_sphere_on_barycentre(
        points_filtres, radius, colors_filtres)

    if (len(points_filtres_sphere) > 1000):
        points_filtres_sphere, colors_filtres_sphere = pc.reduce_density(
            points_filtres_sphere, 1000/len(points_filtres_sphere), colors_filtres_sphere)

    # Rpose objects
    points_reposed = pc.centers_points_on_geometry(points_filtres_sphere)
    translation_vector = pc.get_center_geometry(points_filtres_sphere)

    Mt = tf.translation_matrix(translation_vector)

    # Pre-rotation
    M_pre_rot, best_angle = cp.find_the_best_pre_rotation_to_align_points(points_model_3D_resized, points_reposed, [
                                                                        best_angle[0]-3, best_angle[0]+3, 3], [best_angle[1]-3, best_angle[1]+3, 3], [best_angle[2]-3, best_angle[2]+3, 3])
    # M_pre_rot,best_angle = cp.find_the_best_pre_rotation_to_align_points(points_model_3D_resized, points_reposed,[0, 0, 10],[0, 0, 10],[-180, 180, 20])
    M_pre_rot = np.hstack((M_pre_rot, np.array([[0], [0], [0]])))
    M_pre_rot = np.vstack((M_pre_rot, np.array([0, 0, 0, 1])))

    M_pre_rot_inv = np.linalg.inv(M_pre_rot)

    # ICP
    model_3D_points_after_pre_rotation = np.array([(float(x), float(y), float(z)) for (
        x, y, z, t) in [M_pre_rot_inv @ p for p in np.column_stack((points_model_3D_resized, np.ones(len(
            points_model_3D_resized))))]], dtype=np.float64)

    M_icp, _ = cp.find_transform_matrix_to_align_points_using_icp(
        model_3D_points_after_pre_rotation, points_reposed)

    angles_ICP = tf.transformation_matrix_to_euler_xyz(M_icp)

    x = -angles_ICP[0]
    y = angles_ICP[1]
    z = -angles_ICP[2]

    M_icp_inv = np.linalg.inv(tf.matrix_from_angles(x, y, z))

    # Projection
    M_projection = M_in @ Mt @ M_pre_rot_inv @ M_icp_inv  # Matrice de "projection"

    colors_image = proj.project_3D_model_on_pc(
        colors, points_model_3D_resized, colors_model_3D, M_projection, size_acqui)

    # Display
    cv2.imshow("Color Image", colors_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        pipeline.stop()
        cv2.destroyAllWindows()
        break

    # video_writer.write(colors_image)
    # temps_fin = time.time()
    # temps_execution = temps_fin - temps_debut
    # print(f"Temps affichage : {temps_execution} secondes")
