import os
import logging
import numpy as np
import cv2
import time

from functions import icp as cp
from functions import project_and_display as proj
from functions import matrix_operations as tf

from Python_3D_Toolbox_for_Realsense import acquisition_realsense as aq
from Python_3D_Toolbox_for_Realsense import calibration_matrix_realsense as rc
from Python_3D_Toolbox_for_Realsense.functions.utils import array as array
from Python_3D_Toolbox_for_Realsense.functions import processing_ply as ply
from Python_3D_Toolbox_for_Realsense.functions import processing_point_cloud as pc
from Python_3D_Toolbox_for_Realsense.functions import processing_pixel_list as pixels
from Python_3D_Toolbox_for_Realsense.functions import processing_img as img
from Python_3D_Toolbox_for_Realsense.functions import previsualisation_application_function as Tk

############### Loading ####################

# Load the 3D model

name_model_3D = "example/input/stomach_3D_rainbow_colored.ply"
name_for_output = "example/output/test"

points_model_3D, colors_model_3D = ply.get_points_and_colors(name_model_3D)

################### Acquisition ###########################

# Get point cloud with the realsense camera
size_acqui = (1280,720)
pipeline = aq.init_realsense(size_acqui[0],size_acqui[1])
points, colors = aq.get_points_and_colors_from_realsense(pipeline) # Capture the point cloud

# Or : load an existing .ply file
# name_pc = "example/input/point_cloud_test_stomach.ply"
# size_acqui = (1280,720) # The size of the acquisition
# points, colors = ply.get_points_and_colors(name_pc)

tab_index = np.array([i for i in range(len(points))])

##################### Select Zone ####################

points_crop, colors_crop, tab_index_crop, new_shape = pc.crop_from_zone_selection(
    points=points, colors=colors, shape=size_acqui, tab_index=tab_index)

###################### Mask ###########################

# Get the mask

mask_hsv = pixels.get_hsv_mask_with_sliders(colors_crop, new_shape)

# Apply hsv mask

points_filtered_hsv, colors_filtered_hsv, tab_index_hsv = pc.apply_hsv_mask(
    points_crop, colors_crop, mask_hsv, new_shape, tab_index_crop)

####################### Remove noisy values #####################

radius = Tk.get_parameter_using_preview(
    points_filtered_hsv, pc.filter_with_sphere_on_barycentre, "Radius")

points_filtered_noise, colors_filtered_noise, tab_index_filtered_noise = pc.filter_with_sphere_on_barycentre(
    points_filtered_hsv, radius, colors_filtered_hsv, tab_index_hsv)

######################### Resizing of 3D model ##################################

pc_too_big = True

if (len(points_filtered_noise)>2000):
        points_for_resize_only, _ = pc.reduce_density(points_filtered_noise,2000/len(points_filtered_noise))

# The resize algorithm can crash if we have too many points.
# We check if it is the case (if yes we reduced by half the number of points in the point cloud)
while pc_too_big:
    try:
        points_model_3D_resized = pc.resize_point_cloud_to_another_one(
            points_model_3D, points_for_resize_only)
        pc_too_big = False
    except Exception as e:
        logging.info(
            f"Too many points in the point cloud {name_model_3D} : we reduce the number of points by half")
        points_model_3D, colors_model_3D = pc.reduce_density(
            points_model_3D, 0.5, colors_model_3D)

################## Repose objects ####################

points_reposed = pc.centers_points_on_geometry(points_filtered_noise)
translation_vector = pc.get_center_geometry(points_filtered_noise)

Mt = tf.translation_matrix(translation_vector)

################ Pre-rotation matrix ###################

M_pre_rot, _ = cp.find_the_best_pre_rotation_to_align_points(points_model_3D_resized, points_reposed, [
                                                             0, 0, 10], [0, 0, 10], [-180, 180, 10])

M_pre_rot = np.hstack((M_pre_rot, np.array([[0], [0], [0]])))
M_pre_rot = np.vstack((M_pre_rot, np.array([0, 0, 0, 1])))

M_pre_rot_inv = np.linalg.inv(M_pre_rot)

###################### ICP Matrix #########################

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

################# Display using closest points identification #######################

M = Mt @ M_pre_rot_inv @ M_icp_inv

colors_projection_cpi = proj.project_3D_model_on_pc_using_closest_points_identification(
    points_model_3D_resized, colors_model_3D, points, colors, M,size_acqui)

img.save(colors_projection_cpi, name_for_output +
         "_projection_closest_points.png", size_acqui)
ply.save(name_for_output+"_projection_closest_points.ply", points, colors_projection_cpi)

color_image_cpi = img.load(name_for_output + "_projection_closest_points.png")

while True:
    cv2.imshow("Projection using closest points identification",
               color_image_cpi[:, :, ::-1])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

################# Display using indices identification #######################

colors_projection_ii = proj.project_3D_model_on_pc_using_closest_points_and_indices(
    points_model_3D_resized, colors_model_3D, points_filtered_noise, colors, tab_index_filtered_noise, M,size_acqui)

img.save(colors_projection_ii, name_for_output +
         "_projection_using_indices.png", size_acqui)
ply.save(name_for_output+"_projection_using_indices.ply", points, colors_projection_ii)

color_image_ii = img.load(name_for_output + "_projection_using_indices.png")

while True:
    cv2.imshow("projection using indices identification",
               color_image_ii[:, :, ::-1])
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

################# Display using projection ######################

# Calibration matrix

calibration_matrix = rc.recover_matrix_calib(size_acqui[0],size_acqui[1])
M_in = np.hstack((calibration_matrix, np.zeros((3, 1))))
M_in = np.vstack((M_in, np.array([0, 0, 0, 1])))

projection = M_in @ Mt @ M_pre_rot_inv @ M_icp_inv

if len(colors_model_3D) == 0:
    colors_model_3D = np.asarray(
        [[0., 0., 255.] for i in range(len(np.asarray(points_model_3D)))])

while True:
    frame_apres = proj.project_3D_model_on_pc(
        colors, points_model_3D_resized, colors_model_3D, projection,size_acqui)
    cv2.imshow("projection of 3D model on pc", frame_apres)
    cv2.imwrite(name_for_output + "_projection.png", frame_apres)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
