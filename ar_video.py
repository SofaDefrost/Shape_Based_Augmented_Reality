import numpy as np
import cv2
import time
import logging

from functions import icp as cp
from functions import project_and_display as proj
from functions import matrix_operations as tf

from Python_3D_Toolbox_for_Realsense import acquisition_realsense as aq
from Python_3D_Toolbox_for_Realsense import info_realsense as ir
from Python_3D_Toolbox_for_Realsense.functions import processing_multiple_ply as mply
from Python_3D_Toolbox_for_Realsense.functions import processing_ply as ply
from Python_3D_Toolbox_for_Realsense.functions import processing_point_cloud as pc
from Python_3D_Toolbox_for_Realsense.functions import processing_pixel_list as pixels
from Python_3D_Toolbox_for_Realsense.functions import processing_img as img
from Python_3D_Toolbox_for_Realsense.functions import previsualisation_application_function as Tk
from Python_3D_Toolbox_for_Realsense.functions.utils import array as array

############# Settings ###############

loading_mply_file = True
name_mply_file = "example/input/sofa.mply" # Could be umpty if loading_video_file == False

video_recording = True
name_video_output = "example/output/test_sofa.mp4" # Could be umpty if video_recording == False
fps_for_video_output = 1 # Could be null if video_recording == False

display_processing_time = True

name_model_3D = "example/input/SOFA_logo.ply"

size_acqui = (1280, 720)
# Calibration matrix of the camera (here is the one for (1280, 720) resolution) used for creating the file
M_in = np.asarray([[640.05206, 0, 639.1219, 0], [0, 640.05206, 361.61005, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) # Could be null if loading_mply_file == False

distance_x_max_points = 1 # Unit : meter
distance_y_max_points = 1 # Unit : meter
distance_z_max_points = 1 # Unit : meter

range_for_angles_pre_rot = 1 # Unit : degree
step_for_angles_pre_rot = 1 # Unit : degree

#####################################

## Generate the first picture 

# Load the 3D model

points_model_3D, colors_model_3D = ply.get_points_and_colors(name_model_3D)

# Acquisition

if loading_mply_file:
    points,colors=mply.get_point_cloud(name_mply_file,1)
else:
    # Get the calibration matrix (will be helpfull later)
    calibration_matrix = ir.get_matrix_calib(size_acqui[0],size_acqui[1])
    M_in = np.hstack((calibration_matrix, np.zeros((3, 1))))
    M_in = np.vstack((M_in, np.array([0, 0, 0, 1])))
    # Get point cloud with the realsense camera
    pipeline = aq.init_realsense(size_acqui[0], size_acqui[1])
    points, colors = aq.get_points_and_colors_from_realsense(pipeline)
    
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

M = M_in @ Mt @ M_pre_rot_inv @ M_icp_inv

colors_projection = proj.project_3D_model_on_pc(
    colors, points_model_3D_resized, colors_model_3D, M, size_acqui)

while True:
    cv2.imshow("Projection", colors_projection)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

#################### For next pictures ####################

if video_recording:
    ## For video recording
    video_writer = cv2.VideoWriter(name_video_output, cv2.VideoWriter_fourcc(*'XVID'), fps_for_video_output, (size_acqui[0], size_acqui[1]))

if loading_mply_file:
    file = open(name_mply_file, "r")
    line = file.readline() # mply
    line = file.readline() # format ascii 1.0
    number_pc = int(file.readline().strip().split()[-1:][0]) # number pc
    size_pc = int(file.readline().strip().split()[-1:][0]) # size pc
    while not(line.startswith("end_header")):
        line = file.readline()
    line = file.readline()
    pc_numb = 1

while True:
    if display_processing_time:
        temps_start = time.time()
        
    if loading_mply_file:
        if pc_numb<number_pc:
            points=[]
            colors=[]
            for i in range(size_pc):
                data = line.strip().split()
                points.append([float(x) for x in data[:3]])
                if len(data)>3:
                    colors.append([int(x) for x in data[-3:]])
                line = file.readline()
            points=np.array(points)
            colors=np.array(colors)
            # Because we are now on the line "end_pc_"
            line = file.readline()
            pc_numb +=1
        else:
            break
    else:
        # Acquisition
        points, colors = aq.get_points_and_colors_from_realsense(pipeline)
    
    points_filtered_hsv, colors_filtered_hsv, _ = pc.apply_hsv_mask(points, colors, mask_hsv, size_acqui)

    if (len(points_filtered_hsv) > 1000):
        points_filtered_hsv, colors_filtered_hsv = pc.reduce_density(
            points_filtered_hsv, 1000/len(points_filtered_hsv), colors_filtered_hsv)
    
    # Remove noisy data
    
    points_filtered_x, colors_filtered_x = pc.remove_points_threshold(points_filtered_hsv,distance_x_max_points,1,colors_filtered_hsv,'x')
    points_filtered_y, colors_filtered_y = pc.remove_points_threshold(points_filtered_x,distance_y_max_points,1,colors_filtered_x,'y')
    points_filtered_z, colors_filtered_z = pc.remove_points_threshold(points_filtered_y,distance_z_max_points,1,colors_filtered_y,'z')
    
    points_filtres_sphere, colors_filtres_sphere, _ = pc.filter_with_sphere_on_barycentre(
        points_filtered_z, radius, colors_filtered_z)

    if len(points_filtres_sphere)>400:

        # Repose objects
        points_reposed = pc.centers_points_on_geometry(points_filtres_sphere)
        translation_vector = pc.get_center_geometry(points_filtres_sphere)
            
        Mt = tf.translation_matrix(translation_vector)

        # Pre-rotation
        M_pre_rot, best_angle = cp.find_the_best_pre_rotation_to_align_points(points_model_3D_resized, points_reposed, [
                                                                            best_angle[0]-range_for_angles_pre_rot, best_angle[0]+range_for_angles_pre_rot, step_for_angles_pre_rot], [best_angle[1]-range_for_angles_pre_rot, best_angle[1]+range_for_angles_pre_rot, step_for_angles_pre_rot], [best_angle[2]-range_for_angles_pre_rot, best_angle[2]+range_for_angles_pre_rot, step_for_angles_pre_rot])
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
            # pipeline.stop()
            cv2.destroyAllWindows()
            break
        
        if video_recording:
            ## For video recording
            video_writer.write(colors_image)
            
        if display_processing_time: 
            temps_end = time.time()
            temps_processing = temps_end - temps_start
            print(f"Time for processing: {temps_processing} seconds")
    else:
        print('Not enough points')
        ply.save("debug.ply",points,colors)
        ply.save("debug_filtre.ply",points_filtered_hsv,colors_filtered_hsv)