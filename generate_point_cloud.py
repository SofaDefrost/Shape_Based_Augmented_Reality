"""
Generate point cloud from realsense camera
Function of the script:
    1. Stream the images from the realsense camera
    2. Capture the point cloud on command
    3. Save the point cloud in a ply file
    4. Merge the point clouds and display the result
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
from dt_apriltags import Detector

from Python_3D_Toolbox_for_Realsense import acquisition_realsense as aq
from Python_3D_Toolbox_for_Realsense import info_realsense as ir
from Python_3D_Toolbox_for_Realsense.functions import processing_point_cloud as pc
from Python_3D_Toolbox_for_Realsense.functions import processing_pixel_list as pixels
from Python_3D_Toolbox_for_Realsense.functions import previsualisation_application_function as Tk
from Python_3D_Toolbox_for_Realsense.functions import processing_ply as ply
from Python_3D_Toolbox_for_Realsense.functions.utils import array as array

from rigid_transformation import RigidTransform

import time

size_acqui = (1280,720)
calibration_matrix = ir.get_matrix_calib(size_acqui[0],size_acqui[1])
M_in = np.hstack((calibration_matrix, np.zeros((3, 1))))
M_in = np.vstack((M_in, np.array([0, 0, 0, 1])))
# Get point cloud with the realsense camera
pipeline = aq.init_realsense(size_acqui[0],size_acqui[1])
fx = calibration_matrix[0,0]
fy = calibration_matrix[1,1]
cx = calibration_matrix[0,2]
cy = calibration_matrix[1,2]
tag_size = 0.029

at_detector = Detector(families='tagStandard52h13',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)
point_cloud_id = 0
point_cloud_list = []

#offset of the tags on the documents in meters
offset_tag0 = np.array([ 50,  10, 0]) * 1e-3
offset_tag1 = np.array([ 10, 110, 0]) * 1e-3
offset_tag2 = np.array([120, 110, 0]) * 1e-3

    
while(1):
    points, colors = aq.get_points_and_colors_from_realsense_w_filter(pipeline = pipeline, filtered = True) # Capture the point cloud
    colored_image = cv2.cvtColor(colors, cv2.COLOR_RGB2BGR)
    cv2.imshow('frame', colored_image)
    keys = cv2.waitKey(1)
    if keys & 0xFF == ord('s'):
        # q to merge all point cloud and quit
        points, color = ply.get_points_and_colors("./output_0.ply")
        for i in range(1, point_cloud_id):
            points1, color1 = ply.get_points_and_colors(f"./output_{i}.ply")
            points = np.append(points, points1, axis=0)
            
        print(f"Point cloud are generated in shape of {points.shape}")
        ply.save("./output_point_cloud.ply", points)
        break
    if keys & 0xFF == ord('c'):
        # c to capture the point cloud
        gray_image = cv2.cvtColor(colored_image, cv2.COLOR_RGB2GRAY)
        tags = at_detector.detect(gray_image, True, [fx,fy,cx,cy], tag_size)
        if len(tags) == 0:
            print("No tags detected")
            continue
        # tags.sort(key=lambda x: x.pose_err)
        pose = RigidTransform.create_from_position_matrix(tags[0].pose_t, tags[0].pose_R)
        detected_tag_id = tags[0].tag_id
        print(f"Tag {detected_tag_id} detected")
        
        # select ROI
        # points_crop, colors_crop, tab_index_crop, new_shape = pc.crop_from_zone_selection(
        #     points=points, colors=colors, shape=size_acqui, tab_index=np.array([i for i in range(len(points))]))
        
        colors_crop, new_shape, selection_zone = pc.crop_color_image_from_zone_selection(
            colors=colors, shape=size_acqui)

        # Get the mask
        mask_hsv = pixels.get_hsv_mask_with_sliders(colors_crop, new_shape)

        # Apply hsv mask
        points_filtered_hsv= pc.apply_hsv_mask_f(
            points, colors_crop, mask_hsv, size_acqui, new_shape, selection_zone, calibration_matrix)
        
        print(f"Point cloud filtered in shape of {points_filtered_hsv.shape}")

        
        # Remove noisy values
        radius = Tk.get_parameter_using_preview(
            points_filtered_hsv, pc.filter_with_sphere_on_barycentre, "Radius")
        points_filtered_noise  = pc.filter_with_sphere_on_barycentre(
            points_filtered_hsv, radius)
        
        if (len(points_filtered_noise)>2000):
            points_for_resize_only, _ = pc.reduce_density(points_filtered_noise,2000/len(points_filtered_noise))
        else :
            points_for_resize_only = points_filtered_noise[0]
        
        print(type(points_for_resize_only))
        print(points_for_resize_only.shape)
            
        points_for_resize_only = np.array(points_for_resize_only)
        # # draw point cloud
        axs = plt.figure().add_subplot(projection='3d')
        axs.scatter(points_for_resize_only[:, 0], points_for_resize_only[:, 1], points_for_resize_only[:, 2])
        plt.show()
        
        new_point_list = []
        # Transform the points in the camera frame to the aprialtag frame
        for p in points_for_resize_only:
            point = np.array(pose.inverse().transform_vector_3d(p))
            # if detected_tag_id == 0:
            #     point += offset_tag0
            # elif detected_tag_id == 1:
            #     point += offset_tag1
            # elif detected_tag_id == 2:
            #     point += offset_tag2
            new_point_list.append(point)
        new_point_list = np.array(new_point_list)
        point_cloud_list.append(new_point_list)
        ply.save(f"output_{point_cloud_id}.ply", new_point_list)
        print("Point cloud saved")
        point_cloud_id += 1
        