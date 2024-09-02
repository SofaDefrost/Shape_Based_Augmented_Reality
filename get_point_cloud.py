""" Capture a point cloud from one picture from the realsense camera and save it as a .ply file """
import cv2
import logging
from dt_apriltags import Detector

import numpy as np
import matplotlib.pyplot as plt
from Python_3D_Toolbox_for_Realsense import acquisition_realsense as aq
from Python_3D_Toolbox_for_Realsense import info_realsense as ir
from Python_3D_Toolbox_for_Realsense.functions import processing_point_cloud as pc
from Python_3D_Toolbox_for_Realsense.functions import processing_pixel_list as pixels
from Python_3D_Toolbox_for_Realsense.functions import previsualisation_application_function as Tk
from Python_3D_Toolbox_for_Realsense.functions import processing_ply as ply

from rigid_transformation import RigidTransform

size_acqui = (1280,720)
calibration_matrix = ir.get_matrix_calib(size_acqui[0],size_acqui[1])
M_in = np.hstack((calibration_matrix, np.zeros((3, 1))))
M_in = np.vstack((M_in, np.array([0, 0, 0, 1])))
# Get point cloud with the realsense camera
pipeline = aq.init_realsense(size_acqui[0],size_acqui[1])
points, colors = aq.get_points_and_colors_from_realsense(pipeline) # Capture the point cloud
tab_index = np.array([i for i in range(len(points))])

###################Pose Estimation####################
gray_image = cv2.cvtColor(colors, cv2.COLOR_RGB2GRAY)

fx = calibration_matrix[0,0]
fy = calibration_matrix[1,1]
cx = calibration_matrix[0,2]
cy = calibration_matrix[1,2]


at_detector = Detector(families='tagStandard52h13',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

tags = at_detector.detect(gray_image, True, [fx,fy,cx,cy], 0.029)

pose = RigidTransform.create_from_position_matrix(tags[0].pose_t, tags[0].pose_R)

##################### Select Zone ####################
points_crop, colors_crop, tab_index_crop, new_shape = pc.crop_from_zone_selection(
    points=points, colors=colors, shape=size_acqui, tab_index=tab_index)


###################### Mask ###########################

# Get the mask

mask_hsv = pixels.get_hsv_mask_with_sliders(colors_crop, new_shape)

# Apply hsv mask

points_filtered_hsv, colors_filtered_hsv, tab_index_hsv = pc.apply_hsv_mask(
    points_crop, colors_crop, mask_hsv, new_shape, tab_index_crop)

# ####################### Remove noisy values #####################

# radius = Tk.get_parameter_using_preview(
#     points_filtered_hsv, pc.filter_with_sphere_on_barycentre, "Radius")


radius = 0.08

points_filtered_noise, colors_filtered_noise, tab_index_filtered_noise = pc.filter_with_sphere_on_barycentre(
    points_filtered_hsv, radius, colors_filtered_hsv, tab_index_hsv)

######################### Reduce the size of the point cloud ##################################

pc_too_big = True

if (len(points_filtered_noise)>2000):
        points_for_resize_only, _ = pc.reduce_density(points_filtered_noise,2000/len(points_filtered_noise))


new_point_list = []
for p in points_for_resize_only:
    point = pose.inverse().transform_vector_3d(p)
    new_point_list.append(point)

new_point_list = np.array(new_point_list)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(new_point_list[:, 0], new_point_list[:, 1], new_point_list[:, 2], s=1)
plt.show()

ply.save("output2.ply", new_point_list)
