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

# from Python_3D_Toolbox_for_Realsense import acquisition_realsense as aq
# from Python_3D_Toolbox_for_Realsense import info_realsense as ir
from Python_3D_Toolbox_for_Realsense.functions import processing_point_cloud as pc
from Python_3D_Toolbox_for_Realsense.functions import processing_pixel_list as pixels
from Python_3D_Toolbox_for_Realsense.functions import (
    previsualisation_application_function as Tk,
)
from Python_3D_Toolbox_for_Realsense.functions import processing_ply as ply
from Python_3D_Toolbox_for_Realsense.functions.utils import array as array

from rigid_transformation import RigidTransform

import time

from realsense_camera import RealsenseCamera

# size_acqui = (1280,720)
size_acqui = (848, 480)

rs_camera = RealsenseCamera(size_acqui[0], size_acqui[1])
calibration_matrix = rs_camera.get_calibration_matrix()
# Get point cloud with the realsense camera
fx = calibration_matrix[0, 0]
fy = calibration_matrix[1, 1]
cx = calibration_matrix[0, 2]
cy = calibration_matrix[1, 2]

point_cloud_id = 0
point_cloud_list = []

frame_num = 0

while 1:
    points, colors = rs_camera.get_points_and_colors()  # Capture the point cloud
    frame_num += 1
    colored_image = cv2.cvtColor(colors, cv2.COLOR_RGB2BGR)
    cv2.imshow("frame", colored_image)
    keys = cv2.waitKey(1)
    if keys & 0xFF == ord("s"):
        cv2.destroyAllWindows()
        break

    if keys & 0xFF == ord("c"):
        gray_image = cv2.cvtColor(colored_image, cv2.COLOR_RGB2GRAY)
        # select ROI
        # points_crop, colors_crop, tab_index_crop, new_shape = pc.crop_from_zone_selection(
        #     points=points, colors=colors, shape=size_acqui, tab_index=np.array([i for i in range(len(points))]))

        colors_crop, new_shape, selection_zone = (
            pc.crop_color_image_from_zone_selection(colors=colors, shape=size_acqui)
        )

        # # # Get the mask
        mask_hsv = pixels.get_hsv_mask_with_sliders(colors_crop, new_shape)

        # # # Apply hsv mask
        points_filtered_hsv = pc.apply_hsv_mask_f(
            points,
            colors_crop,
            mask_hsv,
            size_acqui,
            new_shape,
            selection_zone,
            calibration_matrix,
        )

        # only keep point where z is smaller than 0.3:
        points_close = points_filtered_hsv[points_filtered_hsv[:, 2] < 0.5]

        points_close = np.array(points_close)
        # # # draw point cloud
        # axs = plt.figure().add_subplot(projection="3d")
        # axs.scatter(
        #     points_close[:, 0],
        #     points_close[:, 1],
        #     points_close[:, 2],
        # )
        # plt.show()

        new_point_list = []
        # Transform the points in the camera frame to the aprialtag frame
        for p in points_close:
            # if detected_tag_id == 0:
            point = np.array(pose.inverse().transform_vector_3d(p))
            # point = np.array([p[0], p[1], p[2]])
            # elif detected_tag_id == 1:
            #     transformation = tf[1] @ pose.inverse().compute_transformation_matrix()
            #     point = np.array(
            #         RigidTransform.create_from_transformation_matrix(
            #             transformation
            #         ).transform_vector_3d(p)
            #     )

            new_point_list.append(point)
        new_point_list = np.array(new_point_list)
        point_cloud_list.append(new_point_list)
        ply.save(f"./data-1712/output_{point_cloud_id}.ply", new_point_list)
        print("Point cloud saved")
        point_cloud_id += 1
        frame_num = 0
