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
from Python_3D_Toolbox_for_Realsense.functions import (
    previsualisation_application_function as Tk,
)
from Python_3D_Toolbox_for_Realsense.functions import processing_ply as ply
from Python_3D_Toolbox_for_Realsense.functions.utils import array as array

from rigid_transformation import RigidTransform

import time

# size_acqui = (1280,720)
size_acqui = (848, 480)
calibration_matrix = ir.get_matrix_calib(size_acqui[0], size_acqui[1])
M_in = np.hstack((calibration_matrix, np.zeros((3, 1))))
M_in = np.vstack((M_in, np.array([0, 0, 0, 1])))
# Get point cloud with the realsense camera
pipeline = aq.init_realsense(size_acqui[0], size_acqui[1])
fx = calibration_matrix[0, 0]
fy = calibration_matrix[1, 1]
cx = calibration_matrix[0, 2]
cy = calibration_matrix[1, 2]
tag_size = 0.029

at_detector = Detector(
    families="tagStandard52h13",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0,
)
point_cloud_id = 0
point_cloud_list = []

tag_list = []


class tagInfo:
    def __init__(self, tag_id, tag_pose):
        self.tag_id = tag_id
        self.pose = tag_pose


# offset of the tags on the documents in meters
# offset_tag0 = np.array([ 50,  10, 0]) * 1e-3
# offset_tag1 = np.array([ 10, 110, 0]) * 1e-3
# offset_tag2 = np.array([120, 110, 0]) * 1e-3

frame_num = 0

while 1:
    points, colors = aq.get_points_and_colors_from_realsense_w_filter(
        pipeline=pipeline, filtered=False
    )  # Capture the point cloud
    frame_num += 1
    colored_image = cv2.cvtColor(colors, cv2.COLOR_RGB2BGR)
    cv2.imshow("frame", colored_image)
    keys = cv2.waitKey(1)
    if keys & 0xFF == ord("s"):
        cv2.destroyAllWindows()
        # q to merge all point cloud and quit
        # points, color = ply.get_points_and_colors("./output_0.ply")
        # for i in range(1, point_cloud_id):
        #     points1, color1 = ply.get_points_and_colors(f"./output_{i}.ply")
        #     points = np.append(points, points1, axis=0)

        # print(f"Point cloud are generated in shape of {points.shape}")
        # ply.save("./output_point_cloud.ply", points)
        break
    # if keys & 0xFF == ord("l"):
    #     # l to calibrate the pose of the tags
    #     #   get the pose of the tags in frame 0
    #     gray_image = cv2.cvtColor(colored_image, cv2.COLOR_RGB2GRAY)
    #     tags = at_detector.detect(gray_image, True, [fx, fy, cx, cy], tag_size)
    #     if len(tags) == 0:
    #         print("No tags detected")
    #         continue
    #     if len(tags) == 1:
    #         print("Only one tag detected")
    #         continue
    #     print(tags)
    #     for tag in tags:
    #         tag_detected = tagInfo(
    #             tag.tag_id,
    #             RigidTransform.create_from_position_matrix(tag.pose_t, tag.pose_R),
    #         )

    #         tag_list.append(tag_detected)
    #         print(f"Tag {tag_detected.tag_id} detected")

    #     tag_list.sort(key=lambda x: x.tag_id)
    #     tf = [None] * len(tag_list)
    #     for tag in tag_list:
    #         if tag.tag_id > 0:
    #             tf[tag.tag_id] = (
    #                 tag_list[0].pose.inverse().compute_transformation_matrix()
    #                 @ tag.pose.compute_transformation_matrix()
    #             )
    #             # tf[tag.tag_id] = (
    #             #     tag.pose.inverse().compute_transformation_matrix()
    #             #     @ tag_list[0].pose.compute_transformation_matrix()
    #             # )
    #             print(tf[tag.tag_id])

    if keys & 0xFF == ord("c"):
    # if frame_num % 10 == 0:
        # c to capture the point cloud
        gray_image = cv2.cvtColor(colored_image, cv2.COLOR_RGB2GRAY)
        tags = at_detector.detect(gray_image, True, [fx, fy, cx, cy], tag_size)
        if len(tags) == 0:
            print("No tags detected")
            continue
        # tags.sort(key=lambda x: x.pose_err)
        pose = RigidTransform.create_from_position_matrix(
            tags[0].pose_t, tags[0].pose_R
        )
        detected_tag_id = tags[0].tag_id
        print(f"Tag {detected_tag_id} detected")

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

        # print(f"Point cloud filtered in shape of {points_filtered_hsv.shape}")

        # # Remove noisy values
        # radius = Tk.get_parameter_using_preview(
        #     points_filtered_hsv, pc.filter_with_sphere_on_barycentre, "Radius"
        # )
        # points_filtered_noise = pc.filter_with_sphere_on_barycentre(
        #     points_filtered_hsv, radius
        # )

        # if len(points_filtered_noise) > 2000:
        #     points_for_resize_only, _ = pc.reduce_density(
        #         points_filtered_noise, 2000 / len(points_filtered_noise)
        #     )
        # else:
        #     points_for_resize_only = points_filtered_noise[0]

        # print(type(points_for_resize_only))
        # print(points_for_resize_only.shape)

        # only keep point where z is smaller than 0.3:
        points_close = points_filtered_hsv[points_filtered_hsv[:, 2] < 0.5]

        # points_close = points[points[:, 2] < 0.5]
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
