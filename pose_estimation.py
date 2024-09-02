"""Pose estimation using AprilTags and Realsense camera"""

import cv2
import numpy as np
from dt_apriltags import Detector
from Python_3D_Toolbox_for_Realsense import acquisition_realsense as aq
from Python_3D_Toolbox_for_Realsense import info_realsense as ir

from rigid_transformation import RigidTransform

size_acqui = (1280, 720)
calibration_matrix = ir.get_matrix_calib(size_acqui[0], size_acqui[1])
M_in = np.hstack((calibration_matrix, np.zeros((3, 1))))
M_in = np.vstack((M_in, np.array([0, 0, 0, 1])))
# Get point cloud with the realsense camera
pipeline = aq.init_realsense(size_acqui[0], size_acqui[1])
points, colors = aq.get_points_and_colors_from_realsense(
    pipeline
)  # Capture the point cloud

# cv2.imshow("Image", colors)
# cv2.waitKey()
# cv2.destroyAllWindows()
gray_image = cv2.cvtColor(colors, cv2.COLOR_RGB2GRAY)

fx = calibration_matrix[0, 0]
fy = calibration_matrix[1, 1]
cx = calibration_matrix[0, 2]
cy = calibration_matrix[1, 2]


at_detector = Detector(
    families="tagStandard52h13",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0,
)

tags = at_detector.detect(gray_image, True, [fx, fy, cx, cy], 0.029)

print(tags)

pipeline.stop()
