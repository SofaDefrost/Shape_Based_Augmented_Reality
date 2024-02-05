import os
import numpy as np
import cv2
import time
import logging

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


repertoire_cible = "labo_biologie/3eme_semaine/"

def rotation_matrix_y(beta: float) -> np.ndarray:
    """
    Generate a 3x3 rotation matrix for rotation around the y-axis.

    Args:
        beta (float): Rotation angle in radians.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])

fichiers_ply=["poulet_2_3D_model.ply"]

for fichier in fichiers_ply:
    print(fichier)
    p,c=ply.get_points_and_colors(repertoire_cible+fichier)
    angle = np.radians(-180)
    Mat_y =rotation_matrix_y(angle)    
    p= np.array([(float(x), float(y), float(z)) for (
    x, y, z) in [Mat_y @ point for point in p ]])
    ply.save(repertoire_cible+fichier,p,c)