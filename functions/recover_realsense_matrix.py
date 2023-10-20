import pyrealsense2 as rs
import numpy as np

def recover_matrix_calib():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 0, 0, rs.format.z16, 30)  # Configure depth stream
    profile = pipeline.start(config)

    depth_profile = profile.get_stream(rs.stream.depth)
    depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()

    # Récupère les paramètres de la matrice de calibration
    fx, fy, cx, cy = depth_intrinsics.fx, depth_intrinsics.fy, depth_intrinsics.ppx, depth_intrinsics.ppy

    # Matrice de calibration
    print("Matrice de calibration:")
    calibration_matrix = np.array([[fx, 0, cx-100],
                                   [0, fy, cy],
                                   [0, 0, 1]], dtype=np.float32)
    return calibration_matrix

# Appel de la fonction et affichage de la matrice de calibration
calibration_matrix = recover_matrix_calib()
print(calibration_matrix)
