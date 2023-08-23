import numpy as np
import copy
from simpleicp import PointCloud, SimpleICP

import open3d as o3d

from scipy.spatial.transform import Rotation as Rot
import functions.filter_referential as fr


# source = o3d.io.read_point_cloud("./data_exemple/FleurDeLisThing.ply") # ply semblent non compatibles
# target = o3d.io.read_point_cloud("./data_exemple/source_a.ply")

# Read point clouds from xyz files into n-by-3 numpy arrays
target = np.genfromtxt("./data_exemple/fleurdelisthing.xyz")
source = np.genfromtxt("./data_exemple/source_b.xyz")

# Create point cloud objects
pc_fix = PointCloud(target, columns=["x", "y", "z"])
pc_mov = PointCloud(source, columns=["x", "y", "z"])

# Create simpleICP object, add point clouds, and run algorithm!
icp = SimpleICP()
icp.add_point_clouds(pc_fix, pc_mov)
H, X_mov_transformed, rigid_body_transformation_params, distance_residuals =  icp.run(max_overlap_distance=1)

angles_euler= fr.angles(H)