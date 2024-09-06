"""Get volume of the object from its point cloud"""

import numpy as np
import matplotlib.pyplot as plt
from Python_3D_Toolbox_for_Realsense.functions import processing_point_cloud as pc
from Python_3D_Toolbox_for_Realsense.functions import processing_ply as ply

points, color = ply.get_points_and_colors("./output_0.ply")

for i in range(1, 4):
    points1, color1 = ply.get_points_and_colors(f"./output_{i}.ply")
    points = np.append(points, points1, axis=0)

print(points.shape)

ply.save("./output_point_cloud.ply", points)