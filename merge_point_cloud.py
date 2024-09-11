"""Get volume of the object from its point cloud"""
import numpy as np
import open3d as o3d
from tqdm import tqdm

pcd_combined = o3d.geometry.PointCloud()
for i in tqdm(range(3)):
    pcd = o3d.io.read_point_cloud(f"./data-1712/output_{i}.ply")
    pcd_combined += pcd

print("All point read")
# voxel_size = 0.1
pcd_combined.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
print("Statistical outlier removed")
# pcd_combined.voxel_down_sample(voxel_size)
pcd_simp = pcd_combined.farthest_point_down_sample(num_samples=500)
print("Voxel down sample")
o3d.visualization.draw(pcd_simp)

o3d.io.write_point_cloud("pcd_combined.ply", pcd_simp)
