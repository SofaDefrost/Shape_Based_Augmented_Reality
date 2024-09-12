import numpy as np
import open3d as o3d

R = np.identity(3)  
extent = [0.2, 0.2, 0.2] # trying to create a bounding box below 1 unit
center = [0., 0, 0.3]
obb = o3d.geometry.OrientedBoundingBox(center,R,extent) 

data_folder = "data-1050"

cropped_pcd = o3d.geometry.PointCloud()

i = 0
pcd = o3d.io.read_point_cloud(f"./{data_folder}/t_{i}.ply")


for i in range(60):
    pcd = o3d.io.read_point_cloud(f"./{data_folder}/t_{i}.ply")

    cropped = pcd.crop(obb)
    cropped_pcd += cropped
    # o3d.visualization.draw_geometries([cropped]) #press ESC to close

    cropped.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    o3d.io.write_point_cloud(f"./{data_folder}/cropped_{i}.ply", cropped)

cropped_pcd = o3d.geometry.PointCloud()
list = [0, 20, 40, 50]
for i in list:
    pcd = o3d.io.read_point_cloud(f"./{data_folder}/t_{i}.ply")
    cropped = pcd.crop(obb)
    cropped_pcd += cropped
cropped_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
cropped_pcd = cropped_pcd.voxel_down_sample(voxel_size=0.002)
o3d.io.write_point_cloud(f"./{data_folder}/cropped_simp.ply", cropped_pcd)