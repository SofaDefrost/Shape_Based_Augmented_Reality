import numpy as np
import copy

import open3d as o3d
import functions.matrix_function as mf
import functions.repose as rep
import functions.transformations as tf
from scipy.spatial.transform import Rotation as Rot

def draw_two_pc(source, target):
    """
    Function to visualize the result of the alignment between the source point cloud and the target point cloud.

    :param source: The source point cloud (open3d.geometry.PointCloud object).
    :param target: The target point cloud (open3d.geometry.PointCloud object).
    :param transformation: The transformation matrix (4x4 numpy array) to align the source point cloud to the target.
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])

    def key_callback(vis, key):
        # Check if the pressed key is 'Q'
        if key == ord('q'):
            vis.close()  # Close the visualization window

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    vis.add_geometry(source_temp)
    vis.add_geometry(target_temp)
    vis.register_key_callback(ord('q'), key_callback)
    vis.run()
    vis.destroy_window()


def draw_registration_result(source, target, transformation):
    """
    Function to visualize the result of the alignment between the source point cloud and the target point cloud.

    :param source: The source point cloud (open3d.geometry.PointCloud object).
    :param target: The target point cloud (open3d.geometry.PointCloud object).
    :param transformation: The transformation matrix (4x4 numpy array) to align the source point cloud to the target.
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    def key_callback(vis, key):
        # Check if the pressed key is 'Q'
        if key == ord('q'):
            vis.close()  # Close the visualization window

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    vis.add_geometry(source_temp)
    vis.add_geometry(target_temp)
    vis.register_key_callback(ord('q'), key_callback)
    vis.run()
    vis.destroy_window()


import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree

def find_nearest_neighbors(source_pc, target_pc, nearest_neigh_num):
    """
    Function to find the nearest points in the source point cloud for each point in the target point cloud.

    :param source_pc: The source point cloud (open3d.geometry.PointCloud object).
    :param target_pc: The target point cloud (open3d.geometry.PointCloud object).
    :param nearest_neigh_num: The number of nearest points to search for each target point.

    :return: A numpy array containing the nearest points from the source point cloud for each point in the target point cloud.
    """
    source_points = np.asarray(source_pc.points)
    target_points = np.asarray(target_pc.points)

    # Create a KD-Tree from the source points
    source_kdtree = cKDTree(source_points)

    # Find nearest neighbors for each point in the target point cloud
    distances, indices = source_kdtree.query(target_points, k=nearest_neigh_num)

    # Retrieve the nearest neighbor points
    nearest_neighbors = source_points[indices]

    return nearest_neighbors

    # point_cloud_tree = o3d.geometry.KDTreeFlann(source_pc)
    # points_arr = []
    # for point in target_pc.points:
    #     [_, idx, _] = point_cloud_tree.search_knn_vector_3d(point, nearest_neigh_num)
    #     points_arr.append(source_pc.points[idx[0]])
    # return np.asarray(points_arr)

def quick_icp(source, target):
    
    """
Iterative Closest Point (ICP) Alignment Function

This function performs Iterative Closest Point (ICP) alignment between a source point cloud
and a target point cloud. It aims to find the transformation matrix that aligns the source
cloud with the target cloud by minimizing the distance between their corresponding points.

Parameters:
    source (o3d.geometry.PointCloud): The source point cloud.
    target (o3d.geometry.PointCloud): The target point cloud.

Returns:
    transform_matrix (numpy.ndarray): The transformation matrix that aligns the source cloud with the target cloud.
    curr_cost (float): The final cost of the alignment after convergence.
"""
    source.paint_uniform_color([0.5, 0.5, 0.5])
    target.paint_uniform_color([0, 0, 1])
    target_points = np.asarray(target.points)

    transform_matrix = np.asarray([[0.862, 0.011, -0.507, 0.5], [-0.139, 0.967, -0.215, 0.7], [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    source.transform(transform_matrix)

    curr_iteration = 0
    cost_change_threshold = 0.001
    curr_cost = 1000
    prev_cost = 10000

    
    new_source_points = find_nearest_neighbors(source, target, 1) # Il manque la définition de la fonction find_nearest_neighbors

    source_centroid = np.mean(new_source_points, axis=0)
    target_centroid = np.mean(target_points, axis=0)
    source_repos = np.asarray([new_source_points[ind] - source_centroid for ind in range(len(new_source_points))])
    target_repos = np.asarray([target_points[ind] - target_centroid for ind in range(len(target_points))])

    cov_mat = target_repos.transpose() @ source_repos

    U, X, Vt = np.linalg.svd(cov_mat)
    R = U @ Vt
    t = target_centroid - R @ source_centroid
    t = np.reshape(t, (1, 3))
    curr_cost = np.linalg.norm(target_repos - (R @ source_repos.T).T)
       
    print(curr_cost)
    
    draw_registration_result(source, target, transform_matrix)
    return transform_matrix, curr_cost


def icp(source, target):
    
    """
Iterative Closest Point (ICP) Alignment Function

This function performs Iterative Closest Point (ICP) alignment between a source point cloud
and a target point cloud. It aims to find the transformation matrix that aligns the source
cloud with the target cloud by minimizing the distance between their corresponding points.

Parameters:
    source (o3d.geometry.PointCloud): The source point cloud.
    target (o3d.geometry.PointCloud): The target point cloud.

Returns:
    transform_matrix (numpy.ndarray): The transformation matrix that aligns the source cloud with the target cloud.
    curr_cost (float): The final cost of the alignment after convergence.
"""
    target_points = np.asarray(target.points)
    
    transform_matrix = np.identity(4)
    
    source_temp = copy.deepcopy(source)

    source_temp.transform(transform_matrix)

    curr_iteration = 0
    cost_change_threshold = 0.001
    curr_cost = 1000
    prev_cost = 10000
    euler_angle_sum = [0,0,0]
    transform_matrix_cumulee=np.identity(4)
    while True:
        new_source_points = find_nearest_neighbors(source_temp, target, 1)

        source_centroid = np.mean(new_source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)
        source_repos = np.asarray([new_source_points[ind] - source_centroid for ind in range(len(new_source_points))])
        target_repos = np.asarray([target_points[ind] - target_centroid for ind in range(len(target_points))])

        cov_mat = target_repos.transpose() @ source_repos
        U, X, Vt = np.linalg.svd(cov_mat)
        R = U @ Vt
        t = target_centroid - R @ source_centroid
        t = np.reshape(t, (1, 3))
        curr_cost = np.linalg.norm(target_repos - (R @ source_repos.T).T)

        rotation_matrix = R[:3, :3]
        rotation = Rot.from_matrix(rotation_matrix)
        euler_angles = rotation.as_euler('xyz', degrees=True)  # 'zyx' signifie que les rotations sont appliquées dans l'ordre ZYX
           
        # euler_angle_sum = euler_angle_sum + euler_angles # for debug
        # print(euler_angles)
        # print(euler_angle_sum)
        if prev_cost - curr_cost > cost_change_threshold:
            prev_cost = curr_cost
            transform_matrix = np.hstack((R, t.T))
            transform_matrix = np.vstack((transform_matrix, np.array([0, 0, 0, 1])))
            source_temp= source_temp.transform(transform_matrix) 
            curr_iteration += 1
            transform_matrix_cumulee=transform_matrix_cumulee @ transform_matrix
        else:
            transform_matrix = np.hstack((R, t.T))
            transform_matrix = np.vstack((transform_matrix, np.array([0, 0, 0, 1])))
            source_temp= source_temp.transform(transform_matrix) 
            break

    return transform_matrix_cumulee, curr_cost

import numpy as np

def weighted_average_euclidean_distance(list_a, list_b):
    # Si les listes ont des dimensions différentes, ajustez-les pour avoir la même dimension en ajoutant des zéros
    if len(list_a) > len(list_b):
        num_zeros = len(list_a) - len(list_b)
        zero_array = np.zeros((num_zeros, len(list_b[0])))
        list_b = np.concatenate((list_b, zero_array))
    elif len(list_b) > len(list_a):
        num_zeros = len(list_b) - len(list_a)
        zero_array = np.zeros((num_zeros, len(list_a[0])))
        list_a = np.concatenate((list_a, zero_array))
        
    # Calculer les normes de chaque élément de list_b
    norms_b = np.linalg.norm(list_b, axis=1)
    
    total_distance = 0
    for coord_a in list_a:
        # Calculer la distance euclidienne entre coord_a et tous les éléments de list_b
        distances = np.linalg.norm(list_b - coord_a, axis=1)
        
        # Sélectionner la distance minimale pour coord_a
        min_distance = np.min(distances)
        
        total_distance += min_distance

    weighted_avg_distance = total_distance / len(list_a)
    return weighted_avg_distance


def ply_to_points_and_colors(file_path):
    # prend un fichier .ply et le converti en nuage de points et en couleur format RGB entre 0 et 255
    ply_data = o3d.io.read_point_cloud(file_path)
    points = np.array(ply_data.points)
    colors = np.array(ply_data.colors)* 255

    return points, colors

def find_the_best_pre_rotation(source_path, target_path):
    target,_ = ply_to_points_and_colors(target_path)
    source,_=ply_to_points_and_colors(source_path)

    # Initialize the best cost.
    best_cost = np.inf
    print("If you want faster results (for example: for tests) you can change the range of the angles (in multiple_icp)")
    best_angles=[]
    for angle_x in range(0, 10, 10): # En théorie il faut mettre -180, 180 (doit parcourir toutes les positions possibles)
        for angle_y in range(0, 10, 10): # Idem
            for angle_z in range(-180, 180, 20): # Idem
                print([angle_x,angle_y,angle_z])
                M_x = tf.rotation_matrix_x(np.radians(angle_x))
                M_y = tf.rotation_matrix_y(np.radians(angle_y))
                M_z = tf.rotation_matrix_z(np.radians(angle_z))
                          
                transform_matrix = M_x @ M_y @ M_z 
                
                source_rotated=[np.dot(point,transform_matrix) for point in source]
                
                cost = weighted_average_euclidean_distance(source_rotated, target)
                if cost < best_cost:
                    best_transform_matrix = transform_matrix
                    best_cost = cost
                    best_angles=[angle_x,angle_y,angle_z]
    
    print("We keep finally :")
    print(best_transform_matrix)
    print("Best angles :")
    print(best_angles)
    print("With a cost of", best_cost)    
    return best_transform_matrix

def run_icp(source_path, target_path): 
    
    """
   Function to perform Iterative Closest Point (ICP) registration between source and target point clouds.
   
   :param source_path: Path to the source point cloud file.
   :param target_path: Path to the target point cloud file.
   :return: Transformation matrix for alignment and an underscore placeholder.
   """
    
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)
    # Call the icp function to align the source and target point clouds.
    transform_matrix,_ = icp(source, target)
    draw_registration_result(source, target, transform_matrix)
    return transform_matrix, _
