import numpy as np
import copy

import open3d as o3d

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
    o3d.visualization.draw_geometries([source_temp, target_temp], zoom=1, front=[0.6, -0.1, -0.1], lookat=[0, 0, 0], up=[-0.3402, -0.222, -0.1996])



def find_nearest_neighbors(source_pc, target_pc, nearest_neigh_num):
    """
    Function to find the nearest points in the source point cloud for each point in the target point cloud.

    :param source_pc: The source point cloud (open3d.geometry.PointCloud object).
    :param target_pc: The target point cloud (open3d.geometry.PointCloud object).
    :param nearest_neigh_num: The number of nearest points to search for each target point.

    :return: A numpy array containing the nearest points from the source point cloud for each point in the target point cloud.
    """
    point_cloud_tree = o3d.geometry.KDTreeFlann(source_pc)
    points_arr = []
    for point in target_pc.points:
        [_, idx, _] = point_cloud_tree.search_knn_vector_3d(point, nearest_neigh_num)
        points_arr.append(source_pc.points[idx[0]])
    return np.asarray(points_arr)

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
    source.paint_uniform_color([0.5, 0.5, 0.5])
    target.paint_uniform_color([0, 0, 1])
    target_points = np.asarray(target.points)

    transform_matrix = np.asarray([[0.862, 0.011, -0.507, 0.5], [-0.139, 0.967, -0.215, 0.7], [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    source.transform(transform_matrix)

    curr_iteration = 0
    cost_change_threshold = 0.001
    curr_cost = 1000
    prev_cost = 10000

    while True:
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
       

        if prev_cost - curr_cost > cost_change_threshold:
            prev_cost = curr_cost
            transform_matrix = np.hstack((R, t.T))
            transform_matrix = np.vstack((transform_matrix, np.array([0, 0, 0, 1])))
            source= source.transform(transform_matrix)# Il manque une parenthèse fermante pour la fonction multiple_icp
           
            curr_iteration += 1
        else:
            break

    
    
    #draw_registration_result(source, target, transform_matrix) 
   
    return transform_matrix, curr_cost




def multiple_icp(source, target):
    """
Multiple ICP Alignments Function

This function performs multiple ICP alignment attempts on a source point cloud with a target point cloud.
It tries different initial transformations and selects the best transformation that produces the lowest alignment cost.

Parameters:
    source (o3d.geometry.PointCloud): The source point cloud.
    target (o3d.geometry.PointCloud): The target point cloud.

Returns:
    best_transform_matrix (numpy.ndarray): The transformation matrix that yields the best alignment.
    best_cost (float): The cost associated with the best alignment.
"""
    # Create a copy of the source point cloud for each initial transformation.
    source_temp = copy.deepcopy(source)

    # Initialize the best transformation and cost.
    best_transform_matrix = None
    best_cost = np.inf
    
    # Matrice de transformation de 90° selon X
    angle = np.radians(-90)
    Mat_90 = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
    
    # Transformations: Rotation around X-axis.
    angle = np.radians(90)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
    
    angle = np.radians(30)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
    
    angle = np.radians(-90)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
   
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
    
    angle = np.radians(120)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
        
    angle = np.radians(-120)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
          best_transform_matrix = transform_matrix
          best_cost = cost   
         
    angle = np.radians(180)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    if cost < best_cost:
          best_transform_matrix = transform_matrix
          best_cost = cost   
   
    angle = np.radians(-180)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
          best_transform_matrix = transform_matrix
          best_cost = cost   
    
    
    # Transformation: Rotation around Y-axis.
    angle = np.radians(90)
    transform_matrix = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
    
    angle = np.radians(-90)
    transform_matrix = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
    
    angle = np.radians(180)
    transform_matrix = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
   
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
        
    angle = np.radians(-180)
    transform_matrix = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
    
    angle = np.radians(45)
    transform_matrix = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
    
    angle = np.radians(-45)
    transform_matrix = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost 
        
        
        
    # Transformations: Rotation around Z-axis 
    angle = np.radians(90)
    transform_matrix = np.asarray([[np.cos(angle), -np.sin(angle), 0, 0], [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
        
    angle = np.radians(120)
    transform_matrix = np.asarray([[np.cos(angle), -np.sin(angle), 0, 0], [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
          best_transform_matrix = transform_matrix
          best_cost = cost
         
    angle = np.radians(255)
    transform_matrix = np.asarray([[np.cos(angle), -np.sin(angle), 0, 0], [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
      best_transform_matrix = transform_matrix
      best_cost = cost     

    #  transformations: Rotation around X-axis
    angle = np.radians(-45)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
    
    angle = np.radians(132)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost
        
        
    angle = np.radians(-145)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
   
    if cost < best_cost:
          best_transform_matrix = transform_matrix
          
          best_cost = cost
     

    angle = np.radians(120)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        
        best_cost = cost
        
    angle = np.radians(60)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]]) 
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    _, cost = icp(source_temp, target)
    
    if cost < best_cost:
        best_transform_matrix = transform_matrix
        best_cost = cost

    
   
    
    best = best_transform_matrix 
    # Save the source point cloud corresponding to the best transformation.
    best_source_pc = copy.deepcopy(source)
    best_source_pc=best_source_pc.transform(best)
    source=source.transform(best_transform_matrix)
    # Visualize the final alignment result with the best transformation.
    # draw_registration_result(source, target, best_transform_matrix)
    # Visualize the final alignment result with the best transformation.
    # draw_registration_result(  best_source_pc,target, best)
    
    return best, best_cost



def run_icp_1(source_path, target_path,pc_after_multiple_icp): 
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)
    # Call the multiple_icp function to align the source and target point clouds.
    best_transform_matrix,_ = multiple_icp(source, target)
    best = best_transform_matrix 
    # Save the source point cloud corresponding to the best transformation.
    best_source_pc = copy.deepcopy(source)
    best_source_pc=best_source_pc.transform(best)
   
    o3d.io.write_point_cloud(pc_after_multiple_icp, best_source_pc)
    return best_transform_matrix, _

def run_icp_2(source_path, target_path): 
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)
    # Call the icp function to align the source and target point clouds.
    transform_matrix,_ = icp(source, target)
    draw_registration_result(source, target, transform_matrix)
    return transform_matrix, _
