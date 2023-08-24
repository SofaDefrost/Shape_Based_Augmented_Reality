import numpy as np
import copy

import open3d as o3d

from scipy.spatial.transform import Rotation as Rot


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

    euler_angle_sum = [0,0,0]

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

        print(R)
        rotation_matrix = R[:3, :3]
        rotation = Rot.from_matrix(rotation_matrix)
        euler_angles = rotation.as_euler('xyz', degrees=True)  # 'zyx' signifie que les rotations sont appliquées dans l'ordre ZYX
           
        euler_angle_sum = euler_angle_sum + euler_angles
        print(euler_angles)
        print(euler_angle_sum)

        if prev_cost - curr_cost > cost_change_threshold:
            prev_cost = curr_cost
            transform_matrix = np.hstack((R, t.T))
            transform_matrix = np.vstack((transform_matrix, np.array([0, 0, 0, 1])))
            source= source.transform(transform_matrix) # Il manque une parenthèse fermante pour la fonction multiple_icp
           
            curr_iteration += 1
        else:
            break

    return transform_matrix, curr_cost

def multiple_icp(source, target):
    # Initialize the best transformation and cost.
    best_transform_matrix = None
    best_cost = np.inf

    # Iterate through angles for each axis
    # for angle_x in range(-90, -70, 10):
    #     for angle_y in range(0, 5, 5):
    #         for angle_z in range(-100, -90, 10):
        
    for angle_x in range(-120, -80, 10):
        for angle_y in range(-20, 20, 10):
            for angle_z in range(-180, -90, 10):
                
                M_x = mf.create_rot_matrix_x(angle_x)
                M_y = mf.create_rot_matrix_y(angle_y)
                M_z = mf.create_rot_matrix_z(angle_z)
                
                # matrix = M_x @ M_y
                # transform_matrix=matrix@ M_z
                
                transform_matrix = M_z @ M_x @ M_y
                

                # print(best_transform_matrix)

                # Apply transformation to a copy of the source point cloud
                source_temp = copy.deepcopy(source)
                source_temp.transform(transform_matrix)

             
                # _, cost = icp(source_temp, target)
                _, cost = distance_between_pc(source_temp, target)

                
                # target_points = np.asarray(target.points)
                # source_temp_points = np.asarray(source_temp.points)
                # cost = np.linalg.norm(target_points - source_temp)
                
                print("We keep :")
                print([angle_x,angle_y,angle_z])
                print(cost)

        
                if cost < best_cost:
                    best_transform_matrix = transform_matrix
                    best_cost = cost
                    print(best_transform_matrix)
    
    # print("We keep finally :")
    # print(best_transform_matrix)

    return best_transform_matrix, best_cost



def run_icp_1(source_path, target_path,pc_after_multiple_icp): 
    
   """
    Function to perform Iterative Closest Point (ICP) registration between source and target point clouds
    with the objective of applying different transformation matrices to the object model
    and ultimately saving a file with minimized cost.
    
    :param source_path: Path to the source point cloud file.
    :param target_path: Path to the target point cloud file.
    :param pc_after_multiple_icp: Path to save the point cloud after applying ICP.
    :return: Best transformation matrix and an underscore placeholder.
    """

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
