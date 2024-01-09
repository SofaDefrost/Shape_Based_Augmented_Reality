import numpy as np
import sys

mod_name = vars(sys.modules[__name__])['__package__']
if mod_name:
    # Code executed as a module
    from . import project_and_display as proj
    from . import transformations as tf
else:
    # Code executed as a script
    import project_and_display as proj
    import transformations as tf

from scipy.spatial.transform import Rotation as Rot


def find_transform_matrix_to_align_points_using_icp(source_points, target_points):
    """
    Find the transformation matrix to align source points to target points using the ICP (Iterative Closest Point) algorithm.

    Args:
    - source_points (np.ndarray): Source point cloud.
    - target_points (np.ndarray): Target point cloud.

    Returns:
    - Tuple[np.ndarray, float]: Tuple containing the cumulative transformation matrix and the final cost of alignment.
    """    
    transform_matrix = np.identity(4)
    
    source_temp = source_points

    source_temp_shaped = np.column_stack((source_temp, np.ones(len(
    source_temp))))

    source_points= np.array([(float(x), float(y), float(z)) for (
    x, y, z,t) in [transform_matrix @ p for p in source_temp_shaped]], dtype=np.float64)
    
    curr_iteration = 0
    cost_change_threshold = 0.001
    curr_cost = 1000
    prev_cost = 10000
    transform_matrix_cumulee=np.identity(4)
    while True:
        new_source_points,_ = proj.find_nearest_neighbors(source_temp, target_points, 1)

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

        if prev_cost - curr_cost > cost_change_threshold:
            prev_cost = curr_cost
            transform_matrix = np.hstack((R, t.T))
            transform_matrix = np.vstack((transform_matrix, np.array([0, 0, 0, 1])))
            source_temp_shaped = np.column_stack((source_temp, np.ones(len(source_temp))))
            source_points= np.array([(float(x), float(y), float(z)) for (x, y, z,t) in [transform_matrix @ p for p in source_temp_shaped]], dtype=np.float64)
            curr_iteration += 1
            transform_matrix_cumulee=transform_matrix_cumulee @ transform_matrix
        else:
            transform_matrix = np.hstack((R, t.T))
            transform_matrix = np.vstack((transform_matrix, np.array([0, 0, 0, 1])))
            source_temp_shaped = np.column_stack((source_temp, np.ones(len(source_temp))))
            source_points= np.array([(float(x), float(y), float(z)) for (x, y, z,t) in [transform_matrix @ p for p in source_temp_shaped]], dtype=np.float64)
            break

    return transform_matrix_cumulee, curr_cost


def weighted_average_euclidean_distance(array_a:np.ndarray, array_b:np.ndarray)->float:
    """
    Calculate the weighted average Euclidean distance between two lists of 3D coordinates.

    Args:
    - array_a (np.ndarray): First list of 3D coordinates.
    - array_b (np.ndarray): Second list of 3D coordinates.

    Returns:
    - float: Weighted average Euclidean distance.
    """
    # If the lists have different dimensions, adjust them to have the same dimension by adding zeros
    if len(array_a) > len(array_b):
        num_zeros = len(array_a) - len(array_b)
        zero_array = np.zeros((num_zeros, len(array_b[0])))
        array_b = np.concatenate((array_b, zero_array))
    elif len(array_b) > len(array_a):
        num_zeros = len(array_b) - len(array_a)
        zero_array = np.zeros((num_zeros, len(array_a[0])))
        array_a = np.concatenate((array_a, zero_array))
        
    total_distance = 0
    for coord_a in array_a:
        # Calculate the Euclidean distance between coord_a and all elements in array_b
        distances = np.linalg.norm(array_b - coord_a, axis=1)
        
        # Select the minimum distance for coord_a
        min_distance = np.min(distances)
        
        total_distance += min_distance

    weighted_avg_distance = total_distance / len(array_a)
    return weighted_avg_distance



def find_the_best_pre_rotation_to_align_points(points_source:np.ndarray, points_target:np.ndarray)->np.ndarray:
    """
    Find the best pre-rotation matrix to align a source set of points to a target set of points.

    Args:
    - points_source (np.ndarray): Source set of 3D coordinates.
    - points_target (np.ndarray): Target set of 3D coordinates.

    Returns:
    - np.ndarray: Best pre-rotation matrix.
    """
    # Initialize the best cost.
    best_cost = np.inf
    for angle_x in range(0, 10, 10): # In theory, it should be -180, 180 (should cover all possible positions)
        for angle_y in range(0, 10, 10): # Idem
            for angle_z in range(-180, 180, 10): # Idem
                print([angle_x,angle_y,angle_z])
                M_x = tf.rotation_matrix_x(np.radians(angle_x))
                M_y = tf.rotation_matrix_y(np.radians(angle_y))
                M_z = tf.rotation_matrix_z(np.radians(angle_z))
                          
                transform_matrix = M_x @ M_y @ M_z 
                
                source_rotated=np.array([np.dot(point,transform_matrix) for point in points_source])
                
                cost = weighted_average_euclidean_distance(source_rotated, points_target)
                if cost < best_cost:
                    best_transform_matrix = transform_matrix
                    best_cost = cost
                                           
    return best_transform_matrix

