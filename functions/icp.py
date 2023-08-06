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
    o3d.visualization.draw_geometries([source_temp, target_temp], zoom=2, front=[0.5, -0.1, -0.1], lookat=[0, 0, 0], up=[-0.3402, -0.9189, -0.1996])

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
    Fonction principale pour l'algorithme ICP (Iterative Closest Point).

    :param source: Le nuage de points source (objet open3d.geometry.PointCloud).
    :param target: Le nuage de points cible (objet open3d.geometry.PointCloud).

    :return: La matrice de transformation finale (4x4 numpy array) pour aligner le nuage de points source sur le cible.
    """
    # Initialisation des couleurs des nuages de points pour la visualisation.
    source.paint_uniform_color([0.5, 0.5, 0.5])
    target.paint_uniform_color([0, 0, 1])
    target_points = np.asarray(target.points)
    
    # Initialisation des variables pour le suivi des itérations et du coût.
    curr_iteration = 0
    cost_change_threshold = 0.001
    curr_cost = 1000
    prev_cost = 10000
    Mat_90 = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])

    while True:
        # Trouver les points les plus proches du nuage de points source pour chaque point du nuage de points cible.
        new_source_points = find_nearest_neighbors(source, target, 1)

        # Calculer les centroïdes des nuages de points source transformé et cible.
        source_centroid = np.mean(new_source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)

        # Calculer les positions relatives des points par rapport à leurs centroïdes respectifs.
        source_repos = np.asarray([new_source_points[ind] - source_centroid for ind in range(len(new_source_points))])
        target_repos = np.asarray([target_points[ind] - target_centroid for ind in range(len(target_points))])

        # Calculer la matrice de covariance et effectuer la décomposition SVD pour obtenir la meilleure rotation.
        cov_mat = target_repos.transpose() @ source_repos
        U, X, Vt = np.linalg.svd(cov_mat)
        R = U @ Vt

        # Calculer la translation optimale.
        t = target_centroid - R @ source_centroid
        t = np.reshape(t, (1, 3))

        # Calculer le coût actuel de l'itération.
        curr_cost = np.linalg.norm(target_repos - (R @ source_repos.T).T)
        print("Curr_cost=", curr_cost)

        # Vérifier si la convergence est atteinte (différence de coût inférieure au seuil).
        if ((prev_cost - curr_cost) > cost_change_threshold):
            prev_cost = curr_cost
            # Mettre à jour la matrice de transformation avec la rotation et la translation.
            transform_matrix_target = np.hstack((np.eye(3), t.T))
            transform_matrix_target = np.vstack((transform_matrix_target, np.array([0, 0, 0, 1])))
            # Appliquer la transformation au nuage de points source.
            source.transform(transform_matrix_target)
            curr_iteration += 1
            transform_matrix_total = transform_matrix_target @ Mat_90
        else:
            break

    print("\nIteration=", curr_iteration)
    # Visualiser le résultat final de l'alignement.
    # draw_registration_result(source, target, transform_matrix_target)
    # Renvoyer la matrice de transformation finale.
    return transform_matrix_target, curr_cost

def multiple_icp(source, target):
    # Create a copy of the source point cloud for each initial transformation.
    source_temp = copy.deepcopy(source)

    # List of transformation matrices and costs for each initial transformation.
    transformations = []
    costs = []

    # Uncomment and complete the following transformations if needed.

    # # First transformation: Initial translation.
    transform_matrix = np.asarray([[0.862, 0.011, -0.507, 0.5], [-0.139, 0.967, -0.215, 0.7], [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    source_temp.transform(transform_matrix)
    best_transform_matrix, best_cost = icp(source_temp, target)
    transformations.append(best_transform_matrix)
    costs.append(best_cost)

    # # Second transformation: Rotation around X-axis (with 45 degrees).
    angle = np.radians(135)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    best_transform_matrix, best_cost = icp(source_temp, target)
    transformations.append(best_transform_matrix)
    costs.append(best_cost)

    # # Third transformation: Rotation around Y-axis (with 45 degrees).
    angle = np.radians(165)
    transform_matrix = np.asarray([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]])
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    best_transform_matrix,best_cost = icp(source_temp, target)
    transformations.append(best_transform_matrix)
    costs.append(best_cost)

    # Fourth transformation: Rotation around Z-axis (with 90 degrees).
    angle = np.radians(90)
    transform_matrix = np.asarray([[np.cos(angle), -np.sin(angle), 0, 0], [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    best_transform_matrix, best_cost = icp(source_temp, target)
    transformations.append(best_transform_matrix)
    costs.append(best_cost)
    
    # # Fifth transformation: Rotation around X-axis (with 180 degrees).
    angle = np.radians(45)
    transform_matrix = np.asarray([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]])
    source_temp = copy.deepcopy(source)
    source_temp.transform(transform_matrix)
    best_transform_matrix, best_cost = icp(source_temp, target)
    transformations.append(best_transform_matrix)
    costs.append(best_cost)

    # Select the best transformation with the lowest cost.
    best_idx = np.argmin(costs)
    best_transform_matrix = transformations[best_idx]
    best_cost = costs[best_idx]
    
    source.transform(best_transform_matrix)

    # Visualize the final alignment result.
    draw_registration_result(source, target, best_transform_matrix)


    # Save the source point cloud corresponding to the best transformation.
    # best_source_pc = source
    # best_source_pc.transform(best_transform_matrix)
    # o3d.io.write_point_cloud("data_exemple/model_3D_icpp.ply", best_source_pc)
    return best_transform_matrix, best_cost


def run_icp_1(source_path, target_path):  # Pass the file paths as arguments
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)
    # Call the multiple_icp function to align the source and target point clouds.
    best_transform_matrix = multiple_icp(source, target)
    # source_transformed = copy.deepcopy(source)

    # source_transformed.transform(best_transform_matrix)
    # # output_file_path = "data_exemple/source_transformed_icp.ply"
    # # # o3d.io.write_point_cloud(output_file_path, source_transformed)
    # print("The best transformation matrix is", best_transform_matrix)
    return best_transform_matrix

def run_icp_2(source_path, target_path):  # Pass the file paths as arguments
    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)
    # Call the icp function to align the source and target point clouds.
    transform_matrix = icp(source, target)
    print(transform_matrix)
    return transform_matrix



def extract_rotation_angles(transform_matrix):
    """
    Fonction pour extraire les angles de rotation à partir d'une matrice de transformation 4x4.

    :param transform_matrix: La matrice de transformation (4x4 numpy array).

    :return: Les angles de rotation autour des axes X, Y et Z (en degrés).
    """
    # Extraction de la rotation en radians
    rx = np.arctan2(transform_matrix[2, 1], transform_matrix[2, 2])
    ry = np.arctan2(-transform_matrix[2, 0], np.sqrt(transform_matrix[2, 1]**2 + transform_matrix[2, 2]**2))
    rz = np.arctan2(transform_matrix[1, 0], transform_matrix[0, 0])

    # Conversion en degrés
    rx_deg = np.degrees(rx)
    ry_deg = np.degrees(ry)
    rz_deg = np.degrees(rz)

    return rx_deg, ry_deg, rz_deg

# Exemple d'utilisation :
