import open3d as o3d
import numpy as np
import copy


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw([source_temp, target_temp])


def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)
    # pcd_down = pcd

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100),
    )
    return pcd_down, pcd_fpfh


def prepare_dataset(source, target, voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    trans_init = np.asarray(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    source.transform(trans_init)
    # draw_registration_result(source, target, np.identity(4))
    print("start pre-process")
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    print("finish pre-process")
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(
    source_down, target_down, source_fpfh, target_fpfh, voxel_size
):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3,
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold
            ),
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(500000, 0.999),
    )
    return result


def execute_fast_global_registration(
    source_down, target_down, source_fpfh, target_fpfh, voxel_size
):
    distance_threshold = voxel_size * 0.5
    print(
        ":: Apply fast global registration with distance threshold %.3f"
        % distance_threshold
    )
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold
        ),
    )
    return result


def point_to_point_icp(source, target, threshold, trans_init):
    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source,
        target,
        threshold,
        trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    )
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation, "\n")
    # draw_registration_result(source, target, reg_p2p.transformation)
    return reg_p2p


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        distance_threshold,
        result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    return result


def get_filtered_points_by_clustering(pcd):
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True)
        )

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    unique_index = np.unique(labels)
    max_num = 0
    max_label = -1
    for i in unique_index:
        if i == -1:
            continue
        index = np.where(labels == i)
        num = len(index[0])
        if num > max_num:
            max_num = num
            max_label = i

    print(f"largest cluster has {max_num} points")

    index = np.where(labels == max_label)
    p = pcd.select_by_index(indices=index[0])
    # o3d.visualization.draw_geometries([p])
    return p


voxel_size = 0.005

data_folder = "data-1712"

target = o3d.io.read_point_cloud(f"./{data_folder}/output_0.ply")
source = o3d.io.read_point_cloud(f"./{data_folder}/output_1.ply")

# source = get_filtered_points_by_clustering(source)
# target = get_filtered_points_by_clustering(target)

source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
    source, target, voxel_size
)

result_ransac = execute_global_registration(
    source_down, target_down, source_fpfh, target_fpfh, voxel_size
)
draw_registration_result(source_down, target_down, result_ransac.transformation)

threshold = 0.00001
result_icp = point_to_point_icp(source, target, threshold, result_ransac.transformation)
draw_registration_result(source, target, result_icp.transformation)

o3d.io.write_point_cloud(f"./{data_folder}/gr/t_0.ply", target)
o3d.io.write_point_cloud(f"./{data_folder}/gr/t_1.ply", source.transform(result_icp.transformation))

for i in range(1, 25):
    print(f"processing t{i} and output_{i+1}")
    target = o3d.io.read_point_cloud(f"./{data_folder}/gr/t_{i}.ply")
    source = o3d.io.read_point_cloud(f"./{data_folder}/output_{i+1}.ply")
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, voxel_size)
    

    result_ransac = execute_global_registration(
        source_down, target_down, source_fpfh, target_fpfh, voxel_size
    )
    # draw_registration_result(source_down, target_down, result_ransac.transformation)

    result_icp = point_to_point_icp(source, target, threshold, result_ransac.transformation)
    # draw_registration_result(source, target, result_icp.transformation)

    # o3d.io.write_point_cloud(f"t_{i}.ply", target)
    o3d.io.write_point_cloud(
        f"./{data_folder}/gr/t_{i+1}.ply", source.transform(result_icp.transformation)
    )
