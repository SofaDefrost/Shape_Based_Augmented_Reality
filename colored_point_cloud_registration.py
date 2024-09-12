import open3d as o3d
import numpy as np
import copy
import time

from glob import glob


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries(
        [source_temp, target],
        zoom=0.5,
        front=[-0.2458, -0.8088, 0.5342],
        lookat=[1.7745, 2.2305, 0.9787],
        up=[0.3109, -0.5878, -0.7468],
    )


def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    # pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down = pcd

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
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.9999),
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


def colored_registration(source, target, voxel_size):
    source, target, source_down, target_down, source_fpfh, target_fpfh = (
        prepare_dataset(source, target, voxel_size)
    )
    result_ransac = execute_fast_global_registration(
        source_down, target_down, source_fpfh, target_fpfh, voxel_size
    )
    current_transformation = result_ransac.transformation
    # current_transformation = np.identity(4)

    # colored pointcloud registration
    # This is implementation of following paper
    # J. Park, Q.-Y. Zhou, V. Koltun,
    # Colored Point Cloud Registration Revisited, ICCV 2017
    voxel_radius = [0.02, 0.01, 0.005]
    max_iter = [50, 30, 14]
    print("3. Colored point cloud registration")

    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter, radius, scale])

        print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30)
        )
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30)
        )

        print("3-3. Applying colored point cloud registration")

        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down,
            target_down,
            radius,
            current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=iter
            ),
        )
        current_transformation = result_icp.transformation

    return result_icp


# vis = o3d.visualization.Visualizer()
# vis.create_window()



data_folder = "data-1050"
print("1. Load two point clouds and show initial pose")
# demo_colored_icp_pcds = o3d.data.DemoColoredICPPointClouds()
# source = o3d.io.read_point_cloud(demo_colored_icp_pcds.paths[0])
# target = o3d.io.read_point_cloud(demo_colored_icp_pcds.paths[1])
target = o3d.io.read_point_cloud(f"./{data_folder}/output_0.ply")
source = o3d.io.read_point_cloud(f"./{data_folder}/output_1.ply")
# o3d.geometry.PointCloud.estimate_normals(target, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
# o3d.geometry.PointCloud.estimate_normals(source, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
voxel_size = 0.005
result_icp = colored_registration(source, target, voxel_size)

# draw_registration_result_original_color(source, target, result_icp.transformation)

# vis.add_geometry(target)
# vis.add_geometry(source)
# vis.poll_events()
# vis.update_renderer()

o3d.io.write_point_cloud(f"./{data_folder}/t_0.ply", target)
o3d.io.write_point_cloud(
    f"./{data_folder}/t_1.ply", source.transform(result_icp.transformation)
)

for i in range(1, 5):
    print(f"processing t{i} and output_{i+1}")
    target = o3d.io.read_point_cloud(f"./{data_folder}/t_{i}.ply")
    source = o3d.io.read_point_cloud(f"./{data_folder}/output_{i+1}.ply")

    result_icp = colored_registration(source, target, voxel_size)

    # draw_registration_result_original_color(source, target,
    #                                         result_icp.transformation)
    
    # vis.add_geometry(source)
    # vis.poll_events()
    # vis.update_renderer()
    
    o3d.io.write_point_cloud(
        f"./{data_folder}/t_{i+1}.ply", source.transform(result_icp.transformation)
    )



# vis.destroy_window()