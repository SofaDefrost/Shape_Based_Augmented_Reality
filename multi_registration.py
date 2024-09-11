import open3d as o3d

import numpy as np
from tqdm import tqdm


def load_point_clouds(data_folder, voxel_size=0.0):
    pcds = []
    for i in range(21):
        pcd = o3d.io.read_point_cloud(f"./{data_folder}/output_{i}.ply")
        o3d.geometry.PointCloud.estimate_normals(
            pcd,
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 2, max_nn=30
            ),
        )
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds


def pairwise_registration(
    source, target, max_correspondence_distance_coarse, max_correspondence_distance_fine
):
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_correspondence_distance_coarse,
        np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    icp_fine = o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    transformation_icp = icp_fine.transformation
    information_icp = (
        o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            source, target, max_correspondence_distance_fine, icp_fine.transformation
        )
    )
    return transformation_icp, information_icp


def full_registration(
    pcds, max_correspondence_distance_coarse, max_correspondence_distance_fine
):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in tqdm(range(n_pcds)):
        for target_id in tqdm(range(source_id + 1, n_pcds)):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id],
                pcds[target_id],
                max_correspondence_distance_coarse,
                max_correspondence_distance_fine,
            )
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry))
                )
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(
                        source_id,
                        target_id,
                        transformation_icp,
                        information_icp,
                        uncertain=False,
                    )
                )
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(
                        source_id,
                        target_id,
                        transformation_icp,
                        information_icp,
                        uncertain=True,
                    )
                )
    return pose_graph


if __name__ == "__main__":
    voxel_size = 0.0001
    data_folder = "data-1712"
    pcds_down = load_point_clouds(data_folder, voxel_size)
    o3d.visualization.draw(pcds_down)
    print("Full registration ...")
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph = full_registration(
            pcds_down,
            max_correspondence_distance_coarse,
            max_correspondence_distance_fine,
        )
    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0,
    )

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option,
        )
    print("Transform points and display")

    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds_down)):
        print(pose_graph.nodes[point_id].pose)
        pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)

        pcd_combined += pcds_down[point_id]

    pcd_combined.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd_combined.voxel_down_sample(voxel_size)
    o3d.visualization.draw(pcd_combined)

    o3d.io.write_point_cloud(f"./{data_folder}/combined_pcd.ply", pcd_combined)
