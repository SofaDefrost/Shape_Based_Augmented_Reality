"""
Generate surface mesh from point cloud
"""
import pymeshlab

point_cloud_file = "./output.ply"
ms = pymeshlab.MeshSet()
ms.load_new_mesh(point_cloud_file)

ms.compute_normal_for_point_clouds()

ms.generate_simplified_point_cloud(1000)

ms.generate_surface_reconstruction_screened_poisson()

# ms.meshing_invert_face_orientation()

# ms.meshing_close_holes()

ms.save_current_mesh("output_surface.ply")
