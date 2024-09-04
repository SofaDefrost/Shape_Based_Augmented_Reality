"""
Get volume from surface mesh
reference:
https://stackoverflow.com/questions/1406029/how-to-calculate-the-volume-of-a-3d-mesh-object-the-surface-of-which-is-made-up
"""

import pymeshlab
import numpy as np

surface_mesh_file = "output_surface.ply"


def signed_volume_of_triangle(point1, point2, point3):
    v321 = point3[0] * point2[1] * point1[2]
    v231 = point2[0] * point3[1] * point1[2]
    v312 = point3[0] * point1[1] * point2[2]
    v132 = point1[0] * point3[1] * point2[2]
    v213 = point2[0] * point1[1] * point3[2]
    v123 = point1[0] * point2[1] * point3[2]
    return (1.0 / 6.0) * (-v321 + v231 + v312 - v132 - v213 + v123)


ms = pymeshlab.MeshSet()
ms.load_new_mesh(surface_mesh_file)

vertex = ms.current_mesh().vertex_matrix()
faces = ms.current_mesh().face_matrix()

vols = 0
for triangule in faces:
    p1 = vertex[triangule[0]]
    p2 = vertex[triangule[1]]
    p3 = vertex[triangule[2]]
    vols += signed_volume_of_triangle(p1, p2, p3)

print(f"volume = {np.abs(vols)}")
