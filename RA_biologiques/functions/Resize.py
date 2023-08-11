#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  7 09:49:42 2023

@author: tinhinane
"""
import trimesh

# def Resize(output_file, pc_resized, scaling_factor):
#     """
#     Resize the point cloud mesh by applying a scaling factor to its vertices.

#     :param output_file: The filename of the point cloud mesh in PLY format.
#     :param pc_resized: The filename to save the resized point cloud mesh.
#     :param scaling_factor: The scaling factor to resize the vertices of the mesh.
#     """
#     # Load the PLY file and create a Trimesh object containing the point cloud data.
#     mesh = trimesh.load(output_file)
    
#     # Apply scaling by multiplying the vertices' coordinates with the scaling factor.
#     scaled_vertices = mesh.vertices * scaling_factor
    
#     # Create a new Trimesh with the scaled vertices and original faces (if available).
#     scaled_mesh = trimesh.Trimesh(scaled_vertices, faces=mesh.faces if hasattr(mesh, 'faces') else None)
    
#     # Export the resized mesh to the specified file.
#     scaled_mesh.export(pc_resized)
import trimesh

def Resize(output_file, pc_resized, scaling_factor):
    """
    Resize the point cloud mesh by applying a scaling factor to its vertices while preserving colors.

    :param output_file: The filename of the point cloud mesh in PLY format.
    :param pc_resized: The filename to save the resized point cloud mesh.
    :param scaling_factor: The scaling factor to resize the vertices of the mesh.
    """
    # Load the PLY file and create a Trimesh object containing the point cloud data.
    mesh = trimesh.load(output_file)
    
    # Extract the vertex colors from the mesh.
    vertex_colors = mesh.visual.vertex_colors
    
    # Apply scaling by multiplying the vertices' coordinates with the scaling factor.
    scaled_vertices = mesh.vertices * scaling_factor
    
    # Create a new Trimesh with the scaled vertices and original faces (if available).
    scaled_mesh = trimesh.Trimesh(
        vertices=scaled_vertices,
        faces=mesh.faces if hasattr(mesh, 'faces') else None,
        vertex_colors=vertex_colors  # Assign the preserved vertex colors.
    )
    
    # Export the resized mesh to the specified file.
    scaled_mesh.export(pc_resized)
