#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 27 10:22:52 2023

@author: tinhinane
"""
import numpy as np
import cv2
import open3d as o3d



def recover_data(image_path):
    image = cv2.imread(image_path)
    if image is None:
        print("Erreur!! L'image ne peut pas charger.")
        return None, None
    else:
        height, width, channels = image.shape
        color_array = []
        coord_array = []
        depth_array = []

        for y in range(height):
            for x in range(width):
                color = image[y, x]
                color_list = list(color)

                color_array.append(color_list)
                coord_array.append([y, x])
                depth_array.append(image[y,x][2])
                
        color_array = np.array(color_array)
        coord_array = np.array(coord_array)
        depth_array = np.array(depth_array)
        
        
        return color_array, coord_array, depth_array

image= "cropped_Image.png"
color_array, coord_array,depth_array = recover_data(image)

if color_array is not None and coord_array is not None:
    print("Coordonnées des couleurs (RGB) des pixels de l'image:")
    print(color_array)
    print("Coordonnées des pixels de l'image:")
    print(coord_array)

# Charger le nuage de points
point_cloud = o3d.io.read_point_cloud("foie.ply")
#o3d.visualization.draw_geometries([point_cloud])

pc_points = np.asarray(point_cloud.points)
print("Nuage de points (coordonnées XYZ):")
print(pc_points)
pc_points[:,:2]=coord_array[:,::-1]




pc_colors = np.asarray(point_cloud.colors)
print("Couleurs associées au nuage de points (RGB):")
print(pc_colors)
pc_colors[:,:3]=color_array/255.0

pc_points[:,2]=depth_array

point_cloud.points=o3d.utility.Vector3dVector(pc_points)
point_cloud.colors=o3d.utility.Vector3dVector(pc_colors)



o3d.visualization.draw_geometries([point_cloud])

