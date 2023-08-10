import open3d as o3d
import numpy as np
import math

def distance_max(model):
    """
This function calculates the maximum distance between points in a point cloud.

:param model: The filename of the point cloud in PLY format.
:return: The maximum distance between points in the point cloud.
  
 """

    cloud = o3d.io.read_point_cloud(model)
    points = np.asarray(cloud.points)

    max_distance = 0.0

    for i in range(len(points)):
        for j in range(len(points)):
            dx = points[j][0] - points[i][0]
            dy = points[j][1] - points[i][1]
            dz = points[j][2] - points[i][2]
            dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            if dist > max_distance:
                max_distance = dist

    print("La distance maximale du nuage de points:", max_distance)
    return max_distance#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 21 17:56:07 2023

@author: tinhinane
"""

