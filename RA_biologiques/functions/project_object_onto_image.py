#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug  4 09:32:13 2023

@author: tinhinane
"""

import cv2
import open3d as o3d
import numpy as np

def project_object_onto_image(obj,image, projection, h,w):
    for face in obj.faces:
        vertex_indices= face[0]
        points= np.array([obj.vertices[vertex_index-1] for vertex_index in vertex_indices])
        
        projected_points= np.dot(projection, np.vstack([points.T, np.ones(len(points))]))
        projected_points= projected_points.T [:,:2]/ projected_points.T[:,2:]
        projected_points[:,1]= h-projected_points[:,1]
        
        projected_points= projected_points.astype(int)
        
        
        for i in range (len(projected_points)):
            cv2.line(image, tuple(projected_points[i]), tuple(projected_points[(i+1)% len (projected_points)]), (255,0,0),1)
            
    return image
        