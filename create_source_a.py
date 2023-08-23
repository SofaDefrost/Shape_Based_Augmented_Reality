#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 22 18:03:55 2023

@author: tinhinane
"""
import open3d as o3d
import numpy as np
import copy
import functions.matrix_function as mf
import os

source = o3d.io.read_point_cloud("./data_exemple/FleurDeLisThing.ply")

transform_matrix = mf.create_rot_matrix_x(20)  # Assurez-vous que cette fonction existe dans matrix_function

source_temp = copy.deepcopy(source)
source_a = source_temp.transform(transform_matrix)

output_filename = "./data_exemple/source_b.ply"
o3d.io.write_point_cloud(output_filename, source_a)

