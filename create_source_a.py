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
import functions.translation_m as tm

## Choix de la source

# source = o3d.io.read_point_cloud("./data_exemple/FleurDeLisThing.ply")
# source = o3d.io.read_point_cloud("./data_exemple/fleur_18.ply")
# source = o3d.io.read_point_cloud("./data_exemple/fleur_18_reposed.ply")

name_3D="data_exemple/model_3D"
model_3D_resized_name =name_3D + '_resized.ply'
source = o3d.io.read_point_cloud(model_3D_resized_name)

rot_matrix = mf.create_rot_matrix_z(-20) @ mf.create_rot_matrix_x(0) @ mf.create_rot_matrix_y(0) # 14
translation_vector = [0.04029179829605759, -0.007695348260786475, -0.11342572491003634]

# rot_matrix = mf.create_rot_matrix_z(20) @ mf.create_rot_matrix_x(-90) @ mf.create_rot_matrix_y(0) # 3
# translation_vector = [-0.04889121642266511, 0.025976369692733984, -0.1374194654601216]

# rot_matrix = mf.create_rot_matrix_z(-100) @ mf.create_rot_matrix_x(-80) @ mf.create_rot_matrix_y(0) # 18
# translation_vector = [ 0.0124896 , -0.0047727 , -0.133147 ]

translation_vector = [0, -0, -0]


Mt = tm.translation_matrix(translation_vector)  # Matrice de translation

matrix = Mt @ rot_matrix 

source_temp = copy.deepcopy(source)
source_a = source_temp.transform(matrix)

output_filename = "./data_exemple/source_3_reposed.ply"
o3d.io.write_point_cloud(output_filename, source_a)

