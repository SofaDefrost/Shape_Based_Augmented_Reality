#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 23 10:11:49 2023

@author: tinhinane
"""

import functions.icp as cp
import functions.filter_referential as fr
import functions.matrix_function as mf


Mat, _=cp.run_icp_2("./data_exemple/FleurDeLisThing.ply", "./data_exemple/source_b.ply") 
# Mat = mf.create_rot_matrix_x(90)

angles_euler= fr.angles(Mat)

# print(angles_euler)


