#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 23 10:11:49 2023

@author: tinhinane
"""

import icp as cp
import filter_referential as fr

Mat, _=cp.run_icp_2("FleurDeLisThing.ply", "source_a.ply") 
angles_euler= fr.angles(Mat)