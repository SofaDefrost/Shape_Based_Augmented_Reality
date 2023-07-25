#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 23 20:29:42 2023

@author: tinhinane
"""

import numpy as np
import trimesh 

def repose(pc_resized, pc_reposed):
    """
    

    Parameters
    ----------
    pc_resized : TYPE
        DESCRIPTION.
    pc_reposed : TYPE
        DESCRIPTION.

    Returns
    -------
    pt_milieu : TYPE
        DESCRIPTION.

    """
    
    mesh = trimesh.load(pc_resized)
    milieu = np.mean(mesh.vertices)

    tab_x = []
    tab_y = []
    tab_z = []

    for i in mesh.vertices:
        tab_x.append(i[0])
        tab_y.append(i[1])
        tab_z.append(i[2])

    milieu_x = np.mean(tab_x)
    milieu_y = np.mean(tab_y)
    milieu_z = np.mean(tab_z)

    pt_milieu = [milieu_x, milieu_y, milieu_z]

    print(pt_milieu)

    new_vertices = mesh.vertices - pt_milieu
    scaled_mesh = trimesh.Trimesh(vertices=new_vertices, faces=mesh.faces if hasattr(mesh, 'faces') else None)

    scaled_mesh.export(pc_reposed)
    return pt_milieu
    