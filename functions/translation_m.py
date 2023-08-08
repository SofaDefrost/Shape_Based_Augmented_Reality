#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 23 18:50:10 2023

@author: tinhinane
"""

import numpy as np
def translation_matrix(translation_vector):
    """
    Generate a 4x4 translation matrix based on the given translation vector.

    Parameters
    ----------
    translation_vector : array-like
        The translation vector containing the translation along x, y, and z axes.

    Returns
    -------
    T : numpy.ndarray
        The 4x4 translation matrix.
    """ 
     
    T = np.eye(4)  # Création d'une matrice identité 4x4
    T[:3, 3] = translation_vector  # Modification des éléments de translation
    return T