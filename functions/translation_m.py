#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 23 18:50:10 2023

@author: tinhinane
"""

import numpy as np
def translation_matrix(translation_vector):
    T = np.eye(4)  # Création d'une matrice identité 4x4
    T[:3, 3] = translation_vector  # Modification des éléments de translation
    return T