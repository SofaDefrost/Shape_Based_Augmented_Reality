#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 23 09:45:05 2023

@author: tinhinane
"""

""" 
L'utilisateur doit verifier que tous les fichiers sont excistants dans le meme dossier

Dans ce main il fait appel à toutes les fonctions qui permet de faire:
    
   Voici les corrections apportées au texte en soulignant les erreurs :

Dans ce programme, il fait appel à plusieurs fonctions qui permettent de réaliser les étapes suivantes :

1- Acquisition : Cette fonction permet de récupérer le nuage de points en branchant la caméra RealSense. Pour l'utiliser,
 l'utilisateur doit d'abord initialiser le nom du fichier du nuage de points et le nom de l'image optique. Ensuite, 
 il faut appeler la fonction `run_acquisition` en lui fournissant les paramètres suivants : `file_name_point_cloud` (nom du fichier du nuage de points) et `name_image_2D`.

2- Masque : Cette fonction permet de filtrer le nuage de points et de sélectionner uniquement la couleur de l'objet. Elle prend les paramètres suivants :

- `point_cloud_name` : Le nom du fichier du nuage de points généré par la fonction acquisition.
- `filtered_cloud_name` : Le nom du fichier dans lequel le nuage de points sera enregistré après l'application du masque.
- `threshold` : Le seuil utilisé pour filtrer les points selon la couleur ou les coordonnées.
- `color_phase` : Permet de sélectionner quelle phase des couleurs conserver, soit dans le rouge (intervalle [0-1]) et dans la colonne 0.

3- Repositionnement : Cette fonction "repose" les points du nuage de points pour les repositionner correctement. (Remarque : La description ne mentionne pas de paramètres spécifiques pour cette fonction.)

4- Distance maximale : Cette fonction "max_distance" calcule la distance maximale entre le modèle 3D et le nuage de points filtré et repositionné, puis récupère cette valeur. (Remarque : La description ne mentionne pas de paramètres spécifiques pour cette fonction.)

5- Redimensionnement : Cette fonction "resize" permet de redimensionner le modèle 3D et le nuage de points filtré et repositionné. L'utilisateur doit fournir les paramètres suivants :

- `pc_reposing_name` : Le nom du fichier du nuage de points repositionné au format PLY.
- `model_3D_name` : Le nom du fichier du modèle 3D au format PLY.
- `point_cloud_resizing` : Le nom du fichier dans lequel le nouveau nuage de points redimensionné sera enregistré au format PLY.

6- ICP : L'utilisateur doit appeler la fonction `run_icp` en fournissant les paramètres suivants : le nom du fichier du nuage de points repositionné et le modèle 3D redimensionné. (Remarque : La description ne mentionne pas les détails spécifiques de la fonction `run_icp`, comme ses paramètres exacts.)

 
    """

import mask as msk
import icp
import translation_m as tm
import repose as rp
import resize as rz
import point_cloud as pc
import numpy as np
import cv2
from objloader_simple import *
import open3d as o3d
import time as tm
import projection as proj
import acquisition as aq

name_model_3D= "donner le nom du fichier du model 3D"

#Récupération du nuage de point en utilisant la Realsense
name="donner un nom pour ton fichier"
name_pc= name+'.ply'
name_image_2D= name+'jpg'
#appeler la fonction run_aquisition pour récupérer le nuage de point
aq.run_acquisition(name_pc,name_image_2D)


#Application du masque
#donner le nom pour le fichier nouveau aprés l'application du masque 

pc_filtred_name=name+'_masking.ply' #donner le nom pour le fichier nouveau aprés l'application du masque 
threshold="donner le seuil de la couleur"
color_phase= "écrire ""rouge ""si la couleur voudrait selectionnée est rouge ,écrire ""vert"" si la couleur voudrait sélectionnée est verte"
"écrire ""bleu"" si la couleur voudrait sélectionnée est bleu"
#Appeler la fonction mask             
mask(name_pc,pc_filtred_name,threshold,color_phase)


#Application de redimensinnement

pc_reposing_name=name +'_resizing.ply'

point_cloud_resizing=


#Application de repositionnement
pc_reposed= name+'_resizing.ply'
pc_



               


