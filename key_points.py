import numpy as np
import cv2
import time

from functions import icp as cp
from functions import project_and_display as proj
from functions import matrix_operations as tf

from Python_3D_Toolbox_for_Realsense import acquisition_realsense as aq
from Python_3D_Toolbox_for_Realsense import calibration_matrix_realsense as rc
from Python_3D_Toolbox_for_Realsense.functions import processing_ply as ply
from Python_3D_Toolbox_for_Realsense.functions import processing_point_cloud as pc
from Python_3D_Toolbox_for_Realsense.functions import processing_pixel_list as pixels
from Python_3D_Toolbox_for_Realsense.functions import processing_img as img
from Python_3D_Toolbox_for_Realsense.functions import previsualisation_application_function as Tk
from Python_3D_Toolbox_for_Realsense.functions.utils import array as array

def trouver_correspondances(image1, image2):
    
    # Convertir les images en niveaux de gris
    image1_grise = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    image2_grise = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
    
    # Initialiser le détecteur ORB et les descripteurs
    orb = cv2.ORB_create()
    
    # Détecter les points d'intérêt et calculer les descripteurs pour les deux images
    points_interet1, descripteurs1 = orb.detectAndCompute(image1_grise, None)
    points_interet2, descripteurs2 = orb.detectAndCompute(image2_grise, None)
    
    # Dessiner les points d'intérêt sur l'image
    image_points1 = cv2.drawKeypoints(image1, points_interet1, None, color=(0, 255, 0), flags=0)
    image_points2 = cv2.drawKeypoints(image2, points_interet2, None, color=(0, 255, 0), flags=0)
    # print(len(points_interet1))
    # print(len(points_interet2))
    # Afficher l'image avec les points d'intérêt
    cv2.imshow('Points d\'intérêt', image_points1)
    cv2.waitKey(0)
    cv2.imshow('Points d\'intérêt', image_points2)
    cv2.waitKey(0)
    
    # Utiliser le BFMatcher avec la distance de Hamming
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    
    # Trouver les correspondances entre les descripteurs des deux images
    correspondances = bf.match(descripteurs1, descripteurs2)
    
    # Trier les correspondances en fonction de leur distance
    correspondances = sorted(correspondances, key=lambda x: x.distance)
    
    # ça c'est a liste des points des correpondances dans la deuxième image
    points = [ (int(points_interet2[correspondances[i].trainIdx].pt[0]),int(points_interet2[correspondances[i].trainIdx].pt[1])) for i in range(len(correspondances))]
    # ça c'est la masque associé
    mask=np.array([[False for i in range(640)] for i in range(480)])
    for i in range(len(points)):
        mask[points[i][1]][points[i][0]]=True
    # Dessiner les correspondances sur une nouvelle image
    image_correspondances = cv2.drawMatches(image1, points_interet1, image2, points_interet2, correspondances, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    
    # Afficher l'image avec les correspondances
    cv2.imshow('Correspondances', image_correspondances)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return mask

if __name__ == '__main__':

    image1_path = 'test1.png'
    image2_path = 'test2.png'

    # image1_path = 'test3.png'
    # image2_path = 'test4.png'

    pipeline=aq.init_realsense(640,480)
    points,colors=aq.get_points_and_colors_from_realsense(pipeline)
    pixels.display(colors,"fff")
    image1 = cv2.imread(image1_path)
    # image2 = cv2.imread(image2_path)
    mask = trouver_correspondances(image1, colors)
    # points_filtre,colors_filtre,_ = pc.apply_binary_mask(points,colors,mask,(480,640))
    # ply.save("test.ply",points_filtre,colors_filtre)