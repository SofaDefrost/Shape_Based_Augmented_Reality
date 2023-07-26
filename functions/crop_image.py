#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 26 15:02:23 2023

@author: tinhinane
"""

import cv2

def crop_image(image_path):
    # Variables globales pour stocker les coordonnées des clics souris
    start_x, start_y = -1, -1
    end_x, end_y = -1, -1
    cropping = False

    def mouse_click(event, x, y, flags, param):
        # Référence aux variables globales
        nonlocal start_x, start_y, end_x, end_y, cropping
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # Début du cropping
            start_x, start_y = x, y
            end_x, end_y = x, y
            cropping = True
        
        elif event == cv2.EVENT_LBUTTONUP:
            # Fin du cropping
            end_x, end_y = x, y
            cropping = False
            # Dessine le rectangle de cropping
            cv2.rectangle(image_copy, (start_x, start_y), (end_x, end_y), (0, 255, 0), 2)
            cv2.imshow("Cropping", image_copy)

    # Charger l'image
    image = cv2.imread(image_path)
    if image is None:
        print("Erreur: Impossible de charger l'image. Veuillez vérifier le chemin.")
        return None

    image_copy = image.copy()

    # Créer une fenêtre pour l'image
    cv2.namedWindow("Cropping")
    cv2.setMouseCallback("Cropping", mouse_click)

    # Instructions pour l'utilisateur
    print("Utilisez la souris pour sélectionner le rectangle de recadrage. Appuyez sur la touche 'c' pour terminer le recadrage.")

    # Boucle principale
    while True:
        cv2.imshow("Cropping", image_copy)
        key = cv2.waitKey(1) & 0xFF
        
        # Quitter la boucle si la touche "c" est pressée
        if key == ord("c"):
            break

    # Vérifier si le rectangle de recadrage a une taille valide
    if start_x == end_x or start_y == end_y:
        print("Erreur: Le rectangle de recadrage a une taille invalide.")
        return None

    # Récupérer les coordonnées de cropping
    x_min, y_min = min(start_x, end_x), min(start_y, end_y)
    x_max, y_max = max(start_x, end_x), max(start_y, end_y)

    # Recadrer l'image
    cropped_image = image[y_min:y_max, x_min:x_max]

    # Afficher l'image recadrée
    cv2.imshow("Cropped Image", cropped_image)

    # Sauvegarder l'image recadrée
    cv2.imwrite("cropped_Image.jpg", cropped_image)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Appel de la fonction
image_path = "fleur_6.jpg"
crop_image(image_path)
