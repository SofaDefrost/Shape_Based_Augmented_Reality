# Augmented_Reality

Ce programme permet de supperposer une modèle 3D à un nuage de points capturé par une caméra RealSense et d'en faire un affichage.

Auteurs : Tinhinane et Thibaud

## Mode d'emploi 

Il faut executer le code python main.py :
```console
python3 main.py
```
## Points importants
L'utilisateur doit prendre en compte les points suivants :

- **Pendant la capture, l'utilisateur devra appuyer sur la touche S du clavier pour effectuer la capture et la touche Q pour passer à l'étape suivante**.
- Vérifiez le type de caméra utilisée pour appliquer la matrice de calibration spécifique à cette caméra.
- N'oubliez pas de fermer la fenêtre d'affichage de "l'ICP" afin d'avoir l'affichage de la réalité augmentée.
- Pour arrêter cet affichage, il suffit de cliquer sur la touche Q.

    
## Prérecquis :
Librairies python nécessaires : pyrealsense2 et opencv-python, trimesh, open3d, scipy
```console
pip3 install pyrealsense2
pip3 install opencv-python
pip3 install trimesh
pip3 install time
pip3 install copy
pip3 install open3d
pip3 install scipy
pip3 install math
pip3 install numpy
```

## Description générale du programme :

Ce programme utilise plusieurs fonctions pour effectuer les étapes suivantes :

1. **Acquisition** : Cette fonction permet de récupérer un nuage de points en se connectant à la caméra RealSense. Pour l'utiliser, l'utilisateur doit d'abord initialiser le nom du fichier du nuage de points ainsi que le nom de l'image optique. Ensuite, appelez la fonction `run_acquisition` en fournissant les paramètres suivants :
    - `nom_fichier_nuage_points` (nom du fichier du nuage de points) et `nom_image_2D`.

1. **Détermination et application d'un masque** : Cette fonction permet de filtrer le nuage de points avec un masque hsv :
    - Lors de l'éxécution de la fonction *determinemask* plusieures fenêtres s'ouvrent. Ces dernières permettent déterminer le masque hsv à appliquer à notre capture. Parmis elles, une permet à l'utilisateur de paramétrer le masque à l'aide de curseurs. Au fur et à mesure de sa configuration, l'utilisateur voit le retour de l'application de son masque sur une autre fenêtre. Une fois que ce dernier est satisfait par son choix, il peut l'exporter en appuyant sur la touche 'q'.
    - Le masque hsv est alors exporté.
    - On applique ensuite ce masque au nuage de points.

1. **Redimensionnement** : Cette étape permet de redimensionner le nuage de points cible (un modèle CAO par exemple) en réduisant ou en augmentant la taille de ses sommets.

1. **Repositionnement** : Cette étape permet de "repositionner" correctement les points du nuage de points.

1. **Translation** : Cette étape permet de déterminer la translation à effectuer à notre nuage de points pour que ce dernier puisse se superposer à notre modèle 3D.

1. **Pré-rotation** : Cette étape permet de trouver la bonne configuration initiale pour notre nuage de points pour l'algorithme d'ICP. En effet, lors de cette étape, on parcourt l'ensemble des rotations possibles (vous pouvez modifier les plages d'angles à parcourir) et on conserve celle qui match le mieux avec notre nuage de points. Cette étape permet de déterminer "grossièrement" la position du nuage de points.

1. **ICP** : Maintenant que l'on connait les rotations à appliquer à notre nuage de point, on applique un algorithme d'ICP à partir de cette position. Ce dernier permettra d'obtenir avec précision la position du nuage de points.

1. **Points de projections** : Puisque l'on est dans un cadre de réalité augmenté, on cherche à pouvoir superposer un modèle 3D à notre nuage de points (c'est à dire l'inverse que ce que l'on fait depuis le début !). Il faut donc recalculer la position des points de ce modèle 3D dans le repère dans la vue de la camèra. On n'oublie pas de considérer la matrice intrinséque de la caméra (qui gérer les déformations dues à la distance focale, les lentilles...)

1. **Afficher les résulats** : Il ne reste plus qu'à afficher la superposition finale de notre modèle 3D sur notre nuage de points.

""" 
L'utilisateur doit prendre en compte les points suivants :

**Important**:
- **Pendant la capture, l'utilisateur devra appuyer sur la touche S du clavier et la touche Q pour arrêter la capture.**
- Assurez-vous que tous les fichiers existent dans le même dossier.
- Indiquez le chemin vers le modèle 3D ainsi que l'image.
- Nommez le nuage de points acquis par la caméra Realsense.
- Définissez le seuil de couleur et la phase de couleur.
- Vérifiez le type de caméra utilisé pour pouvoir appliquer la matrice de calibration spécifique à cette caméra.
- Pour arrêter l'exécution, il suffit de cliquer sur la touche Q.'

Ce programme utilise plusieurs fonctions pour effectuer les étapes suivantes :

1. **Acquisition** : Cette fonction permet de récupérer le nuage de points en connectant la caméra RealSense. Pour l'utiliser, 
l'utilisateur doit d'abord initialiser le nom du fichier du nuage de points et le nom de l'image optique. 
Ensuite, appelez la fonction `run_acquisition` en fournissant les paramètres suivants : 
    `nom_fichier_nuage_points` (nom du fichier du nuage de points) et `nom_image_2D`.

2. **Masquage** : Cette fonction permet de filtrer le nuage de points selon un filtre hsv. Elle prend les paramètres suivants :

- `points` : Liste des coordonées du nuages de points
- `couleurs` : Liste des couleurs associées à ce nuage de points
- `mask` : Masque hsv à appliquer 

3. **Filtrage Bruit** : Cette fonction permet de filtrer le nuage de points pour éviter le bruit environnant. La fonction crée une interface TKinter qui permet de selectionner la taille du rayon de filtrage (par rapport au centre de masse) :

- `points` : Liste des coordonées du nuages de points
- `couleurs` : Liste des couleurs associées à ce nuage de points

4. **Repositionnement** : Cette fonction "repose" les points du nuage de points pour les repositionner correctement. (Remarque : La description ne mentionne pas de paramètres spécifiques pour cette fonction.)

5. **Redimensionnement automatique** (resize_auto) : Cette fonction permet de redimensionner le modèle 3D et le nuage de points filtré et repositionné. L'utilisateur doit fournir les paramètres suivants :

- `nom_nuage_points_repositionne` : Nom du fichier du nuage de points repositionnés au format PLY.
- `nom_modele_3D` : Nom du fichier du modèle 3D au format PLY.
- `nuage_points_redimensionne` : Nom du fichier dans lequel le nouveau nuage de points redimensionné sera enregistré au format PLY.



6. **Redimensionnement avec seuil** (Resize_pas_auto) :
    - `nom_nuage_points_repositionne`
    - Nom du fichier de sortie après le redimensionnement
    - Facteur de redimensionnement

    
***Remarque*** : Si l'utilisateur souhaite éviter de refaire l'acquisition à chaque fois, 
    il peut commenter la ligne "import functions.acquisition as aq" et utiliser "aq.run_acquisition(name_pc, color_image_name)". 
    De plus, si l'objet 3D possède ses propres couleurs spécifiques, l'utilisateur devra appeler ou décommenter l'appel à la fonction project_and_display.
    Dans le cas contraire, il devra décommenter la ligne contenant la fonction project_and_display_without_colors.
    """