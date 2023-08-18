## Augmented_Reality

# Mode d'emploi 

Il faut executer le code python main.py :
```console
python3 main.py
```
L'utilisateur doit prendre en compte les points suivants :

**Important**:
- Si vous souhaitez exécuter le programme contenant un exemple sans refaire l'acquisition, veuillez exécuter "main_sans_acq.py". En revanche, si vous souhaitez réaliser une acquisition, veuillez exécuter le fichier "main_acq.py".
- Placez votre fichier objet 3D dans le dossier "data_exemple".
- Indiquez le chemin vers le modèle 3D.
- Attribuez un nom au nuage de points et à l'image optique en utilisant la variable "name".
- **Pendant la capture, l'utilisateur devra appuyer sur la touche S du clavier pour démarrer la capture et la touche Q pour l'arrêter, dans le cas de l'exécution de "main_acq.py"**.
- Configurez le seuil de couleur et la plage de couleurs.
- Vérifiez le type de caméra utilisée pour appliquer la matrice de calibration spécifique à cette caméra.
- Si l'objet 3D possède ses propres couleurs spécifiques, l'utilisateur doit appeler ou décommenter l'appel à la fonction "project_and_display". Dans le cas contraire, il doit décommenter la ligne contenant la fonction "project_and_display_without_colors".
- N'oubliez pas de fermer la fenêtre d'affichage de "l'ICP" afin d'avoir l'affichage de la réalité augmentée.
- Pour arrêter l'affichage, il suffit de cliquer sur la touche Q.

Pour les deux fonctions suivantes, vous devez choisir en fonction des résultats souhaités :

- **Redimensionnement automatique** (redimensionnement avec 'r' en minuscule) : Cette fonction "resize" permet de redimensionner le modèle 3D et le nuage de points filtré et repositionné. L'utilisateur doit fournir les paramètres suivants :
    - `nom_nuage_points_repositionne` : Nom du fichier du nuage de points repositionnés au format PLY.
    - `nom_modele_3D` : Nom du fichier du modèle 3D au format PLY.
    - `nuage_points_redimensionne` : Nom du fichier dans lequel le nouveau nuage de points redimensionné sera enregistré au format PLY.

- **Distance maximale** : Cette fonction "max_distance" calcule la distance maximale entre le modèle 3D et le nuage de points filtré et repositionné, puis récupère cette valeur.

- **Redimensionnement avec seuil** (redimensionnement avec 'R' en majuscule) :
    - `nom_nuage_points_repositionne` : Nom du fichier du nuage de points repositionnés.
    - Nom du fichier de sortie après le redimensionnement.
    - Facteur de redimensionnement.
    
# Prérecquis :
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

# Description générale du programme :

Ce programme utilise plusieurs fonctions pour effectuer les étapes suivantes :

1. **Acquisition** : Cette fonction permet de récupérer un nuage de points en se connectant à la caméra RealSense. Pour l'utiliser, l'utilisateur doit d'abord initialiser le nom du fichier du nuage de points ainsi que le nom de l'image optique. Ensuite, appelez la fonction `run_acquisition` en fournissant les paramètres suivants :
    - `nom_fichier_nuage_points` (nom du fichier du nuage de points) et `nom_image_2D`.

2. **Masquage** : Cette fonction permet de filtrer le nuage de points pour ne sélectionner que la couleur de l'objet. Elle prend les paramètres suivants :
    - `nom_nuage_points` : Nom du fichier du nuage de points généré par la fonction d'acquisition.
    - `nom_nuage_points_filtre` : Nom du fichier dans lequel le nuage de points sera enregistré après l'application du masque.
    - `seuil` : Seuil utilisé pour filtrer les points en fonction de la couleur ou des coordonnées.
    - `phase_couleur` : Permet de sélectionner quelle plage de couleurs conserver, par exemple, dans le spectre rouge (intervalle [0-1]) et dans la colonne 0.

3. **Repositionnement** : Cette fonction permet de "repositionner" correctement les points du nuage de points.

4. **Redimensionnement** : Cette fonction permet de redimensionner un nuage de points en réduisant ou en augmentant la taille de ses sommets.

5. **Angles** : Cette fonction permet d'inverser les angles de l'axe 'Y' et 'Z' du nuage de points.

6. **ICP** : Cette fonction permet d'aligner deux nuages de points et de récupérer la meilleure matrice initiale ainsi que la matrice d'alignement.

7. **Translation_m** : Cette fonction permet de convertir le vecteur de translation en une matrice de transformation 4x4.

8. **Project_and_display** : Cette fonction projette le nuage de points de l'objet 3D, qui contient ses couleurs spécifiques, sur une image optique.

9. **Project_and_display_without_colors** : Cette fonction est utilisée pour un fichier modèle 3D qui ne contient pas de couleurs.

10. **Ply2obj** : Cette fonction réalise la conversion d'un fichier .ply en un fichier .obj.
