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

2. **Masquage** : Cette fonction permet de filtrer le nuage de points et de sélectionner uniquement la couleur de l'objet. Elle prend les paramètres suivants :

- `nom_nuage_points` : Nom du fichier du nuage de points généré par la fonction d'acquisition.
- `nom_nuage_points_filtre` : Nom du fichier dans lequel le nuage de points sera enregistré après l'application du masque.
- `seuil` : Seuil utilisé pour filtrer les points en fonction de la couleur ou des coordonnées.
- `phase_couleur` : Permet de sélectionner quelle phase des couleurs conserver, soit dans le rouge (intervalle [0-1]) et dans la colonne 0.

3. **Repositionnement** : Cette fonction "repose" les points du nuage de points pour les repositionner correctement. (Remarque : La description ne mentionne pas de paramètres spécifiques pour cette fonction.)

4. **Redimensionnement automatique** (resize avec 'r' en minuscule) : Cette fonction "resize" permet de redimensionner le modèle 3D et le nuage de points filtré et repositionné. L'utilisateur doit fournir les paramètres suivants :

- `nom_nuage_points_repositionne` : Nom du fichier du nuage de points repositionnés au format PLY.
- `nom_modele_3D` : Nom du fichier du modèle 3D au format PLY.
- `nuage_points_redimensionne` : Nom du fichier dans lequel le nouveau nuage de points redimensionné sera enregistré au format PLY.

 **Distance maximale** : Cette fonction "max_distance" calcule la distance maximale 
entre le modèle 3D et le nuage de points filtré et repositionné, puis récupère cette valeur. 


4. **Redimensionnement avec seuil** (redimensionnement avec 'R' en majuscule) :
    - `nom_nuage_points_repositionne`
    - Nom du fichier de sortie après le redimensionnement
    - Facteur de redimensionnement

    
***Remarque*** : Si l'utilisateur souhaite éviter de refaire l'acquisition à chaque fois, 
    il peut commenter la ligne "import functions.acquisition as aq" et utiliser "aq.run_acquisition(name_pc, color_image_name)". 
    De plus, si l'objet 3D possède ses propres couleurs spécifiques, l'utilisateur devra appeler ou décommenter l'appel à la fonction project_and_display.
    Dans le cas contraire, il devra décommenter la ligne contenant la fonction project_and_display_without_colors.
    """
