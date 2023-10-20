import numpy as np

#### A SUPPRIMER

# Fonction pour créer la matrice de rotation autour de l'axe X
def rotation_matrix_x(alpha):
    return np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])

# Fonction pour créer la matrice de rotation autour de l'axe Y
def rotation_matrix_y(beta):
    return np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])

# Fonction pour créer la matrice de rotation autour de l'axe Z
def rotation_matrix_z(gamma):
    return np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]
    ])

# Angles de rotation autour des axes X, Y et Z (en radians)
alpha = np.radians(10) # Exemple : rotation de 45 degrés autour de l'axe X
beta = np.radians(-10)   # Exemple : rotation de 30 degrés autour de l'axe Y
gamma = np.radians(10)  # Exemple : rotation de 60 degrés autour de l'axe Z

# Calcul de la matrice de rotation composite RXYZ
R_XYZ = np.dot(rotation_matrix_x(alpha), np.dot(rotation_matrix_y(beta), rotation_matrix_z(gamma)))

# Affichage de la matrice de rotation composite
print("Matrice de rotation RXYZ :")
print(R_XYZ)
