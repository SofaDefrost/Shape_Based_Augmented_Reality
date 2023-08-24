

transform_matrix = [[ 0.79258242 , 0.60402277 , 0.08348413 , 0.        ],[-0.58752594,  0.71984631 , 0.36964112,  0.        ],[ 0.16317591, -0.34202014 , 0.92541658,  0.        ],[ 0.     ,     0.  ,        0.  ,        1.        ]]

print(transform_matrix)
rotation_matrix = transform_matrix[:3, :3]

# Convertir la matrice de rotation en objet Rotation
rotation = R.from_matrix(rotation_matrix)

# print("ROTATION :")
# print(rotation)

# Obtenir les angles d'Euler (en radians) sous forme d'un vecteur (roll, pitch, yaw)
euler_angles = rotation.as_euler('zyx', degrees=True)  # 'zyx' signifie que les rotations sont appliquées dans l'ordre ZYX

print("ROTATION :")
print(euler_angles)