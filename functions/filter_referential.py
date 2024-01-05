import numpy as np

import transformations as tf

def transformation_matrix_to_euler_xyz(transformation_matrix):
    # Extraire la sous-matrice de rotation 3x3
    rotation_matrix = transformation_matrix[:3, :3]
    
    # Utiliser la fonction euler_from_matrix pour obtenir les angles d'Euler en XYZ
    euler_angles = tf.euler_from_matrix(rotation_matrix, 'sxyz')  # 'sxyz' order for XYZ Euler angles
    
    return euler_angles

# Define a function to create a 4x4 rotation matrix from XYZ angles in radians
def matrix_from_angles(angle_x, angle_y, angle_z):
    rotation_matrix = np.eye(4)
    rotation_matrix[:3, :3] = tf.euler_matrix(angle_x, angle_y, angle_z, 'sxyz')[:3, :3]
    return rotation_matrix

# # Example usage to compute and print Euler angles
# angles_in_radians = (np.radians(32), np.radians(67), np.radians(95))
# rotation_matrix = matrix_from_angles(*angles_in_radians)
# X, Y, Z = transformation_matrix_to_euler_xyz(rotation_matrix)
# print("X (Pitch): {:.2f} radians".format(X))
# print("Y (Yaw): {:.2f} radians".format(Y))
# print("Z (Roll): {:.2f} radians".format(Z))


