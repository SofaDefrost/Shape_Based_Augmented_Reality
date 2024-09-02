""" Rigid transformation class. """

import numpy as np

# from splib3.numerics import Quat
from scipy.spatial.transform import Rotation as R


class RigidTransform:
    """Class to represent a ridig transformation"""

    def __init__(self, state):
        self.position = np.array(state[0:3])
        # self.orientation = Quat(state[3:7])  # xyzw
        self.orientation = R.from_quat(state[3:7])
        # A possible optimization is to store
        # the transformation matrix and the inverse transformation matrix
        self.transformation_matrix = None
        self.rotation_matrix = None
        self.inverse_transformation_matrix = None
        self.rotation_matrix = None

    @staticmethod
    def create_from_position_quaternion(position, quaternion):
        """Creates a rigid transform from a position and a quaternion."""
        rigid_transformation = RigidTransform([0, 0, 0, 0, 0, 0, 1])
        rigid_transformation.position = np.array(position)
        # rigid_transformation.orientation = Quat(quaternion)
        rigid_transformation.orientation = R.from_quat(quaternion)
        return rigid_transformation

    @staticmethod
    def create_from_position_rpy(position, rpy):
        """
        Creates a rigid transform from a position and roll, pitch, yaw angles.

        Args:
            position (np.ndarray): Position vector.
            rpy (np.ndarray): Roll, pitch, yaw angles in radians.

        Returns:
            RigidTransform: Rigid transform.
        """
        rigid_transformation = RigidTransform([0, 0, 0, 0, 0, 0, 1])
        rigid_transformation.position = np.array(position)
        # rigid_transformation.orientation = Quat.createFromEuler(rpy, "ryxz").normalize()
        # rigid_transformation.orientation = Quat.createFromEuler(rpy, "rxyz").normalize()
        rigid_transformation.orientation = R.from_euler("XYZ", rpy, degrees=False)
        return rigid_transformation

    @staticmethod
    def create_from_transformation_matrix(transformation_matrix):
        """
        Creates a rigid transform from a transformation matrix.

        Args:
            transformation_matrix (np.ndarray): Transformation matrix.

        Returns:
            RigidTransform: Rigid transform.
        """
        position, rotation_matrix = (
            RigidTransform.decompose_rigid_transformation_matrix(transformation_matrix)
        )
        rigid_transformation = RigidTransform([0, 0, 0, 0, 0, 0, 1])
        rigid_transformation.position = position
        # rigid_transformation.orientation = (
        #     RigidTransform.computer_quaternion_from_rotation_matrix(rotation_matrix)
        # )
        rigid_transformation.orientation = R.from_matrix(rotation_matrix)
        # quat = RigidTransform.computer_quaternion_from_rotation_matrix(rotation_matrix)
        # rigid_transformation.orientation = R.from_quat(quat)
        return rigid_transformation

    @staticmethod
    def create_from_position_matrix(pos, rot_matrix):
        """
        Create a new RigidTransform from a position and rotation matrix

        Args:
            pos (np.ndarray): Position vector.
            rot_matrix (np.ndarray): Rotation matrix.

        Returns:
            RigidTransform: Rigid transform.
        """
        rigid_transformation = RigidTransform([0, 0, 0, 0, 0, 0, 1])
        rigid_transformation.position = np.array(pos)
        rigid_transformation.orientation = R.from_matrix(rot_matrix)
        return rigid_transformation

    @staticmethod
    def decompose_rigid_transformation_matrix(rigid_transformation_matrix):
        """
        Decomposes a rigid transformation matrix into a position and a rotation matrix.

        Args:
            rigid_transformation_matrix (np.ndarray): Rigid transformation matrix.

        Returns:
            np.ndarray: Position vector.
            np.ndarray: Rotation matrix.
        """
        return (
            rigid_transformation_matrix[0:3, 3],
            rigid_transformation_matrix[0:3, 0:3],
        )

    @staticmethod
    def computer_quaternion_from_rotation_matrix(rot_matrix):
        """
        Convert a 3x3 rotation matrix to quaternion.

        Parameters:
            R (numpy.ndarray): 3x3 rotation matrix.

        Returns:
            numpy.ndarray: [x, y, z, w].
        """
        # Extract rotation matrix components
        r11, r12, r13 = rot_matrix[0, 0], rot_matrix[0, 1], rot_matrix[0, 2]
        r21, r22, r23 = rot_matrix[1, 0], rot_matrix[1, 1], rot_matrix[1, 2]
        r31, r32, r33 = rot_matrix[2, 0], rot_matrix[2, 1], rot_matrix[2, 2]

        # Calculate quaternion components
        qw = np.sqrt(1 + r11 + r22 + r33) / 2
        qx = (r32 - r23) / (4 * qw)
        qy = (r13 - r31) / (4 * qw)
        qz = (r21 - r12) / (4 * qw)

        return [qx, qy, qz, qw]

    @staticmethod
    def create_translation_matrix(rot_matrix, t_vec):
        """
        Creates a translation matrix from a rotation matrix and a translation vector.

        Args:
            rot_matrix (np.ndarray): Rotation matrix.
            t_vec (np.ndarray): Translation vector.

        Returns:
            np.ndarray: Translation matrix.
        """
        mat = np.concatenate((rot_matrix, t_vec.reshape(3, 1)), axis=1)
        mat = np.concatenate((mat, [[0, 0, 0, 1]]), axis=0)
        return mat

    def get_orientation_quat(self):
        quat = self.orientation.as_quat(canonical=True)
        return quat

    def get_position(self):
        return self.position

    def inverse(self):
        return RigidTransform.create_from_transformation_matrix(
            self.compute_inverse_transformation_matrix()
        )

    def transform_rigid_point(self, rigid_point):
        """
        Transforms a point using the rigid transform.

        Args:
            point (np.ndarray): Point to transform. [x, y, z, qx, qy, qz, qw]

        Returns:
            np.ndarray: Transformed point.
        """
        rigid_body = RigidTransform(rigid_point[0:7])
        pose_transformed = np.dot(
            self.compute_transformation_matrix(),
            rigid_body.compute_transformation_matrix(),
        )
        position, orientation = RigidTransform.decompose_rigid_transformation_matrix(
            pose_transformed
        )
        quaternion = RigidTransform.computer_quaternion_from_rotation_matrix(
            orientation
        )
        result_pose = [
            position[0],
            position[1],
            position[2],
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3],
        ]
        return result_pose

    def transform_vector_3d(self, vector_3d):
        """
        Transforms a 3D vector using the rigid transform.

        Args:
            vector_3d (np.ndarray): 3D vector to transform.   3*1

        Returns:
            np.ndarray: Transformed 3D vector.
        """
        augmented_vector = np.append(vector_3d, 1)
        transformed_vector = np.dot(
            self.compute_transformation_matrix(), augmented_vector
        )
        result_vec = [
            transformed_vector[0],
            transformed_vector[1],
            transformed_vector[2],
        ]
        return result_vec

    # This function is probably wrong
    # def change_to_frame(self, target_frame):
    #     """
    #     Transforms the current rigid transformation to the specified target frame.

    #     Parameters:
    #     - target_frame: The target frame to which the transformation should be changed.

    #     Returns:
    #     - A new RigidTransformation object representing the transformed rigid transformation.
    #     """
    #     rigid_transformation_in_target_frame = np.dot(
    #         target_frame.compute_transformation_matrix(),
    #         np.dot(
    #             self.compute_transformation_matrix(),
    #             target_frame.compute_inverse_transformation_matrix(),
    #         ),
    #     )

    #     return self.create_from_transformation_matrix(
    #         rigid_transformation_in_target_frame
    #     )

    def compute_transformation_matrix(self):
        """
        Returns the transformation matrix of the rigid transform.

        Returns:
            np.ndarray: Transformation matrix.
        """
        rot_matrix = self.orientation.as_matrix()
        vec = self.position.reshape(3, 1)
        self.transformation_matrix = self.create_translation_matrix(rot_matrix, vec)
        return self.transformation_matrix

    def compute_inverse_transformation_matrix(self):
        """
        Returns the inverse transformation matrix of the rigid transform.

        Returns:
            np.ndarray: Inverse transformation matrix.
        """
        rot_inv = self.orientation.inv()
        self.inverse_transformation_matrix = self.create_translation_matrix(
            rot_inv.as_matrix(),
            -np.dot(rot_inv.as_matrix(), self.position).reshape(3, 1),
        )
        return self.inverse_transformation_matrix

    def compute_rotation_matrix(self):
        """
        Returns the rotation matrix of the rigid transform.

        Returns:
            np.ndarray: Rotation matrix.
        """
        self.rotation_matrix = self.orientation.as_matrix()
        return self.rotation_matrix
