""" Some test for the utils module"""

import numpy as np
from rigid_transformation import RigidTransform


# test subject for rigid transformation class
# 1. create a rigid transformation from a position and roll, pitch, yaw angles
# 2. get the transformation matrix
# 3. get the inverse transformation matrix
# 4. get the rotation matrix
# 5. get the quaternion from a rotation matrix
# 6. apply rigid body transformation to a 3d vector / rigid point


def test_rigid_transform_creation_case1():
    """Test the rigid transformation class creation from position and roll, pitch, yaw angles."""
    # setup
    position = [1, 2, 3]
    rpy = [0, 0, 0]
    # exercise
    rigid_transformation = RigidTransform.create_from_position_rpy(position, rpy)
    target_quaternion = [0, 0, 0, 1]  # xyzw
    target_rotation_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    target_transformation_matrix = np.array(
        [[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]]
    )
    target_inverse_transformation_matrix = np.array(
        [[1, 0, 0, -1], [0, 1, 0, -2], [0, 0, 1, -3], [0, 0, 0, 1]]
    )
    # assert
    assert np.allclose(rigid_transformation.position, position)
    assert np.allclose(rigid_transformation.orientation, target_quaternion)
    assert np.allclose(
        rigid_transformation.compute_rotation_matrix(), target_rotation_matrix
    )
    assert np.allclose(
        rigid_transformation.compute_transformation_matrix(),
        target_transformation_matrix,
    )
    assert np.allclose(
        rigid_transformation.compute_inverse_transformation_matrix(),
        target_inverse_transformation_matrix,
    )


def test_rigid_transform_creation_case2():
    """Test the rigid transformation class creation from position and roll, pitch, yaw angles."""
    # setup
    position = [4, 5, 6]
    rpy = [-np.pi / 2.0, np.pi / 2.0, 0.0]
    # exercise
    rigid_transformation = RigidTransform.create_from_position_rpy(position, rpy)
    target_quaternion = [-0.5, 0.5, -0.5, 0.5]
    target_rotation_matrix = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    target_transformation_matrix = np.array(
        [[0, 0, 1, 4], [-1, 0, 0, 5], [0, -1, 0, 6], [0, 0, 0, 1]]
    )
    target_inverse_transformation_matrix = np.array(
        [[0, -1, 0, 5], [0, 0, -1, 6], [1, 0, 0, -4], [0, 0, 0, 1]]
    )
    # assert
    assert np.allclose(rigid_transformation.position, position)
    assert np.allclose(rigid_transformation.orientation, target_quaternion)
    assert np.allclose(
        rigid_transformation.compute_rotation_matrix(), target_rotation_matrix
    )
    assert np.allclose(
        rigid_transformation.compute_transformation_matrix(),
        target_transformation_matrix,
    )
    assert np.allclose(
        rigid_transformation.compute_inverse_transformation_matrix(),
        target_inverse_transformation_matrix,
    )


def test_rigid_transform_creation_case3():
    """Test the rigid transformation class creation from position and roll, pitch, yaw angles."""
    # setup
    position = [7, 8, 9]
    rpy = [(30 / 180) * np.pi, (30 / 180) * np.pi, (30 / 180) * np.pi]
    # exercise
    rigid_transformation = RigidTransform.create_from_position_rpy(position, rpy)
    target_quaternion = [0.3061862, 0.1767767, 0.3061862, 0.8838835]
    target_rotation_matrix = np.array(
        [
            [0.7500000, -0.4330127, 0.5000000],
            [0.6495190, 0.6250000, -0.4330127],
            [-0.1250000, 0.6495190, 0.7500000],
        ]
    )
    target_transformation_matrix = np.array(
        [
            [0.7500000, -0.4330127, 0.5000000, 7],
            [0.6495190, 0.6250000, -0.4330127, 8],
            [-0.1250000, 0.6495190, 0.7500000, 9],
            [0, 0, 0, 1],
        ]
    )
    target_inverse_transformation_matrix = np.array(
        [
            [0.7500000, 0.649519, -0.12500, -9.32115],
            [-0.433013, 0.625000, 0.649519, -7.81458],
            [0.5000000, -0.433013, 0.750000, -6.7859],
            [0, 0, 0, 1],
        ]
    )
    # assert
    assert np.allclose(rigid_transformation.position, position)
    assert np.allclose(rigid_transformation.orientation, target_quaternion)
    assert np.allclose(
        rigid_transformation.compute_rotation_matrix(), target_rotation_matrix
    )
    assert np.allclose(
        rigid_transformation.compute_transformation_matrix(),
        target_transformation_matrix,
    )
    assert np.allclose(
        rigid_transformation.compute_inverse_transformation_matrix(),
        target_inverse_transformation_matrix,
    )


def test_rigid_transformaton_createion_case4():
    """Test the creation of a rigid transformation from a position and quaternion."""
    # setup
    position = [7, 8, 9]
    quaternion = [0.3061862, 0.1767767, 0.3061862, 0.8838835]
    # exercise
    rigid_transformation = RigidTransform.create_from_position_quaternion(
        position=position, quaternion=quaternion
    )
    target_quaternion = [0.3061862, 0.1767767, 0.3061862, 0.8838835]

    # assert
    assert np.allclose(rigid_transformation.position, position)
    assert np.allclose(rigid_transformation.orientation, target_quaternion)


def test_rigid_transformaton_createion_case5():
    """Test the creation of a rigid transformation from a position and quaternion."""
    # setup
    position = [1, 2, 3]
    quaternion = [0.0, 0.0, 0.0, 1]
    # exercise
    rigid_transformation = RigidTransform.create_from_position_quaternion(
        position=position, quaternion=quaternion
    )
    target_quaternion = [0.0, 0.0, 0.0, 1]

    # assert
    assert np.allclose(rigid_transformation.position, position)
    assert np.allclose(rigid_transformation.orientation, target_quaternion)


def test_convert_rotation_matrix_to_quaternion_case1():
    """Test the conversion of a rotation matrix to a quaternion."""
    # setup
    rotation_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    # exercise
    quaternion = RigidTransform.computer_quaternion_from_rotation_matrix(
        rotation_matrix
    )
    # assert
    assert np.allclose(quaternion, [0, 0, 0, 1])


def test_convert_rotation_matrix_quaternion_case2():
    """Test the conversion of a rotation matrix to a quaternion."""
    # setup
    rotation_matrix = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    # exercise
    quaternion = RigidTransform.computer_quaternion_from_rotation_matrix(
        rotation_matrix
    )
    # assert
    assert np.allclose(quaternion, [-0.5, 0.5, -0.5, 0.5])


def test_convert_rotation_matrix_to_quaternion_case3():
    """Test the conversion of a rotation matrix to a quaternion."""
    # setup
    rotation_matrix = np.array(
        [
            [0.7500000, -0.4330127, 0.5000000],
            [0.6495190, 0.6250000, -0.4330127],
            [-0.1250000, 0.6495190, 0.7500000],
        ]
    )
    # exercise
    quaternion = RigidTransform.computer_quaternion_from_rotation_matrix(
        rotation_matrix
    )
    # assert
    assert np.allclose(quaternion, [0.3061862, 0.1767767, 0.3061862, 0.8838835])


def test_rigid_transform_creation_tf_case1():
    """Test the rigid transformation class creation from transformation matrix."""
    # setup
    transformation_matrix = np.array(
        [[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]]
    )
    # exercise
    rigid_transformation = RigidTransform.create_from_transformation_matrix(
        transformation_matrix
    )
    target_position = [1, 2, 3]
    target_quaternion = [0, 0, 0, 1]  # xyzw
    # assert
    assert np.allclose(rigid_transformation.position, target_position)
    assert np.allclose(rigid_transformation.orientation, target_quaternion)


def test_rigid_transform_creation_tf_case2():
    """Test the rigid transformation class creation from transformation matrix."""
    # setup
    transformation_matrix = np.array(
        [[0, 0, 1, 4], [-1, 0, 0, 5], [0, -1, 0, 6], [0, 0, 0, 1]]
    )
    # exercise
    rigid_transformation = RigidTransform.create_from_transformation_matrix(
        transformation_matrix
    )
    target_position = [4, 5, 6]
    target_quaternion = [-0.5, 0.5, -0.5, 0.5]

    # assert
    assert np.allclose(rigid_transformation.position, target_position)
    assert np.allclose(rigid_transformation.orientation, target_quaternion)


def test_rigid_transform_creation_tf_case3():
    """Test the rigid transformation class creation from position and roll, pitch, yaw angles."""
    # setup
    transformation_matrix = np.array(
        [
            [0.7500000, -0.4330127, 0.5000000, 7],
            [0.6495190, 0.6250000, -0.4330127, 8],
            [-0.1250000, 0.6495190, 0.7500000, 9],
            [0, 0, 0, 1],
        ]
    )
    # exercise
    rigid_transformation = RigidTransform.create_from_transformation_matrix(
        transformation_matrix
    )
    target_position = [7, 8, 9]
    target_quaternion = [0.3061862, 0.1767767, 0.3061862, 0.8838835]
    # assert
    assert np.allclose(rigid_transformation.position, target_position)
    assert np.allclose(rigid_transformation.orientation, target_quaternion)


def test_rigid_transform_vector3d_case1():
    """Test the rigid transformation class creation from position and roll, pitch, yaw angles."""
    # setup
    position = [1, 2, 3]
    rpy = [0, 0, 0]
    point = [1, 1, 1]
    # exercise
    rigid_transformation = RigidTransform.create_from_position_rpy(position, rpy)
    transformed = rigid_transformation.transform_vector_3d(point)
    # assert
    assert np.allclose(transformed, [2, 3, 4])


def test_rigid_transform_vector3d_case2():
    """Test the rigid transformation class creation from position and roll, pitch, yaw angles."""
    # setup
    position = [0, 0, 0]
    rpy = [np.pi / 2, 0, 0]
    point = [0, 1, 0]
    # exercise
    rigid_transformation = RigidTransform.create_from_position_rpy(position, rpy)
    transformed = rigid_transformation.transform_vector_3d(point)
    # assert
    assert np.allclose(transformed, [0, 0, 1])


def test_rigid_transform_vector3d_case3():
    """Test the rigid transformation class creation from position and roll, pitch, yaw angles."""
    # setup
    position = [0, 0, 0]
    rpy = [0, 0, np.pi / 2]
    point = [0, 1, 0]
    # exercise
    rigid_transformation = RigidTransform.create_from_position_rpy(position, rpy)
    transformed = rigid_transformation.transform_vector_3d(point)
    # assert
    assert np.allclose(transformed, [-1, 0, 0])


def test_rigid_transform_rigid_case1():
    """Test the rigid transformation class creation from position and roll, pitch, yaw angles."""
    # setup
    position = [1, 2, 3]
    rpy = [0, 0, 0]
    point = [1, 1, 1, 0, 0, 0, 1]
    # exercise
    rigid_transformation = RigidTransform.create_from_position_rpy(position, rpy)
    transformed = rigid_transformation.transform_rigid_point(point)
    # assert
    assert np.allclose(transformed, [2, 3, 4, 0, 0, 0, 1])


def test_rigid_transform_rigid_case2():
    """Test the rigid transformation class creation from position and roll, pitch, yaw angles."""
    # setup
    position = [0, 0, 0]
    rpy = [np.pi / 2, 0, 0]
    point = [1, 1, 1, 0, 0, 0, 1]
    # exercise
    rigid_transformation = RigidTransform.create_from_position_rpy(position, rpy)
    transformed = rigid_transformation.transform_rigid_point(point)
    # assert
    assert np.allclose(transformed, [1, -1, 1, 0.7071068, 0, 0, 0.7071068])


def test_rigid_transform_change_frame_case1():
    """Test change reference frame for a rigid transformation.
        Need better cases
    """
    # setup
    position = [0, 0, 0]
    rpy = [0, 0, 0]
    rigid_transformation = RigidTransform.create_from_position_rpy(position, rpy)

    target_frame_position = [0, 0, 0]
    target_frame_rpy = [np.pi / 2, 0, 0]
    target_frame = RigidTransform.create_from_position_rpy(
        target_frame_position, target_frame_rpy
    )

    print(target_frame.compute_transformation_matrix())

    expected_transformation_matrix = np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )

    new_rigid_transformation = rigid_transformation.change_to_frame(
        target_frame=target_frame
    )
    assert np.allclose(
        new_rigid_transformation.compute_transformation_matrix(),
        expected_transformation_matrix,
    )
