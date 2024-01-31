import math
import numpy as np
from typing import Tuple

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())


def rotation_matrix_x(alpha: float) -> np.ndarray:
    """
    Generate a 3x3 rotation matrix for rotation around the x-axis.

    Args:
        alpha (float): Rotation angle in radians.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])

def rotation_matrix_y(beta: float) -> np.ndarray:
    """
    Generate a 3x3 rotation matrix for rotation around the y-axis.

    Args:
        beta (float): Rotation angle in radians.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])


def rotation_matrix_z(gamma: float) -> np.ndarray:
    """
    Generate a 3x3 rotation matrix for rotation around the z-axis.

    Args:
        gamma (float): Rotation angle in radians.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    return np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]
    ])


def euler_matrix(ai: float, aj: float, ak: float, axes: str = 'sxyz') -> np.ndarray:
    """
    Generate a 4x4 transformation matrix based on Euler angles.

    Args:
        ai (float): Angle around the first axis.
        aj (float): Angle around the second axis.
        ak (float): Angle around the third axis.
        axes (str, optional): Euler rotation sequence. Default is 'sxyz'.

    Returns:
        np.ndarray: 4x4 transformation matrix.
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci*ck, ci*sk
    sc, ss = si*ck, si*sk

    M = np.identity(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj*si
        M[i, k] = sj*ci
        M[j, i] = sj*sk
        M[j, j] = -cj*ss+cc
        M[j, k] = -cj*cs-sc
        M[k, i] = -sj*ck
        M[k, j] = cj*sc+cs
        M[k, k] = cj*cc-ss
    else:
        M[i, i] = cj*ck
        M[i, j] = sj*sc-cs
        M[i, k] = sj*cc+ss
        M[j, i] = cj*sk
        M[j, j] = sj*ss+cc
        M[j, k] = sj*cs-sc
        M[k, i] = -sj
        M[k, j] = cj*si
        M[k, k] = cj*ci
    return M


def euler_from_matrix(matrix: np.ndarray, axes: str = 'sxyz') -> Tuple[float, float, float]:
    """
    Extract Euler angles from a given rotation matrix.

    Args:
        matrix (np.ndarray): 3x3 rotation matrix.
        axes (str, optional): Euler rotation sequence. Default is 'sxyz'.

    Returns:
        Tuple[float, float, float]: Euler angles (in radians) around specified axes.
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2(M[i, j],  M[i, k])
            ay = math.atan2(sy,       M[i, i])
            az = math.atan2(M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2(M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2(M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


def translation_matrix(translation_vector):
    """
    Generate a 4x4 translation matrix based on the given translation vector.

    Parameters
    ----------
    translation_vector : array-like
        The translation vector containing the translation along x, y, and z axes.

    Returns
    -------
    T : np.ndarray
        The 4x4 translation matrix.
    """

    T = np.eye(4)  # Création d'une matrice identité 4x4
    T[:3, 3] = translation_vector  # Modification des éléments de translation
    return T


def transformation_matrix_to_euler_xyz(transformation_matrix):
    """
    Convertit une matrice de transformation en angles d'Euler selon l'ordre XYZ.

    Args:
        transformation_matrix (np.ndarray): Matrice de transformation 4x4.

    Returns:
        Tuple[float, float, float]: Angles d'Euler (radians) selon l'ordre XYZ.
    """
    rotation_matrix = transformation_matrix[:3, :3]

    euler_angles = euler_from_matrix(rotation_matrix, 'sxyz')

    return euler_angles


def matrix_from_angles(angle_x, angle_y, angle_z):
    """
    Crée une matrice de rotation à partir des angles d'Euler en XYZ.

    Args:
        angle_x (float): Angle d'Euler autour de l'axe X en radians.
        angle_y (float): Angle d'Euler autour de l'axe Y en radians.
        angle_z (float): Angle d'Euler autour de l'axe Z en radians.

    Returns:
        np.ndarray: Matrice de rotation 4x4.
    """
    rotation_matrix = np.eye(4)
    rotation_matrix[:3, :3] = euler_matrix(
        angle_x, angle_y, angle_z, 'sxyz')[:3, :3]
    return rotation_matrix
