import numpy as np
import numpy.typing as npt

Array2DFloat = npt.NDArray[np.float32]

def project_vector_to_plane(v, n):
    """
    Projects a vector v onto a plane defined by a normal vector n.

    Parameters:
        v: The vector to project (3D).
        n: The normal vector of the plane (3D).

    Returns:
        The projected vector lying on the plane.
    """
    v = np.array(v).flatten()
    n = np.array(n).flatten()
    # Normalize the normal vector
    n_normalized = n / np.linalg.norm(n)

    # Compute the parallel component
    v_parallel = np.dot(v, n_normalized) * n_normalized

    # Subtract the parallel component to get the projection
    v_projected = v - v_parallel

    return v_projected


def angle_between_vectors(v1, v2):

    v1 = np.array(v1).flatten()
    v2 = np.array(v2).flatten()
    # Normalize the vectors
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    # Compute the dot product and angle
    dot_product = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
    angle = np.arccos(dot_product)  # Angle in radians (0 to pi)

    # Compute the cross product to determine the direction
    cross_product = np.cross(v1, v2)

    if np.dot(cross_product, [0, 0, 1]) < 0:  # Assuming a Z-axis reference
        angle = 2 * np.pi - angle  # Adjust angle to 360-degree range

    return np.degrees(angle)


def rotate_vector_around_normal(v, n, theta) -> Array2DFloat:
    """
    Rotate vector `v` around normal `n` by angle `theta` (in radians).

    Args:
        v: The vector to rotate (3D array-like).
        n: The normal vector (3D array-like, should be normalized).
        theta: The rotation angle in radians.

    Returns:
        Rotated vector.
    """
    n = np.array(n).flatten()
    v = np.array(v).flatten()

    n = np.array(n) / np.linalg.norm(n)  # Ensure `n` is a unit vector

    # Compute terms of Rodrigues' rotation formula
    v_cos_theta = v * np.cos(theta)
    n_cross_v = np.cross(n, v) * np.sin(theta)
    n_dot_v = np.dot(n, v) * (1 - np.cos(theta)) * n

    # Combine terms
    v_rot = v_cos_theta + n_cross_v + n_dot_v
    return v_rot


def angle_between_vectors_2d(u, v):
    # Normalize the vectors
    u = np.array(u).flatten()
    v = np.array(v).flatten()

    # Normalize the vectors
    u_norm = u / np.linalg.norm(u)
    v_norm = v / np.linalg.norm(v)

    # Compute the dot product and clip to avoid numerical errors
    dot_product = np.dot(u_norm, v_norm)
    dot_product = np.clip(dot_product, -1.0, 1.0)

    # Compute the cross product (z-component for 2D vectors)
    cross_product = u_norm[0] * v_norm[1] - u_norm[1] * v_norm[0]

    # Compute the angle in radians
    angle_rad = np.arctan2(cross_product, dot_product)

    # Convert to degrees
    angle_deg = np.degrees(angle_rad)

    # Ensure the angle is in the range 0–360
    if angle_deg < 0:
        angle_deg += 360

    return angle_deg


def unit_vector(vector: Array2DFloat):
    return vector / np.linalg.norm(vector)