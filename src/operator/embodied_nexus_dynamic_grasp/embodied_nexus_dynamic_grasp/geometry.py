"""Estimate an object's observable 3-D pose from segmented depth points."""

import numpy as np


def matrix_quaternion(matrix):
    """Convert a proper rotation matrix to an xyzw unit quaternion.

    Args:
        matrix: Three-by-three orthonormal rotation matrix.

    Returns:
        Four-element xyzw quaternion.
    """
    m = np.asarray(matrix, dtype=float)
    if m.shape != (3, 3) or not np.isfinite(m).all():
        raise ValueError("invalid rotation matrix")
    eigenvalues, eigenvectors = np.linalg.eigh(np.array([
        [m[0, 0] - m[1, 1] - m[2, 2], m[0, 1] + m[1, 0], m[0, 2] + m[2, 0], m[2, 1] - m[1, 2]],
        [m[0, 1] + m[1, 0], m[1, 1] - m[0, 0] - m[2, 2], m[1, 2] + m[2, 1], m[0, 2] - m[2, 0]],
        [m[0, 2] + m[2, 0], m[1, 2] + m[2, 1], m[2, 2] - m[0, 0] - m[1, 1], m[1, 0] - m[0, 1]],
        [m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1], m.trace()],
    ]) / 3.0)
    result = eigenvectors[:, int(np.argmax(eigenvalues))]
    return result if result[3] >= 0 else -result


def estimate_geometry_3d(points, ground_height, shape, minimum_points=80, supported_box=False):
    """Estimate a 3-D principal frame and center without assuming ground contact.

    Args:
        points: Segmented Nx3 depth points transformed into the fixed base frame.
        ground_height: Height of the support plane in meters.
        shape: Either cylinder or box.
        minimum_points: Minimum number of finite inlier depth points.
        supported_box: Permit a horizontal top face to use the known support plane.

    Returns:
        Mapping with center, dimensions, xyzw orientation and observation count.

    Raises:
        ValueError: If the cloud or geometry is too incomplete to estimate.
    """
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or shape not in ("box", "cylinder"):
        raise ValueError("invalid target cloud or shape")
    values = values[np.isfinite(values).all(axis=1)]
    values = values[(values[:, 2] > ground_height + 0.002) & (values[:, 2] < ground_height + 0.8)]
    if len(values) < minimum_points:
        raise ValueError("insufficient target depth points")
    median = np.median(values, axis=0)
    distances = np.linalg.norm(values - median, axis=1)
    values = values[distances <= np.percentile(distances, 97)]
    if len(values) < minimum_points:
        raise ValueError("insufficient target inliers")
    eigenvalues, axes = np.linalg.eigh(np.cov(values, rowvar=False))
    order = np.argsort(eigenvalues)[::-1]
    axes, eigenvalues = axes[:, order], eigenvalues[order]
    if eigenvalues[0] < 1e-7 or eigenvalues[0] < 1.2 * eigenvalues[1]:
        raise ValueError("target long-axis orientation is unobservable")
    if shape == "cylinder":
        # The cylinder's local z axis is its observable long axis.
        long_axis = axes[:, 0]
        reference = np.array([0., 0., 1.]) if abs(long_axis[2]) < 0.9 else np.array([1., 0., 0.])
        local_x = np.cross(reference, long_axis)
        local_x /= np.linalg.norm(local_x)
        axes = np.column_stack((local_x, np.cross(long_axis, local_x), long_axis))
    elif eigenvalues[1] < 1.05 * max(eigenvalues[2], 1e-12):
        raise ValueError("box orientation is unobservable")
    if np.linalg.det(axes) < 0:
        axes[:, 2] *= -1
    projected = values @ axes
    low, high = np.percentile(projected, [2, 98], axis=0)
    extents = high - low
    if (shape == "box" and supported_box and extents[2] < 0.002 and
            abs(float(axes[2, 2])) > 0.98):
        height = float(np.median(values[:, 2]) - ground_height)
        if 0.004 <= height <= 0.15 and min(extents[:2]) >= 0.002 and max(extents[:2]) <= 0.5:
            # A horizontal observed top face and the known support plane bound
            # the unobserved thickness; do not infer it for a free 3-D target.
            extents[2] = height
            center = axes @ ((low + high) / 2)
            center[2] -= height / 2.0
            return {"center": center.tolist(), "size": extents.tolist(),
                    "orientation": matrix_quaternion(axes).tolist(), "point_count": len(values)}
    if min(extents) < 0.002 or max(extents) > 0.5:
        raise ValueError("target extents are outside supported bounds")
    center = axes @ ((low + high) / 2)
    if shape == "cylinder":
        dimensions = [extents[2], max(extents[0], extents[1]), max(extents[0], extents[1])]
    else:
        dimensions = extents
    return {"center": center.tolist(), "size": [float(value) for value in dimensions],
            "orientation": matrix_quaternion(axes).tolist(), "point_count": len(values)}
