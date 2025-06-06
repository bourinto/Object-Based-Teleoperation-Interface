"""Utility functions for pose conversions and math helpers."""

from typing import Iterable, Mapping
import numpy as np
from geometry_msgs.msg import Twist


def sawtooth(angle: float) -> float:
    """Wrap angle to [-180, 180) degrees."""
    return (angle + 180.0) % 360.0 - 180.0


def convert_pose(
    R_matrix: np.ndarray,
    msg: Twist,
    offsets: Mapping[str, float] | None = None,
) -> Twist:
    """Convert Unity controller pose to an xArm ``Twist``.

    Parameters
    ----------
    R_matrix:
        2x2 rotation matrix for the controller frame.
    msg:
        Incoming controller pose.
    offsets:
        Mapping with optional ``x``, ``y``, ``z``, ``roll``, ``pitch`` and
        ``yaw`` keys containing offset values in millimetres/degrees.
    """
    offsets = offsets or {}
    off_x = offsets.get("x", 0.0)
    off_y = offsets.get("y", 0.0)
    off_z = offsets.get("z", 0.0)
    off_roll = offsets.get("roll", 0.0)
    off_pitch = offsets.get("pitch", 0.0)
    off_yaw = offsets.get("yaw", 0.0)

    xy = R_matrix @ [-(msg.linear.z) * 1000.0, msg.linear.x * 1000.0]
    converted = Twist()
    converted.linear.x = xy[0] + off_x
    converted.linear.y = xy[1] + off_y
    converted.linear.z = msg.linear.y * 1000.0 + off_z
    converted.angular.x = 180.0 + off_roll
    converted.angular.y = sawtooth(-msg.angular.x + off_pitch)
    converted.angular.z = sawtooth(-msg.angular.y + off_yaw)
    return converted

def twist_to_vector(t: Twist) -> np.ndarray:
    """Return a 6D vector representation of ``Twist``."""
    return np.array(
        [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z],
        dtype=float,
    )


def vector_to_twist(vec: Iterable[float]) -> Twist:
    """Create ``Twist`` from a 6-element vector-like object."""
    t = Twist()
    t.linear.x, t.linear.y, t.linear.z = vec[0], vec[1], vec[2]
    t.angular.x, t.angular.y, t.angular.z = vec[3], vec[4], vec[5]
    return t
