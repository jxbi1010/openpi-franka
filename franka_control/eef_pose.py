#!/usr/bin/env python3
"""
Compute Franka end-effector (EEF) pose in Cartesian space from joint states.

Uses the franky library's robot model for forward kinematics.
Run with the robot controller connected, or pass joint positions for offline computation.
"""

from typing import Tuple

import numpy as np

# For local robot: from franky import *
# For remote robot via net_franky:
# from net_franky import setup_net_franky
# setup_net_franky(ip="172.16.0.10", port=18812)
# from net_franky.franky import *
from franky import Affine, Frame, Robot, RealtimeConfig


def compute_eef_pose(
    robot: Robot,
    joint_positions: np.ndarray,
    frame: Frame = Frame.EndEffector,
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute end-effector pose in Cartesian space from joint positions.

    Args:
        robot: Franky Robot instance (provides the kinematics model).
        joint_positions: Joint angles in radians, shape (7,) or (7, 1).
        frame: Frame to compute pose for. Default is Frame.EndEffector.
               Use Frame.Flange for flange pose instead.

    Returns:
        position: Translation (x, y, z) in meters, shape (3,).
        quaternion: Orientation as quaternion (x, y, z, w), shape (4,).
    """
    q = np.asarray(joint_positions, dtype=np.float64).reshape(7)
    f_t_ee = Affine()  # Default flange-to-EE transform
    ee_t_k = Affine()  # Output: pose of frame in base frame

    robot.model.pose(frame, q, f_t_ee, ee_t_k)

    position = np.asarray(ee_t_k.translation).flatten()
    quaternion = np.asarray(ee_t_k.quaternion).flatten()

    return position, quaternion


def compute_eef_pose_dict(
    robot: Robot,
    joint_positions: np.ndarray,
    frame: Frame = Frame.EndEffector,
) -> dict:
    """Compute end-effector pose and return as a dictionary.

    Returns:
        dict with keys:
            - "position": (x, y, z) in meters
            - "quaternion": (x, y, z, w)
            - "matrix": 4x4 homogeneous transformation matrix
    """
    position, quaternion = compute_eef_pose(robot, joint_positions, frame)
    return {
        "position": position,
        "quaternion": quaternion,
    }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Compute Franka EEF pose from joint states")
    parser.add_argument(
        "--host",
        default="172.16.0.2",
        help="Robot controller IP/hostname",
    )
    parser.add_argument(
        "--joints",
        type=float,
        nargs=7,
        default=None,
        metavar=("q1", "q2", "q3", "q4", "q5", "q6", "q7"),
        help="Joint positions in radians (default: read from robot)",
    )
    args = parser.parse_args()

    print("Connecting to robot...")
    robot = Robot(args.host, realtime_config=RealtimeConfig.Ignore)

    if args.joints is not None:
        joint_positions = np.array(args.joints)
        print(f"Using provided joint positions: {joint_positions}")
    else:
        joint_positions = np.array(robot.current_joint_state.position).flatten()
        print(f"Using current joint positions from robot: {joint_positions}")

    position, quaternion = compute_eef_pose(robot, joint_positions)

    print("\nEnd-effector pose (in base frame):")
    print(f"  Position (x, y, z) [m]: {position}")
    print(f"  Quaternion (x, y, z, w): {quaternion}")
