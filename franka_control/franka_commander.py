import threading
import time
import numpy as np
from typing import Tuple, Optional, List
from franky import *
# from net_franky import setup_net_franky
# setup_net_franky(ip="172.16.0.10", port=18812)
# from net_franky.franky import *

class FrankaCommander:
    """Commander for executing action chunks on Franka robot."""
    
    def __init__(self, robot, gripper):
        """
        Initialize the Franka commander.
        
        Args:
            robot_ip: Robot controller IP/hostname (default: 172.16.0.2)
            controller: Controller type, either "joint" or "cartesian" (default: "joint")
            gripper_speed: Speed for gripper movements (default: 0.1)
        """
        self.robot = robot
        self.robot.relative_dynamics_factor = 0.1
        self.robot.set_joint_impedance([3000, 3000, 3000, 2500, 2500, 2000, 2000]) # Lower values = more compliant, higher = stiffer
        self.control_frequency = 15
        self.action_duration = int(1000 / self.control_frequency)

        self.gripper = gripper

        # Gripper: execute positions in sequence at control frequency, then hold last
        self._gripper_sequence: Optional[List[float]] = None
        self._gripper_index: int = 0
        self._gripper_lock = threading.Lock()
        self._gripper_stop = threading.Event()
        self._gripper_thread = threading.Thread(target=self._gripper_publish_loop, daemon=True)
        self._gripper_thread.start()

    def _extract_actions(self, action_chunk: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:

        if action_chunk.ndim == 1:
            joint_positions = action_chunk[:7]
            gripper_positions = action_chunk[7]
        elif action_chunk.ndim == 2:
            joint_positions = action_chunk[:, :7]
            gripper_positions = action_chunk[:, 7]
        else:
            raise ValueError(f"Action chunk must be 1D or 2D, got {action_chunk.ndim}D")
        return joint_positions, gripper_positions

    def _gripper_publish_loop(self) -> None:
        """Execute gripper positions in sequence at control frequency, then hold last."""
        period = 1.0 / self.control_frequency
        while not self._gripper_stop.is_set():
            with self._gripper_lock:
                seq = self._gripper_sequence
                idx = self._gripper_index
                if seq is not None and len(seq) > 0:
                    target = seq[min(idx, len(seq) - 1)]
                    if idx < len(seq):
                        self._gripper_index = idx + 1
                else:
                    target = None
            if target is not None:
                try:
                    self.gripper.move(target)
                except Exception:
                    pass  # Avoid crashing the loop on transient errors
            time.sleep(period)

    def _set_gripper_sequence(self, gripper_positions: np.ndarray) -> None:
        """Set gripper sequence (0-255). Background thread executes in order at control frequency."""
        positions = np.atleast_1d(gripper_positions)
        sequence = [float(p * 255) for p in positions]
        with self._gripper_lock:
            self._gripper_sequence = sequence
            self._gripper_index = 0

    def _execute_joint_states(self, joint_positions: np.ndarray) -> None:
        joint_waypoints = [JointWaypoint(joint_pos.tolist()) for joint_pos in joint_positions]
        joint_motion = JointWaypointMotion(joint_waypoints)
        self.robot.move(joint_motion)

    def _execute_joint_velocity(self, joint_velocities: np.ndarray) -> None:
        joint_velocity_waypoints = [JointVelocityWaypoint(joint_vel.tolist(),max_total_duration=Duration(self.action_duration)) for joint_vel in joint_velocities]
        joint_velocity_waypoints.append(JointVelocityWaypoint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        joint_motion = JointVelocityWaypointMotion(joint_velocity_waypoints)

        self.robot.move(joint_motion)

    # def _execute_single_joint_velocity(self, joint_velocity: np.ndarray) -> None:
    #     joint_velocity_motion = JointVelocityMotion(joint_velocity,duration=Duration(self.action_duration))
    #     self.robot.move(joint_velocity_motion)

    def execute_action(
        self, 
        action_chunk: np.ndarray, 
        execute_gripper: bool = True,
        action_type: str = "joint_velocity"
    ) -> None:
        franka_actions, gripper_positions = self._extract_actions(action_chunk)

        if execute_gripper:
            self._set_gripper_sequence(gripper_positions)

        if action_type == "joint_velocity":
            self._execute_joint_velocity(franka_actions)
        elif action_type == "joint_states":
            self._execute_joint_states(franka_actions)

        # elif action_type == "joint_velocity_single":
        #     self._execute_single_joint_velocity(franka_actions)



if __name__ == "__main__":

    from .franka_state_reader import FrankaStateReader
    from .robotiq import RobotiqInterface

    robot = Robot("172.16.0.2",realtime_config=RealtimeConfig.Ignore)
    gripper = RobotiqInterface('/dev/ttyUSB0')

    commander = FrankaCommander(robot, gripper)
    state_reader = FrankaStateReader(robot, gripper)

    ## test of joint states controller
    # action_chunk = np.array([
    # [-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7, 1.0],
    # [0.0, 0.3, 0.3, -1.5, -0.2, 1.5, 0.8, 0.0],
    # [0.1, 0.4, 0.3, -1.4, -0.3, 1.7, 0.9, 1.0],
    # [-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7, 0.0]])

    # commander.execute_action(action_chunk, action_type="joint_states", execute_gripper=True)
    # current_joint_pos = state_reader.get_state('joint_position')
    # current_gripper_pos = state_reader.get_state('gripper_position')
    # print(f"current_joint_pos: {current_joint_pos}")
    # print(f"current_gripper_pos: {current_gripper_pos}")

    # test of joint velocity controller
    init_joint_pos = state_reader.get_state()
    joint_velocities = np.array([
    [0.1, 0.3, -0.1, 0.0, 0.1, -0.2, 0.4, 0.0], 
    [-0.1, -0.3, 0.1, -0.0, -0.1, 0.2, -0.4, 1.0],
    [-0.1, -0.3, 0.1, -0.0, -0.1, 0.2, -0.4, 1.0],
    [0.1, 0.3, -0.1, 0.0, 0.1, -0.2, 0.4, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ])
    commander.execute_action(joint_velocities, action_type="joint_velocity")
    new_joint_pos = state_reader.get_state()

    target_joint_pos = init_joint_pos + np.array([-0.1, -0.3, 0.1, -0.0, -0.1, 0.2, -0.4]) * 0.2
    print(f"delta_joint_pos: {new_joint_pos - target_joint_pos}")
