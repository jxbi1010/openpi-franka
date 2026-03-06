#!/usr/bin/env python3
"""
Continuously read and print Franka robot joint positions.

Uses the franky library. Run with the robot controller at the default host
or pass --host. Press Ctrl+C to stop.
"""

import argparse
import signal
import sys
import time
from typing import Optional, Dict, Any

import rpyc
import numpy as np
from franky import *

# from net_franky import setup_net_franky
# setup_net_franky(ip="172.16.0.10", port=18812)
# from net_franky.franky import *



class FrankaStateReader:
    """Class for reading and managing Franka robot state."""
    
    def __init__(self, robot, gripper, read_rate: float = 30.0):

        self.robot = robot
        self.gripper = gripper
        self.read_rate = read_rate
        self.period = 1.0 / read_rate if read_rate > 0 else 0.0
        self._shutdown = False
        
    def _read_joint_position(self) -> Optional[np.ndarray]:
        state = self.robot.current_joint_state
        return np.array(state.position)

    def _read_cartesian_pose(self) -> Optional[np.ndarray]:
        eef_pose = self.robot.current_cartesian_state.pose.end_effector_pose
        position = np.array(eef_pose.translation).flatten()
        quaternion = np.array(eef_pose.quaternion).flatten()
        return np.concatenate([position, quaternion])

    def _read_gripper_position(self) -> Optional[np.ndarray]:
        gripper_pos = self.gripper.position/255.0
        return np.array([gripper_pos], dtype=np.float32)
    
    def get_state(self, state_type: str = "joint_position") -> Optional[Any]:

        if state_type == "joint_position":
            return self._read_joint_position()
        elif state_type == "cartesian_position":
            return self._read_cartesian_pose()
        elif state_type == "gripper_position":
            return self._read_gripper_position()
        else:
            print(f"Unknown state type: {state_type}", file=sys.stderr)
            return None
    
    
    def read_loop(self, state_type: str = "joint_position", print_output: bool = True):
        
        print(f"Reading {state_type} at {self.read_rate} Hz. Press Ctrl+C to stop.\n")
        
        while not self._shutdown:
            t0 = time.perf_counter()
            
            state_data = self.get_state('joint_position')
            cartesian_pose = self.get_state('cartesian_position')
            gripper_pos = self.get_state('gripper_position')

            if state_data is not None and print_output:
                print("joint_position: ", state_data)
                print("cartesian_pose: ", cartesian_pose)
                print("gripper_pos: ", gripper_pos)
            
            elapsed = time.perf_counter() - t0
            if elapsed < self.period:
                time.sleep(self.period - elapsed)
            
        print("\nStopped.")
    
    def stop(self):
        """Stop the reading loop."""
        self._shutdown = True
    
    def setup_signal_handler(self):
        """Setup signal handler for graceful shutdown."""
        def sigint_handler(_signum, _frame):
            self.stop()
        
        signal.signal(signal.SIGINT, sigint_handler)


if __name__ == "__main__":
    from .robotiq import RobotiqInterface


    robot = Robot("172.16.0.2",realtime_config=RealtimeConfig.Ignore)
    gripper = RobotiqInterface('/dev/ttyUSB0')
    # Create robot reader instance
    reader = FrankaStateReader(robot=robot, gripper=gripper)
    
    # Setup signal handler
    reader.setup_signal_handler()
    
    # Start reading loop
    reader.read_loop()
