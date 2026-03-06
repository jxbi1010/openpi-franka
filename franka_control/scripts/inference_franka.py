import time
from typing import Tuple

import numpy as np
import tqdm
from openpi.training import config as _config
from openpi.policies import policy_config
from openpi_client import image_tools
from franka_control.camera import RealSenseCamera
from franka_control.franka_state_reader import FrankaStateReader
from franka_control.franka_commander import FrankaCommander
from franka_control.robotiq import RobotiqInterface
from franky import *

# from net_franky import setup_net_franky
# setup_net_franky(ip="172.16.0.10", port=18812)
# from net_franky.franky import Robot, RealtimeConfig

# DROID data collection frequency -- we slow down execution to match this frequency
DROID_CONTROL_FREQUENCY = 15


class RobotManager:
    def __init__(self):
        self.robot = Robot("172.16.0.2",realtime_config=RealtimeConfig.Ignore)
        self.gripper = RobotiqInterface('/dev/ttyUSB0')
        self.state_reader = FrankaStateReader(self.robot,self.gripper)
        self.commander = FrankaCommander(self.robot,self.gripper)

    def get_state(self):
        return self.state_reader.get_state('joint_position'), self.state_reader.get_state('gripper_position')

    def get_cartesian_state(self):
        return self.state_reader.get_state('cartesian_position')

    def execute_action(self, action_chunk: np.ndarray, action_type: str = "joint_velocity",execute_gripper: bool = False) -> None:
        self.commander.execute_action(action_chunk, action_type=action_type,execute_gripper=execute_gripper)

def prepare_inference_input(
    exterior_image: np.ndarray,
    wrist_image: np.ndarray,
    joint_position: np.ndarray,
    gripper_position: np.ndarray,
    prompt: str,
    target_size: Tuple[int, int] = (224, 224),
) -> dict:
    """
    Prepare input dictionary for policy inference.
    
    Args:
        exterior_image: RGB image from external camera, shape (H, W, 3), dtype uint8
        wrist_image: RGB image from wrist camera, shape (H, W, 3), dtype uint8
        joint_position: Joint positions, shape (7,), dtype float32
        gripper_position: Gripper position, shape (1,), dtype float32
        prompt: Language instruction string
        target_size: Target image size (height, width), default (224, 224)
        
    Returns:
        Dictionary with keys expected by the policy:
        - "observation/exterior_image_1_left": resized RGB image, shape (224, 224, 3), dtype uint8
        - "observation/wrist_image_left": resized RGB image, shape (224, 224, 3), dtype uint8
        - "observation/joint_position": joint positions, shape (7,), dtype float32
        - "observation/gripper_position": gripper position, shape (1,), dtype float32
        - "prompt": language instruction string
    """
    # Resize and pad images to target size
    exterior_resized = image_tools.resize_with_pad(exterior_image, target_size[0], target_size[1])
    wrist_resized = image_tools.resize_with_pad(wrist_image, target_size[0], target_size[1])
    
    joint_position = np.asarray(joint_position, dtype=np.float32)
    
    gripper_position = np.asarray(gripper_position, dtype=np.float32)
    if gripper_position.ndim == 0:
        gripper_position = gripper_position[np.newaxis]
    
    return {
        "observation/exterior_image_1_left": exterior_resized,
        "observation/wrist_image_left": wrist_resized,
        "observation/joint_position": joint_position,
        "observation/gripper_position": gripper_position,
        "prompt": prompt,
    }


# Example usage:
if __name__ == "__main__":
    # Rollout parameters (matching DROID main.py)
    max_timesteps = 600
    open_loop_horizon = 8  # Re-query policy every 8 steps
    execute_horizon = 8

    model_name = "pi05_droid"

    # Initialize policy
    print("Initializing Policy...")
    checkpoint_dir = f"checkpoints/{model_name}"
    config = _config.get_config(f"{model_name}")
    policy = policy_config.create_trained_policy(config, checkpoint_dir)


    print("Initializing robot manager...")
    robot = RobotManager()

    # Start RealSense cameras
    exterior_camera = RealSenseCamera(
        serial_number="254522070421",
        width=640,
        height=480,
        fps=30,
    )
    wrist_camera = RealSenseCamera(
        serial_number="254522070220",
        width=640,
        height=480,
        fps=30,
    )


    while True:

        if input("Moving to initial joint position? (enter y or n)").lower() == "y":
            init_joint_pos = np.array([[0.011335312069000092,0.2120501795277465,-0.01088267017501314,-2.01687728054916,-0.036302827384647,2.3479571350249087,0.09651094572890309,0.0]])
            robot.execute_action(init_joint_pos, action_type="joint_states")

        instruction = input("Enter instruction: ")

        actions_from_chunk_completed = 0
        pred_action_chunk = None

        print("Running rollout... press Ctrl+C to stop early.")
        bar = tqdm.tqdm(range(max_timesteps))

        for t_step in bar:
            start_time = time.time()
            try:
                # Re-query policy every open_loop_horizon steps
                if actions_from_chunk_completed == 0 or actions_from_chunk_completed >= open_loop_horizon:
                    actions_from_chunk_completed = 0

                    # Get current observation
                    exterior_img = exterior_camera.get_image()
                    wrist_img = wrist_camera.get_image()
                    joint_pos, gripper_pos = robot.get_state()

                    inference_input = prepare_inference_input(
                        exterior_image=exterior_img,
                        wrist_image=wrist_img,
                        joint_position=joint_pos,
                        gripper_position=gripper_pos,
                        prompt=instruction,
                    )

                    pred_action_chunk = policy.infer(inference_input)["actions"]
                    assert pred_action_chunk.shape[1] == 8, (
                        f"Expected 8 action dims (7 joint velocity + 1 gripper), got {pred_action_chunk.shape[1]}"
                    )

                # Select current action from chunk
                action = pred_action_chunk[actions_from_chunk_completed:actions_from_chunk_completed+execute_horizon].copy()
                actions_from_chunk_completed += execute_horizon

                # Binarize gripper action (matching DROID main.py)
                if action[-1,-1].item() > 0.5:
                    action = np.concatenate([action[:,:-1], np.ones((execute_horizon,1))],axis=-1)
                else:
                    action = np.concatenate([action[:,:-1], np.zeros((execute_horizon,1))],axis=-1)

                # Clip all dimensions to [-1, 1]
                action = np.clip(action, -1, 1)

                # Execute single action
                cartesian_pose = robot.get_cartesian_state()
                print(f"cartesian_pose: {cartesian_pose}")
                print(f"action: {action[-1,-1]}")
                robot.execute_action(action, action_type="joint_velocity",execute_gripper=True)


                # Sleep to match DROID control frequency (15 Hz)
                elapsed_time = time.time() - start_time
                if elapsed_time < 1 / DROID_CONTROL_FREQUENCY:
                    time.sleep(1 / DROID_CONTROL_FREQUENCY - elapsed_time)

            except KeyboardInterrupt:
                break

        if input("Do another rollout? (enter y or n) ").lower() != "y":
            break
        
