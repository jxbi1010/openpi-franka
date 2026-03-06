# openpi-franka

Franka + Robotiq + RealSense interface for [openpi](https://github.com/Physical-Intelligence/openpi). This add-on provides hardware interfaces to run openpi policies on a DROID-style Franka setup with two RealSense cameras and a Robotiq gripper.

## Hardware

- **Franka Panda** robot (via [franky](https://github.com/niklasreidel/franky))
- **Robotiq 2F-85** gripper (or compatible)
- **Intel RealSense** D435 or similar (two cameras: wrist + exterior)

## Installation

### 1. Install openpi

First install openpi from the [official repo](https://github.com/Physical-Intelligence/openpi):

```bash
git clone --recurse-submodules https://github.com/Physical-Intelligence/openpi.git
cd openpi
GIT_LFS_SKIP_SMUDGE=1 uv sync
GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .
```

### 2. Install this add-on

Clone and install into the same environment:

```bash
git clone https://github.com/YOUR_USERNAME/openpi-franka.git
cd openpi-franka
uv pip install -e .
```

The inference script requires openpi and openpi-client, which are installed in step 1.

## Usage

### Robot and gripper only

```python
from franky import Robot, RealtimeConfig
from franka_control.robotiq import RobotiqInterface
from franka_control.franka_state_reader import FrankaStateReader
from franka_control.franka_commander import FrankaCommander

robot = Robot("172.16.0.2", realtime_config=RealtimeConfig.Ignore)
gripper = RobotiqInterface('/dev/ttyUSB0')

state_reader = FrankaStateReader(robot, gripper)
commander = FrankaCommander(robot, gripper)

# Read state
joint_pos = state_reader.get_state('joint_position')
gripper_pos = state_reader.get_state('gripper_position')

# Execute action (joint velocity, 8 dims: 7 joints + gripper)
import numpy as np
action = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]])  # open gripper
commander.execute_action(action, action_type="joint_velocity", execute_gripper=True)
```

### Cameras

```python
from franka_control.camera import RealSenseCamera

exterior_camera = RealSenseCamera(serial_number="YOUR_SERIAL", width=640, height=480, fps=30)
wrist_camera = RealSenseCamera(serial_number="YOUR_SERIAL", width=640, height=480, fps=30)

rgb = exterior_camera.get_image()  # shape (H, W, 3), uint8
```

### Full inference with openpi

From the **openpi** repo directory (where `checkpoints/` lives):

```bash
uv run python -m franka_control.inference
```

Or in Python:

```python
from franka_control.inference import RobotManager, prepare_inference_input
from franka_control.camera import RealSenseCamera

robot = RobotManager()
exterior_camera = RealSenseCamera(serial_number="...", width=640, height=480, fps=30)
wrist_camera = RealSenseCamera(serial_number="...", width=640, height=480, fps=30)

# Get observation and run policy
exterior_img = exterior_camera.get_image()
wrist_img = wrist_camera.get_image()
joint_pos, gripper_pos = robot.get_state()

inference_input = prepare_inference_input(
    exterior_image=exterior_img,
    wrist_image=wrist_img,
    joint_position=joint_pos,
    gripper_position=gripper_pos,
    prompt="pick up the fork",
)

# policy.infer(inference_input)["actions"]
```

## Configuration

- **Robot IP**: Default `172.16.0.2` — change in `RobotManager` or when creating `Robot`.
- **Gripper port**: Default `/dev/ttyUSB0` — change in `RobotiqInterface('/dev/ttyUSB0')`.
- **Camera serials**: Update in `inference.py` or when creating `RealSenseCamera`.

## Examples

Scripts in `franka_control/examples/` demonstrate franky usage:

- `joint_test.py` — joint space motions
- `joint_velocity.py` — joint velocity control
- `cartesian_test.py` — Cartesian motions
- `get_state.py` — reading robot state
- `compliance_test.py` — force reactions
- `realtime_motion.py` — asynchronous motion

Run from the openpi-franka directory:

```bash
uv run python franka_control/examples/get_state.py
```

## License

MIT
