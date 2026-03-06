# openpi-franka
**One-line to run openpi on franka using a single workstation.**

Franka + Robotiq + RealSense interface for [openpi](https://github.com/Physical-Intelligence/openpi). This add-on provides hardware interfaces to run openpi policies on a DROID-style Franka setup with two RealSense cameras and a Robotiq gripper. **One-line power on sensors, start Pi-0/0.5 inference, and control franka robot.** — copy the inference script into openpi root and run.


## System Requirements
The current system uses below configs, Ubuntu 22.04 is compulsory. Others are highly encouraged for successful installation and deployment.

| Component | Version |
|-----------|---------|
| **OS** | Ubuntu 22.04 |
| **GPU** | NVIDIA RTX 4090 |
| **CUDA** | 12.8 |
| **libfranka** | 0.19.0 |
| **librealsense2** | 2.56.5 |

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

### 3. Copy inference script to openpi

To enable one-line power-on and inference, copy the inference script into the **root of your openpi repo** (where `checkpoints/` lives):

```bash
cp franka_control/scripts/inference_franka.py /path/to/openpi/
```

## Usage

### Test Robot and gripper only

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

From the **openpi** repo root (where you copied `inference_franka.py` and where `checkpoints/` lives):

```bash
uv run python inference_franka.py
```

## Configuration

- **Robot IP**: Default `172.16.0.2` — change in `RobotManager` or when creating `Robot`.
- **Gripper port**: Default `/dev/ttyUSB0` — change in `RobotiqInterface('/dev/ttyUSB0')`.
- **Camera serials**: Update in `inference_franka.py` or when creating `RealSenseCamera`.

## Examples

Scripts in `franka_control/examples/` demonstrate franky usage:

- `joint_test.py` — joint space motions
- `joint_velocity.py` — joint velocity control
- `cartesian_test.py` — Cartesian motions
- `get_state.py` — reading robot state
- `compliance_test.py` — force reactions
- `realtime_motion.py` — asynchronous motion

Refer to official [Franky Repo](https://github.com/TimSchneider42/franky) for more examples.

Run from the openpi-franka directory:

```bash
uv run python franka_control/examples/get_state.py
```

## License

MIT
