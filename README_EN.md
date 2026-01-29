# Alicia-M SDK

[English Version](README_EN.md) | [中文版](README.md)

**Alicia-M SDK** is a Python toolkit for controlling the "Alicia-M" series robotic arms (with gripper). Built on top of the `RoboCore` library, it provides functionalities to control the arm's movement, operate the gripper, and read posture and status data via serial communication.

## RoboCore: Unified High-Throughput Robotics Library 

This SDK is powered by [RoboCore (Unified High-Throughput Robotics Library)](https://github.com/Synria-Robotics/RoboCore) developed by [Synria Robotics Co., Ltd.](https://synriarobotics.ai)

[![License](https://img.shields.io/badge/License-GPL--3.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)

---

## ✨ Features

| Module | Functionality | Status |
|--------|---------------|--------|
| **Modeling** | URDF/MJCF parsing, Robot model abstraction | ✅ Stable |
| **Forward Kinematics** | NumPy/PyTorch backends, Batch processing | ✅ Stable |
| **Inverse Kinematics** | DLS/Pinv/Transpose methods, Multi-start | ✅ Stable |
| **Jacobian** | Analytic/Numeric/Autograd methods | ✅ Stable |
| **Transform** | SE(3)/SO(3) operations, Conversions | ✅ Stable |
| **Analysis** | Workspace/Singularity analysis | ✅ Beta |
| **Planning** | Trajectory generation | 🚧 Alpha |
| **Visualization** | Kinematic tree display | ✅ Stable |
| **Configuration** | YAML-based config management | ✅ Stable |

## Key Features

*   **Joint Control**: Supports setting and reading the angles of the six joints, with smooth interpolation for execution.
*   **End-Effector Trajectory**: Cartesian end-effector pose-based trajectory planning and execution.
*   **Gripper Control**: Supports precise angle control or one-click open/close.
*   **Torque Control**: Enable or disable joint motor torque for free-drag teaching.
*   **Zero-Point Setting**: Set the current position as the new zero point.
*   **Status Reading**: Real-time retrieval of joint angles, gripper angle, and end-effector pose.
*   **Automatic Serial Connection**: Automatically searches for serial ports or allows manual specification.
*   **Drag Teaching**: Record pose points by dragging and execute the trajectory.
*   **SparkVis Integration**: Supports WebSocket UI bidirectional sync and data recording.
*   **Smart Logging System**: Supports log level filtering to control console output verbosity.
*   **RoboCore Integration**: Integrated high-performance kinematics and trajectory planning library.

## Project Structure

```
├── alicia_m_sdk
│   ├── api
│   │   ├── synria_robot_api.py      # User-level API
│   │   └── firmware_version.json    # Firmware version/metadata
│   ├── execution
│   │   ├── drag_teaching.py         # Drag teaching
│   │   ├── hardware_executor.py     # Hardware executor
│   │   ├── sparkvis.py              # SparkVis WebSocket bridge
│   │   └── trajectory_executor.py   # Trajectory executor
│   ├── hardware
│   │   ├── comm_manager.py          # Serial port management/reconnection
│   │   ├── serial_comm.py           # Serial communication
│   │   ├── data_parser.py           # Data parser
│   │   └── servo_driver.py          # Servo driver
│   ├── __init__.py
│   └── utils
│       ├── calculate.py             # Calculation utilities
│       ├── control_utils.py         # Control utilities
│       ├── fps_utils.py             # FPS statistics
│       ├── trajectory_utils.py      # Trajectory utilities
│       ├── unit_conversion.py       # Unit conversion
│       └── logger/                  # Logging system
├── docs
│   ├── api_reference.md             # API reference
│   ├── examples.md                  # Examples guide
│   └── ...                          # Other documentation
├── examples
│   ├── 00_demo_read_version.py      # Read firmware version
│   ├── 01_demo_switch_mode.py       # Switch control mode
│   ├── 02_demo_monitor_arm_status_simple.py  # Status monitoring
│   ├── 03_demo_read_state.py        # Read state
│   ├── 04_demo_move_gripper.py      # Gripper control
│   ├── 05_demo_move_joint.py        # Joint motion
│   ├── 07_demo_forward_kinematics.py   # Forward kinematics
│   ├── 08_demo_inverse_kinematics.py   # Inverse kinematics
│   ├── 09_demo_joint_traj.py        # Joint trajectory
│   ├── 10_demo_cartesian_traj.py    # Cartesian trajectory
│   ├── 11_demo_slider_control_joint.py  # Slider control
│   └── 12_demo_sparkvis.py          # SparkVis UI bidirectional sync
```

## Quick Start

1.  **Installation**:
```bash
# Clone the repository
git clone https://github.com/Synria-Robotics/Alicia-M-SDK.git
cd Alicia-M-SDK

# Install dependencies (Python 3.8+ recommended)
pip install -e .
```

2.  **Run Examples**:
```bash
cd examples
python3 00_demo_read_version.py    # Read firmware version
python3 03_demo_read_state.py      # Read status
python3 04_demo_move_gripper.py    # Gripper control
python3 05_demo_move_joint.py      # Joint movement
```

3.  **SparkVis Visualization** (Optional):
```bash
# Start SparkVis bridge for UI bidirectional sync
python3 examples/12_demo_sparkvis.py
```

## Documentation

**English Documentation:**
*   [API Reference](docs/api_reference.md)
*   [Examples Guide](docs/examples.md)
*   [MIT Mode Usage](docs/mit_mode_usage.md)
*   [Status Bit Parsing](docs/status_bit_parsing.md)
*   [High Frequency Sync Communication](docs/HIGH_FREQUENCY_SYNC_COMMUNICATION.md)

## Example Programs

The `examples/` directory contains multiple demonstration scripts:

| Example | Description |
|---------|-------------|
| `00_demo_read_version.py` | Read firmware version information |
| `01_demo_switch_mode.py` | Switch control mode |
| `02_demo_monitor_arm_status_simple.py` | Simple status monitoring |
| `03_demo_read_state.py` | Read joint and end-effector state |
| `04_demo_move_gripper.py` | Gripper control example |
| `05_demo_move_joint.py` | Joint position control |
| `07_demo_forward_kinematics.py` | Forward kinematics calculation |
| `08_demo_inverse_kinematics.py` | Inverse kinematics solving |
| `09_demo_joint_traj.py` | Joint space trajectory execution |
| `10_demo_cartesian_traj.py` | Cartesian space trajectory execution |
| `11_demo_slider_control_joint.py` | Joint control using sliders |
| `12_demo_sparkvis.py` | SparkVis UI bidirectional sync |

For detailed information, please refer to the [Examples Guide](docs/examples.md).

## License

This project is licensed under the [GPL-3.0 License](LICENSE).

## Contact

*   **Development Team**: [Synria Robotics Co., Ltd.](https://synriarobotics.ai)
*   **Technical Support**: support@synriarobotics.ai

---

**Crafted with care by Synria Robotics** 🤖
