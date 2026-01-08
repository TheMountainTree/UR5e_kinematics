# UR5e Kinematics and Control for ManiSkill3

This project provides a comprehensive suite for UR5e robot kinematics, visualization, and control using the SAPIEN physics engine and ManiSkill3 framework. It includes forward/inverse kinematics computation, a unified visualization module, gripper control, and trajectory planning.

## Project Structure

```
ur5e_kinematics/
├── README.md                    # This file
├── ur5e_robot.py                # Core Robot Logic (Model)
├── ur5e_visualization.py        # Core Visualization (View)
├── ur5e_kinematics_complete.py  # Kinematics CLI & Demo
├── ur5e_gripper_control.py      # Gripper Control Demo
├── ur5e_planner.py              # Trajectory Planner App
├── create_combined_urdf.py      # Tool to merge UR5e and Gripper URDFs
├── fix_urdf.py                  # Tool to fix mesh paths in URDFs
└── ur5e_robotiq.urdf            # Combined Robot+Gripper URDF (generated)
```

## Requirements

- **Conda Environment**: `robot_env`
- **Python**: 3.10+
- **Dependencies**:
  - `sapien`: Physics and Rendering
  - `mani_skill`: Simulation Framework
  - `numpy`: Numerical Computation

## Setup

1.  **Activate Environment**:
    ```bash
    conda activate robot_env
    ```

2.  **Prepare URDFs**:
    The project is self-contained with assets in `assets/`.
    
    Ensure `ur5e_robotiq.urdf` exists. If not, run:
    ```bash
    python create_combined_urdf.py
    ```

## Module Overview

### 1. `ur5e_robot.py` (Model)
Encapsulates all robot-specific logic.
- **Class**: `UR5eRobot`
- **Features**:
    - Automatic loading of URDF (Standard or with Gripper).
    - Joint management (Arm indices, Gripper mimic joints).
    - **FK**: `forward_kinematics(q)`
    - **IK**: `inverse_kinematics(target)` (Damped Least Squares).
    - **Control**: `set_qpos(q)`, `set_gripper(val)`.

### 2. `ur5e_visualization.py` (View)
Handles SAPIEN rendering and visualization.
- **Class**: `UR5eVisualizer`
- **Features**:
    - SAPIEN Scene setup (Lights, Ground).
    - Viewer configuration.
    - End-effector trail visualization (`add_trail_point`).

## Usage Guides

### Kinematics Demo
Test mathematical accuracy or visualize simple movements.

```bash
# Run math tests
python ur5e_kinematics_complete.py --test

# Visualize Cartesian motion
python ur5e_kinematics_complete.py --visualize

# Visualize specific angles
python ur5e_kinematics_complete.py --visualize --angles 0.0 -1.57 1.57 -1.57 -1.57 0.0
```

### Gripper Control
Demonstrates gripper actuation using the unified robot model.

```bash
python ur5e_gripper_control.py
```

### Trajectory Planner
Complex application showing FSM-based control, path planning (Lines/Circles), and trail visualization.

```bash
python ur5e_planner.py
```

## Technical Details

### Architecture (MVC-ish)
- **Model**: `UR5eRobot` holds state and physics logic.
- **View**: `UR5eVisualizer` handles rendering.
- **Controller**: Scripts like `ur5e_planner.py` contain the application logic driving the Model and updating the View.

### Inverse Kinematics
Implemented using **Damped Least Squares (Levenberg-Marquardt)** with Jacobian Transpose.
- Supports position targets (3D).
- Includes multi-start capability to avoid local minima.

### Coordinate Systems
- **Base Frame**: SAPIEN World Frame (z-up).
- **End-Effector**: `tool0` (attachment point for gripper).

## Troubleshooting

- **Import Errors**: Ensure you are in `robot_env`.
- **URDF Errors**: Run `fix_urdf.py` and `create_combined_urdf.py`.
- **Visualization Issues**: If the window is black, ensure your GPU drivers are compatible with SAPIEN/Vulkan.
