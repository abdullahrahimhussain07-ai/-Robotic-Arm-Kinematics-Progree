# 3-DOF Robotic Arm Kinematics Assembly & Path Trace

![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Kinematics](https://img.shields.io/badge/Kinematics-FK%20%7C%20Analytical%20IK-2EA44F?style=for-the-badge)
![Trajectory](https://img.shields.io/badge/Trajectory-Quintic%20Pick%20%26%20Place-FF6F00?style=for-the-badge)
![Internship](https://img.shields.io/badge/Progree-Robotics%20%26%20Automation%20Intern-0A66C2?style=for-the-badge)

A Python simulation framework for a spatial **3-DOF (R-R-R: Base Yaw, Shoulder Pitch, Elbow Pitch)** robotic manipulator performing smooth multi-point pick-and-place operations in 3D space. Developed during the **Progree Robotics & Automation Internship**, this project combines closed-form inverse kinematics, minimum-jerk (quintic) trajectory generation, safety/singularity checks, and multi-panel visualization.

## Table of Contents
1. Problem Statement & Engineering Motivation
2. System Architecture & Motion Pipeline
3. Core Classes & Functions
4. Simulation Outputs & Verification
5. Repository Structure
6. Installation & Quickstart
7. Author

## 1. Problem Statement & Engineering Motivation

Robotic manipulators executing pick-and-place tasks in 3D workspaces require reliable coordinate mapping and smooth joint motion:
- Spatial Coordinate Mapping: Translating Cartesian pick, lift, transit, and place targets (x, y, z) into valid joint angles (q1, q2, q3) without workspace reach errors.
- Motion Smoothness: Avoiding abrupt velocity or acceleration steps across multi-point sequences using quintic polynomial interpolation.
- Workspace & Safety Guardrails: Enforcing joint limits, floor clearance (z >= min_z), and monitoring proximity to elbow singularities.

## 2. System Architecture & Motion Pipeline

```text
Pick Bin & Place Targets ---> [ Quintic Trajectory Planner ] ---> ( 3D Path + Gripper State )
                                                                            |
                                                                            v
[ Multi-Panel Plot ] <--- [ Safety & Limit Validator ] <--- [ Analytical IK + FK Check ]
cd C:\RoboticsProjects\Task_3_Robotic_Arm_Kinematics

@'
# 3-DOF Robotic Arm Kinematics Assembly & Path Trace

![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Kinematics](https://img.shields.io/badge/Kinematics-FK%20%7C%20Analytical%20IK-2EA44F?style=for-the-badge)
![Trajectory](https://img.shields.io/badge/Trajectory-Quintic%20Pick%20%26%20Place-FF6F00?style=for-the-badge)
![Internship](https://img.shields.io/badge/Progree-Robotics%20%26%20Automation%20Intern-0A66C2?style=for-the-badge)

A Python simulation framework for a spatial **3-DOF (R-R-R: Base Yaw, Shoulder Pitch, Elbow Pitch)** robotic manipulator performing smooth multi-point pick-and-place operations in 3D space. Developed during the **Progree Robotics & Automation Internship**, this project combines closed-form inverse kinematics, minimum-jerk (quintic) trajectory generation, safety/singularity checks, and multi-panel visualization.

## Table of Contents
1. Problem Statement & Engineering Motivation
2. System Architecture & Motion Pipeline
3. Core Classes & Functions
4. Simulation Outputs & Verification
5. Repository Structure
6. Installation & Quickstart
7. Author

## 1. Problem Statement & Engineering Motivation

Robotic manipulators executing pick-and-place tasks in 3D workspaces require reliable coordinate mapping and smooth joint motion:
- Spatial Coordinate Mapping: Translating Cartesian pick, lift, transit, and place targets (x, y, z) into valid joint angles (q1, q2, q3) without workspace reach errors.
- Motion Smoothness: Avoiding abrupt velocity or acceleration steps across multi-point sequences using quintic polynomial interpolation.
- Workspace & Safety Guardrails: Enforcing joint limits, floor clearance (z >= min_z), and monitoring proximity to elbow singularities.

## 2. System Architecture & Motion Pipeline

Pick Bin & Place Targets ---> [ Quintic Trajectory Planner ] ---> ( 3D Path + Gripper State )
                                                                            |
                                                                            v
[ Multi-Panel Plot ] <--- [ Safety & Limit Validator ] <--- [ Analytical IK + FK Check ]

## 3. Core Classes & Functions

### `ThreeDOFArm`
Models the spatial 3-DOF serial manipulator (`d1 = 0.20 m`, `l2 = 0.35 m`, `l3 = 0.30 m`) with configurable joint angle bounds and minimum height clearance (`min_z = 0.0 m`).
- `link_positions(q1, q2, q3)`: Computes 3D link joint coordinates for `[BaseTop, Shoulder, Elbow, EndEffector]`.
- `forward_kinematics(q1, q2, q3)`: Evaluates end-effector `(x, y, z)` coordinates from joint angles in radians.
- `inverse_kinematics(x, y, z, elbow_up=False)`: Solves closed-form analytical IK for `(q1, q2, q3)` with reach and `z >= min_z` validation.
- `validate_trajectory(q_deg, path_xyz)`: Checks joint limits, floor clearance, and elbow singularity margin (`|sin(q3)|`).

### Trajectory & Visualization Functions
- `quintic_segment(p_start, p_end, steps)`: Generates smooth fifth-order polynomial motion between two 3D points.
- `build_pick_place_cycle(pick_pt, place_pt, clearance, steps_per_leg)`: Builds a single pick, lift, transit, place, and retract cycle along with gripper state (`0` = Open, `1` = Closed).
- `generate_multi_point_trajectory(pick_bin, place_targets, clearance=0.12, steps_per_leg=35)`: Chains pick-and-place cycles across multiple target coordinates.
- `plot_workspace_envelope(ax3d, arm, alpha=0.06)`: Plots the outer reachable workspace surface in 3D.
- `plot_arm_pose(ax3d, arm, q_rad, color, label)`: Renders manipulator links and joint markers at key poses.

## 4. Simulation Outputs & Verification

Running `kinematics_path_trace.py` verifies FK/IK round-trip accuracy, runs trajectory safety checks, and exports `pick_and_place_trace.png` containing:
- 3D Workspace & Poses: Reachable envelope, end-effector path, pick/drop markers, and arm configurations at key waypoints.
- Joint Trajectories: Joint angles (`q1`, `q2`, `q3`) in degrees over time.
- Joint Velocities & Accelerations: Time derivatives (`deg/s` and `deg/s^2`) across the multi-point sequence.
- Gripper State: Discrete open/closed status synchronized with pick and place phases.

## 5. Repository Structure

-Robotic-Arm-Kinematics-Progree/
├── kinematics_path_trace.py   # 3-DOF kinematics, quintic planner, safety checks & plotting
├── pick_and_place_trace.png   # Generated 3D manipulator & trajectory visualization
├── .gitignore                 # Git ignore rules
└── README.md                  # Project documentation

## 6. Installation & Quickstart

git clone https://github.com/abdullahrahimhussain07-ai/-Robotic-Arm-Kinematics-Progree.git
cd -Robotic-Arm-Kinematics-Progree
python kinematics_path_trace.py

## 7. Author
**Abdullah Rahim Hussain**  
BS Robotics — Department of Electrical & Computer Engineering, PAF-IAST  
*Robotics & Automation Intern @ Progree*
