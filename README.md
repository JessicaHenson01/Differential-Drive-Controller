# 🤖 ROS 2 Differential Drive Robot Simulation and Control

## 🚀 Project Overview
This project implements a complete, closed-loop control system for a **Differential Drive Robotic Vehicle** within a simulated environment using the **Robot Operating System 2 (ROS 2)**.

The core objective is to design, model, simulate, and control the robot to achieve stable, goal-directed movement — serving as a foundation for future **Hardware-in-the-Loop (HITL)** deployment.

[Project Presentation](https://docs.google.com/presentation/d/1KPECrR1dM-u0sb_scJ924KAFFRbGSRPeo42Sj7wYiDE/edit?usp=sharing)

---

## 🎯 Key Goals
- Design a 3D robot model using **URDF (Unified Robot Description Format)**  
- Develop a **Robot Simulator Node** to emulate virtual hardware dynamics (kinematics)  
- Implement a robust **PD (Proportional-Derivative) Controller Node** for closed-loop motion control  
- Visualize the robot's movement and sensor data in **RViz**  
- Establish a modular, real-time **node-based architecture** for verification and future expansion  

---

## 🛠️ System Architecture (ROS Nodes)
The control system is composed of four primary ROS 2 nodes and external tools, communicating via specific topics.

| Node | Role | Input Topic(s) | Output Topic(s) | Key Files |
| :--- | :--- | :--- | :--- | :--- |
| **Robot Simulator** | Virtual Hardware (emulates robot physics and odometry based on motor commands) | `/cmd_vel` (Twist messages) | `/joint_state`, `/tf` (Odometry) | `diffdrive_sim.py` |
| **Robot PID Controller** | Control Logic (calculates motor commands to reach a target pose) | `/goal_pose`, `/tf` | `/cmd_vel` | `diffdrive_pid.py` |
| **Robot State Publisher** | Visualization Bridge (publishes transforms and descriptions for RViz) | `/joint_state`, `/tf` | `/robot_description`, `/tf` | `state_publisher.py` |
| **RViz** | Visualization Tool (displays 3D model and navigation goals) | `/robot_description`, `/tf`, `/goal_pose` (from user) | — | `urdf_test.rviz` |

---

## 🦾 URDF Model
The robot model is described in XML using the **URDF format**, defining the geometry, links (chassis, wheels, LIDAR dome), and continuous joints.  
This model is loaded into RViz via the **Robot State Publisher** node for accurate visualization.

**File:** `basic_robot.urdf` (or `r2d2.urdf.xml` for an alternate model)

---

## ⚙️ PD Controller Logic
The `diffdrive_pid.py` node executes the motion control logic, following a two-phase approach:

1. **Turn to Face Target** — The robot rotates until it is oriented toward the goal location.  
2. **Navigate to Location** — The robot drives forward while continuously adjusting heading using a PD control loop to maintain stability and accuracy.

---

## 💻 Getting Started

### Prerequisites
- ROS 2 (**Humble** or later recommended)  
- Python 3  
- `ros2_control` and associated packages (if using standard ROS 2 controllers)

### Installation
Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone [your-repo-link]
