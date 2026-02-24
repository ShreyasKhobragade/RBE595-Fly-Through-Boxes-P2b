<div align="center">
  
# Fly Through Boxes! (P2b)
### Hands-On Aerial Robotics [RBE595]

[![Simulation](https://img.shields.io/badge/Simulation-VizFlyt-orange.svg)](#)
[![Control](https://img.shields.io/badge/Control-Cascaded%20PID-blue.svg)](#)
[![Framework](https://img.shields.io/badge/Language-Python-green.svg)](#)

*Implementation of full-stack 3D motion planning and robust quadrotor control integrated into the Turing cluster VizFlyt simulation.*

</div>

---

## 📖 Overview

This repository holds Project 2b (P2b) of the *Hands-On Aerial Robotics* course. Building directly on the kinematics and path-planning groundwork from Project 2a, this project applies the entire navigation stack to a high-fidelity **VizFlyt simulated quadrotor**. 

The goal is to generate optimal trajectories using an RRT* planner, smooth the paths using polynomial splines, and implement a cascaded PID controller to autonomously pilot the drone through a dense mapped environment populated with virtual obstacles—strictly enforcing zero-tolerance collision rules.

### Key Features
- **3D Spatial Navigation:** Autonomously traversing from a designated takeoff coordinate to a landing zone through clustered boxes.
- **VizFlyt Integration:** Transitioning the algorithms from Matplotlib abstractions to full physics-based Unity/VizFlyt renderings running on WPI's Turing compute cluster.
- **Robust Collision Avoidance:** Dynamically inflating known environment bounding boxes by a `safety_margin` and constantly monitoring the state against the `mapSplat.txt` boundary lines to entirely avoid collision states computationally.

---

## 🚀 Execution & Methodology

### 1. Planning Layer (RRT* & Splines)
RRT* generates a collision-free tree spanning from the start coordinate to the spatial goal, optimizing for the shortest path. This tree is then parameterized into segmented cubic/quintic splines to ensure C2 continuity—mandating that the drone can trace the trajectory without stopping or facing impossible instantaneous acceleration bounds.

### 2. Control Stack (Cascaded PID)
An intricate nested PID architecture translates the optimal spline trajectory into motor mixing signals for the quadrotor. The outer loop measures coordinate discrepancies against the target XYZ to derive a demand velocity. The inner loop matches the quadrotor's current simulated velocities with this demand, outputting precise pitch/roll angles and thrust targets.

### 3. VizFlyt Rendering
Leveraging the Turing cluster, the simulation streams the Python-based navigational decisions into visually compelling, real-time quadrotor movement inside an immersive digital twin.

---

## 🎥 Simulation Showcase

Here is the successful autonomous simulation flight executed by the quadrotor cleanly dodging the mapped obstacles:

<div align="center">
  <video src="assets/Video.mp4" width="100%" controls autoplay loop muted></video>
</div>

---

## 🚀 Quick Start & Usage

### Requirements
Ensure you are connected to the Turing cluster with `VizFlyt` correctly patched per the assignment instructions.
```bash
pip install numpy scipy matplotlib
```

### Usage
```bash
# Enter the source code directory
cd src/

# Launch the primary simulation node
python main.py
```

---

## 📁 Repository Structure

```text
RBE595-Fly-Through-Boxes-P2b/
├── src/                   # Python sources (Planners, Controls, VizFlyt integration)
├── Report.pdf             # Exhaustive formulation, math models, and analysis
├── install.md             # WPI Turing Cluster setup specifications for VizFlyt
├── assets/                # Rendered video clips of successful simulation flights
└── README.md              # Project overview
```
