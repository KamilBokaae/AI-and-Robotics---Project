# Project Overview

This project is divided into three parts: **Part A**, **Part B**, and **Part C**.  
Each part includes Python scripts and corresponding videos demonstrating robotic tasks—either in simulation or real-world settings.

For each part:

- Scripts are located in: `partA/`, `partB/`, or `partC/`.
- Related videos are stored under: `videos/partX/`, where `X` is A, B, or C.

---

## Part A – Simulated Stacking and Transfer

**Script:** `partA/partA.py`

### Description
This script runs two simulated robotic tasks:
1. **Stacking** – The robot picks and stacks blocks at a specified target location.
2. **Transfer with Touch** – One robot transfers a block while another approaches and lightly touches it.

### Videos (Simulation Recordings)  
Located in `videos/partA/`:
- `stackPartA.mp4` – Demonstrates the stacking task.
- `transferPartA.mp4` – Demonstrates the transfer task.

### Note
To run a specific task:
- Comment out the unwanted function in `partA.py`.

**Functions:**
- `stack()`
- `transferCubeBetweenRobots()`

---

## Part B – Real-World Robotic Tasks

**Script:** `partB/partB.py`

### Description  
This part includes two robotic behaviors implemented using UR5 arms:
1. **Stacking Task** – A robot stacks blocks at a target location.
2. **Transfer Task** – One robot holds a block; the second robot approaches and gently touches it with its end-effector.

### Videos (Real Lab Recordings)  
Located in `videos/partB/`:
- `stackLab.mp4` – UR5 robot performing the stacking operation.
- `transferLab.mp4` – Cooperative touch interaction task.

---

## Part C – Heuristic-Based Block Assignment (Simulation)

**Script:** `partC/partC.py`

### Description  
This simulation involves a robot arranging blocks to form the word **"AI"**, using different heuristic strategies for assignment.

### Videos (Simulation Recordings)  
Located in `videos/partC/`:
- `greedy.mp4` – Greedy Nearest heuristic.
- `hungarian.mp4` – Hungarian algorithm for optimal matching.
- `euclidean.mp4` – Assignment based on Euclidean distance.
- `random.mp4` – Random assignment strategy.

---

## Important Note

To run **Part C** correctly, you **must replace** the existing `scene.xml` file at the following path:

```
sim_ur5/mujoco_env/assets/scenes/clairlab/scene.xml
```

---

## Project Report

For a detailed explanation of the project's objectives, methods, and results, please refer to the report file:

**File:** `report.pdf`
---

## Contributors

- **Kamil Bokaae**
- **Rula Younis**
