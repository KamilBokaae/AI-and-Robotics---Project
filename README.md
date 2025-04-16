# Project Homework

This project is divided into three parts: Part A, Part B, and Part C. Each part contains Python scripts (and videos) that demonstrate different functionalities related to robotic manipulation and simulation.

## Part A
- File: `partA/partA.py`

- Description: This script performs basic tasks as part of the initial assignment.

-Videos:
  Simulation videos are included in this folder to demonstrate the behavior implemented in `partA.py`.

## Part B
- File: `partB/stack.py`

- Description:  
  This part includes two implemented real-world robotic behaviors using UR5 arms:
  1. Stacking Task:  
     A robot stacks blocks at a target position.
  2. Touch Interaction Task:  
     One robot picks and holds a block at a designated point, while the second robot approaches and gently touches the block with its end-effector without grasping or taking it.

- Videos (Real Lab Recordings):
  - `stackLab.mp4` – UR5 robot performing the stacking operation.
  - `transferLab.mp4` – UR5 robots performing the cooperative touch interaction task.


## Part C
- File: `partC/partC.py`
- Description:
  This script runs in a simulated environment, where one robot places a set of blocks to form the word "AI" as a symbolic layout task.

- Video:  
  A simulation video is provided demonstrating this block-arranging behavior.

- **Important Note**: 
To run Part C correctly, you must replace the existing `scene.xml` file located at: sim_ur5/mujoco_env/assets/scenes/clairlab/scene.xml
