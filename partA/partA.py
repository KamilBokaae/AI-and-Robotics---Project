from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
from sim_ur5.mujoco_env.common.ur5e_fk import forward
from sim_ur5.motion_planning.motion_executor import compose_transformation_matrix
import numpy as np


def Stack(block_position, target_location):    

    # Create the simulation environment and the executor
    env = SimEnv()
    executor = MotionExecutor(env)

    # Add blocks to the world by enabling the randomize argument and setting the block position in the reset function of the SimEnv class 
    env.reset(randomize=False, block_positions=block_position)

    
    # Validate target location
    if not (workspace_x_lims[0] <= target_location[0] <= workspace_x_lims[1] and 
            workspace_y_lims[0] <= target_location[1] <= workspace_y_lims[1]):
        raise ValueError(f"Target location {target_location} is outside the workspace limits!")

    for i, position in enumerate(block_position):
        x, y, z = position

        # Validate cube position
        if not (workspace_x_lims[0] <= x <= workspace_x_lims[1] and workspace_y_lims[0] <= y <= workspace_y_lims[1]):
            raise ValueError(f"Cube position {position} is outside the workspace limits!")

        # Pick up the cube
        executor.pick_up("ur5e_2", x, y, z + 0.12)
        # Move and place the cube
        stack_z = 0.2 + 0.02 * i  # Compute stacking height

        executor.put_down("ur5e_2", target_location[0], target_location[1], stack_z)
        
def TransferCubeBetweenRobots(start_position, handover_position):
    # Create the simulation environment and the executor
    env = SimEnv()
    executor = MotionExecutor(env)

    # Add blocks to the world by enabling the randomize argument and setting the block position in the reset function of the SimEnv class 
    env.reset(randomize=False, block_positions=block_position)
    
    # Validate start position
    if not (workspace_x_lims[0] <= start_position[0] <= workspace_x_lims[1] and
            workspace_y_lims[0] <= start_position[1] <= workspace_y_lims[1]):
        raise ValueError(f"Start position {start_position} is outside the workspace limits!")

    # Validate handover position
    if not (workspace_x_lims[0] <= handover_position[0] <= workspace_x_lims[1] and
            workspace_y_lims[0] <= handover_position[1] <= workspace_y_lims[1]):
        raise ValueError(f"Handover position {handover_position} is outside the workspace limits!")

    x_start, y_start, z_start = start_position
    print(f"[DEBUG] ur5e_2 moving to pick_up position: {start_position}")
    executor.pick_up("ur5e_2", x_start, y_start, z_start+0.12)
    executor.wait(1)
    
    print(f"[DEBUG] ur5e_2 moving to handover position: {handover_position}")
    executor.plan_and_move_to_xyz_facing_down("ur5e_2", handover_position)
    executor.wait(1)


    print(f"[DEBUG] ur5e_2 changing the gripper direction")

    current_joints = executor.env.robots_joint_pos["ur5e_2"]
    new_joints = current_joints.copy()
    new_joints[4] += np.pi
    executor.moveJ("ur5e_2",new_joints)

    x_handover, y_handover, z_handover = handover_position

    print(f"[DEBUG] Robot 1 moving to handover position: {handover_position}")
    executor.plan_and_move_to_xyz_facing_down("ur5e_1", [x_handover, y_handover, z_handover+0.25])
    executor.wait(5)
    current_joints_for_robot1 = executor.env.robots_joint_pos["ur5e_1"]

    print("Robots are now facing each other.")


    
# TODO: check the limits  

workspace_x_lims = [-1.0, -0.45]
workspace_y_lims = [-1.0, -0.45]



block_position = [
    [-0.7, -0.6, 0.03],
    [-0.7, -0.7, 0.03],
    [-0.7, -0.8, 0.03],
    [-0.7, -0.9, 0.03]]

# Locate an area where both robots can reach
# You can choose any location in the area to place the blocks



Stack(block_position, target_location=[-0.8, -0.5])
TransferCubeBetweenRobots(start_position=[-0.7, -0.9, 0.03], handover_position=[-0.5, -0.45, 0.15])

