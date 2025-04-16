from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
from sim_ur5.mujoco_env.common.ur5e_fk import forward
import numpy as np
import time


def euclidean(p1, p2):
    return np.linalg.norm(np.array(p1[:2]) - np.array(p2[:2]))

def MakeAIWord():
    target_positions = [
        # I
        [-1.12, -0.60, 0.045],
        [-1.12, -0.68, 0.045],
        [-1.12, -0.76, 0.045],
        [-1.04, -0.68, 0.045],
        [-0.96, -0.68, 0.045],
        [-0.88, -0.68, 0.045],
        [-0.80, -0.60, 0.045],
        [-0.80, -0.682, 0.045],
        [-0.80, -0.764, 0.045],

        # A
        [-1.12, -0.88, 0.045],
        [-0.96, -0.88, 0.045],
        [-0.88, -0.88, 0.045],
        [-0.80, -0.88, 0.045],
        [-1.12, -0.96, 0.045],
        [-1.12, -1.04, 0.045],
        [-1.04, -1.04, 0.045],
        [-0.96, -1.04, 0.045],
        [-0.88, -1.04, 0.045],
        [-0.80, -1.04, 0.045],
        [-0.96, -0.96, 0.045]
    ]

    start_positions = [
        [-0.6, -0.50, 0.05],
        [-0.6, -0.60, 0.05],
        [-0.6, -0.70, 0.05],
        [-0.6, -0.80, 0.05],
        [-0.6, -0.90, 0.05],
        [-0.5, -0.50, 0.05],
        [-0.5, -0.60, 0.05],
        [-0.5, -0.70, 0.05],
        [-0.5, -0.80, 0.05],
        [-0.5, -0.90, 0.05],
        [-0.6, -0.50, 0.025],
        [-0.6, -0.60, 0.025],
        [-0.6, -0.70, 0.025],
        [-0.6, -0.80, 0.025],
        [-0.6, -0.90, 0.025],
        [-0.5, -0.50, 0.025],
        [-0.5, -0.60, 0.025],
        [-0.5, -0.70, 0.025],
        [-0.5, -0.80, 0.025],
        [-0.5, -0.90, 0.025]
    ]

    used_targets = [False] * len(target_positions)

    env = SimEnv()
    executor = MotionExecutor(env)
    env.reset(randomize=False, block_positions=start_positions)

    current_pos = [-0.5, -0.5]  # arbitrary initial robot position

    for i, start in enumerate(start_positions):
        # pick from start
        if(i<10):
            pick_z = 0.2
        else:
            pick_z = 0.16
        executor.pick_up("ur5e_2", start[0], start[1], pick_z)

        # choose best target greedily
        best_idx = -1
        min_dist = float("inf")
        for j, target in enumerate(target_positions):
            if not used_targets[j]:
                dist = euclidean(start, target)
                if dist < min_dist:
                    min_dist = dist
                    best_idx = j

        if best_idx == -1:
            print("No unused target found!")
            break

        target = target_positions[best_idx]
        put_z = target[2] + 0.06
        executor.put_down("ur5e_2", target[0], target[1], put_z)
        used_targets[best_idx] = True
        current_pos = target[:2]

    executor.wait(4)




start_time = time.time()
MakeAIWord()
end_time = time.time()
print(end_time-start_time)


