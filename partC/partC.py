from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
from sim_ur5.mujoco_env.common.ur5e_fk import forward

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


        #A
        [-1.12, -0.88, 0.045],
        #[-1.04, -0.88, 0.045],
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
        [-0.8, -0.50, 0.025],
        [-0.8, -0.60, 0.025],
        [-0.8, -0.70, 0.025],
        [-0.8, -0.80, 0.025],
        [-0.8, -0.90, 0.025],

        [-0.7, -0.50, 0.025],
        [-0.7, -0.60, 0.025],
        [-0.7, -0.70, 0.025],
        [-0.7, -0.80, 0.025],
        [-0.7, -0.90, 0.025],

        [-0.6, -0.50, 0.025],
        [-0.6, -0.60, 0.025],
        [-0.6, -0.70, 0.025],
        [-0.6, -0.80, 0.025],
        [-0.6, -0.90, 0.025],

        [-0.5, -0.50, 0.025],
        [-0.5, -0.60, 0.025],
        [-0.5, -0.70, 0.025],
        [-0.5, -0.80, 0.025],
        [-0.5, -0.90, 0.025],
        [-1.04, -0.88, 0.025]
    ]
    
    env = SimEnv()
    executor = MotionExecutor(env)

    # Add blocks to the world by enabling the randomize argument and setting the block position in the reset function of the SimEnv class 
    env.reset(randomize=False, block_positions=start_positions)

    
    for i in range(20):
        executor.pick_up("ur5e_2", start_positions[i][0], start_positions[i][1], start_positions[i][2] + 0.13)
        executor.put_down("ur5e_2", target_positions[i][0], target_positions[i][1], target_positions[i][2] + 0.06)

    executor.wait(4)


MakeAIWord()