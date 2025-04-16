#import typer
import numpy as np

from ..motion_planning.motion_planner import MotionPlanner
from ..motion_planning.geometry_and_transforms import GeometryAndTransforms
from lab_ur5.manipulation.manipulation_controller import ManipulationController
from lab_ur5.robot_inteface.robots_metadata import ur5e_1, ur5e_2

workspace_x_lim = [-1.0, 1]
workspace_y_lim = [-1.0, 1]

block_positions = [
    [0.3, 0, 0.03],
    [0.4, 0, 0.03],
    [0.5, 0, 0.03],
    [0.6, 0, 0.03],
]

target_position = [0.7, 0, 0.03]

def Stack(block_positions, target_position):

    motion_planner = MotionPlanner()
    gt = GeometryAndTransforms.from_motion_planner(motion_planner)
    r1_controller = ManipulationController(ur5e_1["ip"],ur5e_1["name"],motion_planner,gt)
    r1_controller.speed = 2
    r1_controller.acceleration = 0.3
    r1_controller.move_home()

    if not(workspace_x_lim[0] <= target_position[0] <= workspace_x_lim[1] and
        workspace_y_lim[0] <= target_position[1] <= workspace_y_lim[1]):
        raise ValueError("target_position must be within workspace_x_lim")
    for i , position in enumerate(block_positions):
        x, y, z = position

        if not (workspace_x_lim[0] <= x <= workspace_x_lim[1] and workspace_y_lim[0] <= y <= workspace_y_lim[1]):
            raise ValueError("block_positions must be within workspace_x_lim")

        r1_controller.pick_up(x, y, 0)
        r1_controller.put_down(target_position[0], target_position[1], 0)

    r1_controller.move_home()

def TransferCubeBetweenRobots(start_position, handover_position):
    motion_planner = MotionPlanner()
    gt = GeometryAndTransforms.from_motion_planner(motion_planner)
    r1_controller = ManipulationController(ur5e_1["ip"],ur5e_1["name"],motion_planner,gt)
    r1_controller.speed = 2
    r1_controller.acceleration = 0.3

    r2_controller = ManipulationController(ur5e_2["ip"],ur5e_2["name"],motion_planner,gt)
    r2_controller.speed = 2
    r2_controller.acceleration = 0.3

    if not(workspace_x_lim[0] <= start_position[0] <= workspace_x_lim[1] and
        workspace_y_lim[0] <= start_position[1] <= workspace_y_lim[1]):
        raise ValueError("target_position must be within workspace_x_lim")

    if not(workspace_x_lim[0] <= handover_position[0] <= workspace_x_lim[1] and
        workspace_y_lim[0] <= handover_position[1] <= workspace_y_lim[1]):
        raise ValueError("target_position must be within workspace_x_lim")

    r1_controller.move_home()
    r2_controller.move_home()

    x_start, y_start, z_start = start_position
    r1_controller.pick_up(x_start, y_start, 0)
    r1_joints = [-1.5453937689410608, -3.05643429378652, 0.0006359259234827164, -4.813620945016378, 1.4802601337432861, 0.024399036541581154]
    r1_controller.moveJ(r1_joints)

    r2_joints =[0.5302243232727051, -0.7629957360080262, 0.0037229696856897476, -0.8698388499072571, -1.5010808149920862, 0.5871725678443909]
    r2_controller.moveJ(r2_joints)

    # r1_controller.plan_and_move_to_xyzrz(handover_position[0], handover_position[1], handover_position[2], 0)
    #
    # current_joints = r1_controller.getActualQ()
    # new_joints = current_joints.copy()
    # new_joints[4] += np.pi
    # r1_controller.moveJ(new_joints)
    tcp1_xyz = r1_controller.getActualTCPPose()
    tcp1_joints = r1_controller.getActualQ()

    print("r1 xyz")
    print(tcp1_xyz)
    print("r1 joints")
    print(tcp1_joints)


    tcp2_xyz = r2_controller.getActualTCPPose()
    tcp2_joints = r2_controller.getActualQ()

    print("r2 xyz:")
    print(tcp2_xyz)
    print("r2 joints:")
    print(tcp2_joints)
    #r2_controller.plan_and_move_to_xyzrz(handover_position[0], handover_position[1], handover_position[2], 0)
#Stack(block_positions, target_position)
TransferCubeBetweenRobots([0.3, 0, 0.03], [-0.5, -0.45, 0.15])
