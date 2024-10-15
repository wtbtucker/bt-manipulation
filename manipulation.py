from controller import Supervisor
import numpy as np
import py_trees
from py_trees.composites import Sequence, Parallel
from AdjustArm import AdjustArm
from navigation import Navigate, DriveForward, Reverse
from Gripper import Gripper
from RotateToAngle import RotateToAngle
from AdjustTorsoHeight import AdjustTorsoHeight
from planning import Planning

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

class Blackboard:
    def __init__(self):
        self.blackboard = {}

    def write(self, key, value):
        self.blackboard[key] = value

    def read(self, key):
        return self.blackboard[key]
    
blackboard = Blackboard()
blackboard.write('robot', robot)
blackboard.write('WP', [])

lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

forward_values = {
    'torso_lift_joint' : 0.065,
    'arm_1_joint' : 1.6,
    'arm_2_joint' : np.pi/4,
    'arm_3_joint' :0,
    'arm_4_joint' : np.pi/4,
    'arm_5_joint' : np.pi/2,
    'arm_6_joint' : 0,
    'arm_7_joint' : 0,
}

tucked_values = {
    'arm_1_joint' : 1.6,
    'arm_2_joint' : 1.4,
    'arm_3_joint' : 0,
    'arm_4_joint' : 2.29,
    'arm_5_joint' : np.pi/2,
    'arm_6_joint' : 1.8,
    'arm_7_joint' : 0
}

WP = [(0.55, -0.3, 0), (0.6, -1.75, np.pi), (0.65, -0.045, 0.45), (0.65, -1.75, np.pi), (0.7, 0.1, 0.13), (0.75, -1.8, np.pi)]

tree = Sequence("Main Sequence: ", children= [
    # First jar
    Gripper("Open", blackboard, action='open'),
    AdjustArm("Lower arm", blackboard, forward_values),
    AdjustTorsoHeight("Raise to counter", blackboard, 0.15),
    Planning("Route to first jar", blackboard, WP[0][:2]),
    Navigate("Move to first jar", blackboard),
    RotateToAngle("Orient towards first jar", blackboard, WP[0][2]),
    AdjustArm("Move arm forwards", blackboard, forward_values),
    DriveForward("Approach first Jar", blackboard, distance=0.1),
    Gripper("Grab jar", blackboard, 'close'),
    Parallel("Bring Jar 1 to table", py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True), children=[
        Gripper("Grab jar 1", blackboard, action='close'),
        Sequence("Lift jar 1 and carry to table", children=[
            AdjustTorsoHeight("Raise robot 1", blackboard, 0.15),
            AdjustArm("Tuck arm 1", blackboard, tucked_values),
            Reverse("Back away from counter 1", blackboard, distance=0.1),
            Planning("Route to table 1", blackboard, WP[1][:2]),
            Navigate("Drive to table 1", blackboard),
            RotateToAngle("Orient towards table 1", blackboard, WP[1][2]),
            AdjustTorsoHeight("Lower robot 1", blackboard, 0.05),
            AdjustArm("Lower Arm", blackboard, forward_values)
        ], memory=True)
    ]),
    Gripper("Place jar", blackboard, action='open'),

    # Second jar
    AdjustArm("Tuck arm", blackboard, tucked_values),
    Planning("Route to second jar", blackboard, WP[2][:2]),
    Navigate("Move to second jar", blackboard),
    RotateToAngle("Orient towards second jar", blackboard, WP[2][2]),
    AdjustTorsoHeight("Lift to clear counter", blackboard, 0.2),
    AdjustArm("Move arm forwards", blackboard, forward_values),
    DriveForward("Approach second Jar", blackboard, distance=0.05),
    Gripper("Grab jar", blackboard, 'close'),
    Parallel("Bring Jar 2 to table", py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True), children=[
        Gripper("Grab jar 2", blackboard, action='close'),
        Sequence("Lift jar 2 and carry to table", children=[
            AdjustTorsoHeight("Raise robot 2", blackboard, 0.15),
            AdjustArm("Tuck arm 2", blackboard, tucked_values),
            Reverse("Back away from counter 2", blackboard, distance=0.1),
            Planning("Route to table 2", blackboard, WP[3][:2]),
            Navigate("Move to table 2", blackboard),
            RotateToAngle("Orient towards table 2", blackboard, WP[3][2]),
            AdjustArm("Lower Arm", blackboard, forward_values)
        ], memory=True)
    ]),
    Gripper("Place jar", blackboard, action='open'),
    Reverse("Back away from table 2", blackboard, distance=0.05),

    # Third jar
    AdjustArm("Forward arm", blackboard, tucked_values),
    AdjustTorsoHeight("Raise robot 3", blackboard, 0.35),
    Planning("Route to third jar", blackboard, WP[4][:2]),
    Navigate("Move to third jar", blackboard),
    RotateToAngle("Orient towards third jar", blackboard, WP[4][2]),
    AdjustArm("Move arm forwards", blackboard, forward_values),
    DriveForward("Approach third Jar", blackboard, distance=0.1),
    Gripper("Grab jar", blackboard, 'close'),
    Parallel("Bring Jar 3 to table", py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True), children=[
        Gripper("Grab jar 3", blackboard, action='close'),
        Sequence("Lift jar 3 and carry to table", children=[
            AdjustTorsoHeight("Raise robot 3", blackboard, 0.2),
            AdjustArm("Tuck arm 3", blackboard, tucked_values),
            Reverse("Back away from counter 3", blackboard, distance=0.1),
            Planning("Route to table 3", blackboard, WP[5][:2]),
            Navigate("Move to table 3", blackboard),
            RotateToAngle("Orient towards table 3", blackboard, WP[5][2]),
            AdjustArm("Lower Arm", blackboard, forward_values)
        ], memory=True)
    ]),
    Gripper("Place jar", blackboard, action='open'),

], memory=True)
tree.setup_with_descendants()

while robot.step(timestep) != -1:
    tree.tick_once()