from controller import Supervisor
import numpy as np
import py_trees
from py_trees.composites import Selector, Sequence, Parallel
from AdjustArm import AdjustArm
from navigation import Navigate, DriveForward, Reverse
from Gripper import Gripper
from RotateToAngle import RotateToAngle
from AdjustTorsoHeight import AdjustTorsoHeight

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

# TODO:
# using computer vision?

jam_jar_pose = (1.71, -0.302, 0.889)

WP = [(0.5, -0.3, 0), (0.5, -1.75, np.pi), (0.545, -0.115, 0.465), (0.65, -1.75, np.pi)]

forward_values = {
    'torso_lift_joint' : 0.075,
    'arm_1_joint' : 1.6,
    'arm_2_joint' : np.pi/4,
    'arm_3_joint' :0,
    'arm_4_joint' : np.pi/4,
    'arm_5_joint' : np.pi/2,
    'arm_6_joint' : 0,
    'arm_7_joint' : 0,
}

raised_values = {
    'torso_lift_joint' : 0.30,
    'arm_1_joint' : 0.71, 
    'arm_2_joint' : 1.02,
    'arm_3_joint' : -2.815,
    'arm_4_joint' : 1.011,
    'arm_5_joint' : 0,
    'arm_6_joint' : 0,  
    'arm_7_joint' : 0
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

# Commonly used patterns
def create_grab_jar():
    return Sequence("Grab jar", children=[
    AdjustArm("Move arm forwards", blackboard, forward_values),
    DriveForward("Approach first Jar", blackboard, distance=0.1),
    Gripper("Grab jar", blackboard, 'close'),
], memory=True)

def create_bring_jar_to(target_coordinates):
    return Parallel("Bring Jar to table", py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True), children=[
        Gripper("Grab jar", blackboard, action='close'),
        Sequence("Lift jar and carry to table", children=[
            AdjustTorsoHeight("Raise robot", blackboard, 0.15),
            AdjustArm("Tuck arm", blackboard, tucked_values),
            Reverse("Back away from counter", blackboard, distance=0.1),
            Navigate("Drive to table", blackboard, target_coordinates[:2]),
            RotateToAngle("Orient towards table", blackboard, target_coordinates[2]),
            AdjustArm("Lower Arm", blackboard, forward_values)
        ], memory=True) 
    ])

tree = Sequence("Main Sequence: ", children= [
    Gripper("Open", blackboard, action='open'),
    AdjustArm("Lower arm", blackboard, forward_values),
    Navigate("Move to first jar", blackboard, WP[0][:2]),
    RotateToAngle("Orient towards first jar", blackboard, WP[0][2]),
    create_grab_jar(),
    create_bring_jar_to(WP[1]),
    Gripper("Place jar", blackboard, action='open'),
    AdjustArm("Tuck arm", blackboard, tucked_values),
    Navigate("Move to second jar", blackboard, WP[2][:2]),
    RotateToAngle("Orient towards second jar", blackboard, WP[2][2]),
    AdjustArm("Move arm forwards", blackboard, forward_values),
    create_grab_jar(),
    create_bring_jar_to(WP[3]),
    Gripper("Place jar", blackboard, action='open'),

], memory=True)
tree.setup_with_descendants()

while robot.step(timestep) != -1:
    tree.tick_once()