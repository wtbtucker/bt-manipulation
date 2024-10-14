from controller import Supervisor
import numpy as np
import py_trees
from py_trees.composites import Selector, Sequence, Parallel
from AdjustArm import AdjustArm
from navigation import Navigation
from Gripper import Gripper
from RotateToAngle import RotateToAngle

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

# Identify position an orientation of robot where it can grab jam jar
# Plan route/navigate to pose with arm up in safe mode
# Drop arm down so it grabs jam jar
    # Not terminating
    # Something wrong with target values?
    # gripper is stopped from moving by external force (jam jar)
# close grippers and lift arm back into safe mode


# safe configuration for nagivating the table
# calculate position and orientation of robot necessary to grab jam jar with arm in forward position
# How to reliably line up the robot with the jar?
# using computer vision?

# Do I need to create a configuration map of the area?
# Really only navigating between the counter and table
#2-layer configuration map
# layer for arm up in safe position vs arm out straight





jam_jar_pose = (1.71, -0.302, 0.889)

# position and orientation of robot to reach first jam jar
first_jar_position = [0.4, -0.3, 0]
first_jar_grabbing_position = [0.6, -0.3, 0]

forward_values = {
    'torso_lift_joint' : 0.35,
    'arm_1_joint' : 1.6,
    'arm_2_joint' : 0,
    'arm_3_joint' : 0,
    'arm_4_joint' : 0,
    'arm_5_joint' : 0,
    'arm_6_joint' : 0,
    'arm_7_joint' : 1.6,
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
    'torso_lift_joint' : 0.40,
    'arm_1_joint' : 0.07,
    'arm_2_joint' : 0,
    'arm_3_joint' : -np.pi/2,
    'arm_4_joint' : 2.29,
    'arm_5_joint' : 0,
    'arm_6_joint' : 0,  
    'arm_7_joint' : 0
}


tree = Sequence("Main Sequence: ", children= [
    Gripper("Open", blackboard, 'open'),
    AdjustArm("Lower arm", blackboard, forward_values),
    Navigation("Move to first jar", blackboard, first_jar_position[:2]),
    RotateToAngle("Orient towards first jar", blackboard, first_jar_position[2]),
    AdjustArm("Move arm forwards", blackboard, forward_values),
    Navigation("Move forwards", blackboard, first_jar_grabbing_position[:2]),
    Gripper("Grab jar", blackboard, 'close'),
    Parallel("Grip and Raise", py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True), children=[
        Gripper("Grab jar", blackboard, action='close'),
        AdjustArm("Tuck arm", blackboard, tucked_values)
    ])
], memory=True)
tree.setup_with_descendants()

while robot.step(timestep) != -1:
    tree.tick_once()