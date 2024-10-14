from controller import Supervisor
import py_trees
from py_trees.composites import Selector, Sequence
from AdjustArm import AdjustArm
from navigation import Navigation
from CloseGripper import CloseGripper
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
# safe configuration for nagivating the table
# calculate position and orientation of robot necessary to grab jam jar with arm in forward position
# How to reliably line up the robot with the jar?
# using computer vision?

# Do I need to create a configuration map of the area?
# Really only navigating between the counter and table
#2-layer configuration map
# layer for arm up in safe position vs arm out straight

# Identify position an orientation of robot where it can grab jam jar
# Plan route/navigate to pose with arm up in safe mode
# Drop arm down so it grabs jam jar
# close grippers and lift arm back into safe mode

# Planning is way too complicated at the moment
# Can drop configuration map for the moment, just get it to work before adding mapping
# drop the shortest path


jam_jar_pose = (1.71, -0.302, 0.889)

# position and orientation of robot to reach first jam jar
first_jar_position = [0.57, -0.28, 0]

forward_values = {
            'torso_lift_joint' : 0.30,
            'arm_1_joint' : 1.6,
            'arm_2_joint' : 0,
            'arm_3_joint' : 0,
            'arm_4_joint' : 0,
            'arm_5_joint' : 0,
            'arm_6_joint' : 0,
            'arm_7_joint' : 1.6,
            'gripper_left_finger_joint': 0.045,
            'gripper_right_finger_joint': 0.045
        }

raised_values = {
            'torso_lift_joint' : 0.30,
            'arm_1_joint' : 0.71,
            'arm_2_joint' : 1.02,
            'arm_3_joint' : -2.815,
            'arm_4_joint' : 1.011,
            'arm_5_joint' : 0,
            'arm_6_joint' : 0,
            'arm_7_joint' : 0,
        }


tree = Sequence("Main Sequence: ", children= [
    # AdjustArm("Raise arm", blackboard, raised_values),
    Navigation("Move to first jar", blackboard, first_jar_position[:2]),
    RotateToAngle("Orient towards first jar", blackboard, first_jar_position[2]),
    AdjustArm("Move arm forwards", blackboard, forward_values),
    CloseGripper("Grab first jar", blackboard),
    
], memory=True)
tree.setup_with_descendants()

while robot.step(timestep) != -1:
    tree.tick_once()