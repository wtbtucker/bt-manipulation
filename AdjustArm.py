import py_trees
from py_trees.common import Status

def get_encoder_name(joint_name):
    if joint_name == 'gripper_left_finger_joint':
        return 'gripper_left_sensor_finger_joint'
    elif joint_name == 'gripper_right_finger_joint':
        return 'gripper_right_sensor_finger_joint'
    else:
        return joint_name + '_sensor'
    
class AdjustArm(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, goal):
        super(AdjustArm, self).__init__(name)
        self.blackboard = blackboard
        self.robot = self.blackboard.read('robot')
        self.timestep = int(self.robot.getBasicTimeStep())
        self.target_values = goal
        self.joints = self.target_values.keys()

        self.joint2max_speed = {
            'arm_1_joint': 1.95,
            'arm_2_joint': 1.95,
            'arm_3_joint': 2.35,
            'arm_4_joint': 2.35,
            'arm_5_joint': 1.95,
            'arm_6_joint': 1.76,
            'arm_7_joint': 1.76,
            'gripper_left_finger_joint': 0.05,
            'gripper_right_finger_joint': 0.05,
            'torso_lift_joint': 0.07
        }
        self.kp = 0.75
        
        self.key2handle = {}
        self.key2sensor = {}

    def initialise(self) -> None:
        # Initialize joint motors and set to velocity mode
        for joint in self.joints:
            self.key2handle[joint] = self.robot.getDevice(joint)
            self.key2handle[joint].setPosition(float('inf'))
            self.key2handle[joint].setVelocity(0.0)
        
        # Initialize and enable joint sensors
        for joint_name in self.joints:
            sensor_name = get_encoder_name(joint_name)
            self.key2sensor[joint_name] = self.robot.getDevice(sensor_name)
            self.key2sensor[joint_name].enable(self.timestep)
    

    # keep gripped in positional mode, error is so small force is decreasing
    def update(self):
        total_error = 0.0
        for joint, target_value in self.target_values.items():
            current_value = self.key2sensor[joint].getValue()
            error = target_value - current_value

            # calculate and normalize joint speed
            joint_speed = self.kp * error
            max_speed = self.joint2max_speed[joint]
            joint_speed = max(min(joint_speed, max_speed), -max_speed)
            self.key2handle[joint].setVelocity(joint_speed)

            # square error to avoid negatives cancelling out
            total_error += error**2

        if total_error < 0.001:
            print(f'{self.name}: success')
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status) -> None:
        # Stop the joint motors
        for handle in self.key2handle.values():
            handle.setVelocity(0.0)

        

