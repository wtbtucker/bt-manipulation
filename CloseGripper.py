import py_trees

class CloseGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(CloseGripper, self).__init__(name)
        self.blackboard = blackboard
        self.robot = self.blackboard.read('robot')
        self.timestep = int(self.robot.getBasicTimeStep())
        self.threshold = -10
    
    def setup(self):
        # Initialize joint handles and enable force feedback
        self.left_gripper = self.robot.getDevice('gripper_left_finger_joint')
        self.right_gripper = self.robot.getDevice('gripper_right_finger_joint')
        self.left_gripper.enableForceFeedback(self.timestep)
        self.right_gripper.enableForceFeedback(self.timestep)

        # Initialize and enable force sensors
        self.force_sensor_left = self.robot.getDevice('gripper_left_sensor_finger_joint')
        self.force_sensor_left.enable(self.timestep)
        self.force_sensor_right = self.robot.getDevice('gripper_right_sensor_finger_joint')
        self.force_sensor_right.enable(self.timestep)
    
    def initialise(self):
        print('closing gripper')
        # Close the gripper
        self.left_gripper.setPosition(0.0)
        self.right_gripper.setPosition(0.0)

    def update(self):        
        # Check force sensor value
        force_value = self.left_gripper.getForceFeedback()
        print(force_value)
        if force_value < self.threshold:
            print('Grasped object')
            return py_trees.common.Status.SUCCESS  # Gripper successfully closed on the object
        else:
            return py_trees.common.Status.RUNNING  # Keep closing until successful