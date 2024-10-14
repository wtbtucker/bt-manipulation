import py_trees

class Gripper(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, action):
        super(Gripper, self).__init__(name)
        self.blackboard = blackboard
        self.robot = self.blackboard.read('robot')
        self.timestep = int(self.robot.getBasicTimeStep())
        self.threshold = -10
        self.action = action
        if action == 'close':
            self.target = 0.0
        elif action == 'open':
            self.target = 0.045
        else:
            print('Error with selected')
    
    def setup(self):
        # Initialize joint handles and enable force feedback
        self.left_gripper = self.robot.getDevice('gripper_left_finger_joint')
        self.right_gripper = self.robot.getDevice('gripper_right_finger_joint')
        self.left_gripper.enableForceFeedback(self.timestep)
        self.right_gripper.enableForceFeedback(self.timestep)

        # Initialize and enable force sensors
        self.sensor_left = self.robot.getDevice('gripper_left_sensor_finger_joint')
        self.sensor_left.enable(self.timestep)
        self.sensor_right = self.robot.getDevice('gripper_right_sensor_finger_joint')
        self.sensor_right.enable(self.timestep)
    
    def initialise(self):
        print(f'Setting gripper to: {self.target}')
        self.left_gripper.setPosition(self.target)
        self.right_gripper.setPosition(self.target)

    def update(self):
        if self.target == 0.0:
            # Check force sensor value 
            force_value = self.left_gripper.getForceFeedback()
            print(force_value)
            if force_value < self.threshold:
                print('Grasped object')
                return py_trees.common.Status.SUCCESS  # Gripper successfully closed on the object
            else:
                return py_trees.common.Status.RUNNING  # Keep closing until successful
        else:
            # Check sensor for gripper position
            left_position = self.sensor_left.getValue()
            right_position = self.sensor_right.getValue()
            if abs(left_position - self.target) < 0.001 and abs(right_position - self.target) < 0.001:
                print('Gripper opened')
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            if self.target == 0.0:
                # Keep gripper closed
                self.left_gripper.setPosition(self.target)
                self.right_gripper.setPosition(self.target)
            else:
                # Keep gripper open
                self.left_gripper.setPosition(self.target)
                self.right_gripper.setPosition(self.target)