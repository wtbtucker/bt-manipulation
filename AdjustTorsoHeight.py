import py_trees

class AdjustTorsoHeight(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, target_height):
        super(AdjustTorsoHeight, self).__init__(name)
        self.blackboard = blackboard
        self.robot = self.blackboard.read('robot')
        self.timestep = int(self.robot.getBasicTimeStep())
        self.target_height = target_height

    def setup(self):
        # proportional constant
        self.kp = 2

        # Initialize torso lift joint motor and set to velocity mode
        self.torso_lift_joint = self.robot.getDevice('torso_lift_joint')
        self.torso_lift_joint.setPosition(float('inf'))
        self.torso_lift_joint.setVelocity(0.0)

        # Initialize and enable torso lift sensor
        self.torso_sensor = self.robot.getDevice('torso_lift_joint_sensor')
        self.torso_sensor.enable(self.timestep)

        # Define maximum speed for the torso lift joint
        self.max_speed = 0.07

    def update(self):
        # Get current torso height
        current_height = self.torso_sensor.getValue()
        
        # Calculate error
        error = self.target_height - current_height

        # Calculate speed using proportional control
        joint_speed = self.kp * error
        joint_speed = max(min(joint_speed, self.max_speed), -self.max_speed)

        # Apply calculated speed
        self.torso_lift_joint.setVelocity(joint_speed)

        # If the error is small enough, consider it a success
        if abs(error) < 0.01:  # Adjust tolerance as needed
            print(f'{self.name}: success, reached target height')
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status) -> None:
        # Stop the torso lift motor
        self.torso_lift_joint.setVelocity(0.0)
