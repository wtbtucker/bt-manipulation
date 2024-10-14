import py_trees
import numpy as np

class RotateToAngle(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, target_angle):
        super(RotateToAngle, self).__init__(name)
        self.blackboard = blackboard
        self.robot = self.blackboard.read('robot')
        self.timestep = int(self.robot.getBasicTimeStep())
        self.target_angle = target_angle  # Target orientation in radians
    
    def setup(self):
        # Get sensor handles (GPS and compass)
        self.gps = self.robot.getDevice('gps')
        self.compass = self.robot.getDevice('compass')

        # Enable sensors
        self.gps.enable(self.timestep)
        self.compass.enable(self.timestep)

        self.MAX_SPEED = 6.28  # Max motor speed
        self.kp = 1.0  # Proportional gain for orientation control
    
    def initialise(self) -> None:
        # Get motor handles and put in velocity mode
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def update(self):
        # Get current orientation from compass
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])

        # Normalize theta
        if theta > np.pi:
            theta -= 2 * np.pi
        elif theta < -np.pi:
            theta += 2 * np.pi

        # Calculate the orientation error
        error = self.target_angle - theta

        # Normalize the error
        if error > np.pi:
            error -= 2 * np.pi
        elif error < -np.pi:
            error += 2 * np.pi

        # Proportional control for wheel velocities to correct orientation
        angular_velocity = self.kp * error
        angular_velocity = max(min(angular_velocity, self.MAX_SPEED), -self.MAX_SPEED)

        # Set the wheel speeds (rotate in place)
        self.left_motor.setVelocity(-angular_velocity)
        self.right_motor.setVelocity(angular_velocity)

        # Stop when error is sufficiently small
        if abs(error) < 0.01:  # You can adjust this threshold
            print('Reached target orientation')
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)