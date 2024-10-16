import py_trees
import numpy as np

class Navigate(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Navigate, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())

        # initialize and enable sensors
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)

        # self.marker = self.robot.getFromDef('marker').getField('translation')

        self.logger.debug(" %s [Nagivation::setup()]" % self.name)
    
    def initialise(self):
        # Initialize and set motors to velocity mode
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.logger.debug(" %s [Nagivation::update()]" % self.name)

        # set up to iterate through waypoints
        self.index = 0
        self.WP = self.blackboard.read('WP')
    
    def update(self):
        self.logger.debug(" %s [Navigation::update()]" % self.name) 

        # Get current position
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])

        # normalize theta
        if theta > np.pi:
            theta -= 2*np.pi
        elif theta < -np.pi:
            theta += 2*np.pi

        # Calculate error
        delta_x = self.WP[self.index][0] - xw
        delta_y = self.WP[self.index][1] - yw

        rho = np.sqrt(delta_x**2 + delta_y**2)
        alpha = np.arctan2(delta_y, delta_x) - theta

        # normalize alpha
        if alpha > np.pi:
            alpha -= 2*np.pi
        elif alpha < -np.pi:
            alpha += 2*np.pi
        
        # Compute actuator values
        p1 = 4
        p2 = 2
        vL = -p1*alpha + p2*rho
        vR = p1*alpha + p2*rho

        # Normalize speed
        MAX_SPEED = 6.28
        vL = max(min(vL, MAX_SPEED), -MAX_SPEED)
        vR = max(min(vR, MAX_SPEED), -MAX_SPEED)

        # Set actuator values
        self.left_motor.setVelocity(vL)
        self.right_motor.setVelocity(vR)

        if self.index == (len(self.WP) - 1):
            if rho < 0.01:
                print(f"Reached target coordinates")
                self.feedback_message = "Last waypoint reached"
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        elif rho < 0.4:
            self.index += 1
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING
        
    
    def terminate(self, new_status):
        self.logger.debug(" %s [Foo::terminate().terminate()]" %self.name)
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

class DriveForward(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, distance=0.15):
        super(DriveForward, self).__init__(name)
        self.blackboard = blackboard
        self.robot = self.blackboard.read('robot')
        self.distance = distance

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

    def initialise(self):
        self.start_position = self.gps.getValues()[:2]
        # Initialize motors and put in velocity mode
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
    
    def update(self):
        xw, yw = self.gps.getValues()[:2]
        dx = xw - self.start_position[0]
        dy = yw - self.start_position[1]
        distance_travelled = np.sqrt(dx**2 + dy**2)

        # Continue moving forward until target distance is reached
        if distance_travelled >= self.distance:
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            return py_trees.common.Status.SUCCESS
        else:
            self.left_motor.setVelocity(4.0)
            self.right_motor.setVelocity(4.0)
            return py_trees.common.Status.RUNNING
        
class Reverse(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, distance=0.15):
        super(Reverse, self).__init__(name)
        self.blackboard = blackboard
        self.robot = self.blackboard.read('robot')
        self.distance = distance

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

    def initialise(self):
        self.start_position = self.gps.getValues()[:2]
        # Initialize motors and put in velocity mode
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
    
    def update(self):
        xw, yw = self.gps.getValues()[:2]
        dx = xw - self.start_position[0]
        dy = yw - self.start_position[1]
        distance_travelled = np.sqrt(dx**2 + dy**2)

        # Continue moving forward until target distance is reached
        if distance_travelled >= self.distance:
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            return py_trees.common.Status.SUCCESS
        else:
            self.left_motor.setVelocity(-4.0)
            self.right_motor.setVelocity(-4.0)
            return py_trees.common.Status.RUNNING