import py_trees
import numpy as np
from scipy import signal
from matplotlib import pyplot as plt

def world2map(xw, yw):
    px = int(xw * 40 + 90)
    py = int(yw * -50 + 100)

    px = max(min(px, 199), 0)
    py = max(min(py, 299), 0)

    return (px, py)

def map2world(px, py):
    xw = px/40 - 2.25
    yw = py/(-50) + 2

    return (xw, yw)

def probability_to_hex(prob):
    # Ensure the probabilities are within [0, 1]
    prob = max(min(prob, 1), 0)
    
    grayscale_value = int(prob * 255)
    
    # Combine the grayscale values into 24-bit RGB (grayscale)
    color_24bit = (grayscale_value << 16) | (grayscale_value << 8) | grayscale_value
    
    return color_24bit

class Mapping(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Mapping, self).__init__(name)

        self.has_run = False
        self.robot = blackboard.read('robot')

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # initialize and enable sensors
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        self.display = self.robot.getDevice('display')

        self.logger.debug(" %s [Mapping::setup()]" % self.name)
    
    def initialise(self):
        self.logger.debug(" %s [Map::initialize()]" % self.name)
        self.map = np.zeros((200,300))

        # do not include the first or last 80 lidar readings which are blocked by the case
        self.angles = np.linspace(4.19/2, -4.19/2, 667)
        self.angles = self.angles[80:len(self.angles)-80]

    def update(self):
        self.has_run = True

        # get sensor values
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])

        # plot robot trajectory
        px, py = world2map(xw, yw)
        self.display.setColor(0xFF0000)
        self.display.drawPixel(px, py)

        w_T_r = np.array([[np.cos(theta), -np.sin(theta), xw],
                     [np.sin(theta), np.cos(theta), yw],
                     [0, 0, 1]])
        
        # Read range images
        ranges = np.array(self.lidar.getRangeImage())
        ranges = ranges[80:len(ranges)-80]
        ranges[ranges == np.inf] = 100

        X_i = np.array([(ranges*np.cos(self.angles) + 0.202),
                        ranges*np.sin(self.angles),
                        np.ones(len(self.angles))])
        
        # transform range image into world coordinates
        D = w_T_r @ X_i 

        for d in D.transpose():
            px, py = world2map(d[0], d[1])
            self.map[px, py] += 0.1
            color = probability_to_hex(self.map[px, py])
            self.display.setColor(color)
            self.display.drawPixel(px, py)

        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        self.logger.debug(" %s [Foo::terminate().terminate()]" %self.name)

        if self.has_run:
            cmap = signal.convolve2d(self.map, np.ones((26, 26)), mode='same')
            cspace = cmap>0.9

            plt.figure(1)
            plt.imshow(cspace)
            plt.show()
            np.save('cspace', cspace)