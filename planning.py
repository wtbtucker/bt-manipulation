from heapq import heapify, heappush, heappop
import numpy as np
from collections import defaultdict
import py_trees

def world2map(xw, yw):
    px = int(xw * 40 + 90)
    py = int(yw * -50 + 100)

    px = max(min(px, 199), 0)
    py = max(min(py, 299), 0)

    return (px, py)

def map2world(point):
    px, py = point
    xw = px/40 - 2.25
    yw = py/(-50) + 2

    return (xw, yw)

# get all navigatable pixels from the current position
def get_neighbors(curr, map):
    neighors = []
    x, y  = curr
    for delta_x, delta_y in [(-1,0), (1,0), (0, -1), (0, 1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
        new_x, new_y = x+delta_x, y+delta_y
        if 0 <= new_x < len(map) and 0 <= new_y < len(map[0]):
            neighors.append(((new_x, new_y), np.sqrt(delta_x**2 + delta_y**2)))      
    return neighors

# A* shortest path algorithm
def get_shortest_path(map, start, goal):
    q = [(0, start)]
    heapify(q)

    distances = defaultdict(lambda: float('inf'))
    visited = set()
    parents = {}
    distances[start] = 0

    while q:
        curr_dist, curr = heappop(q)
        visited.add(curr)
        if curr == goal:
            q.clear()
            break
        for cand, cost_curr_cand in get_neighbors(curr, map):
            if cand not in visited:
                new_dist = curr_dist + cost_curr_cand
                if new_dist < distances[cand]:
                    distances[cand] = new_dist
                    parents[cand] = curr
                    # Use heuristic to direct path-finding towards goal
                    heuristic = new_dist + np.sqrt((goal[0]-cand[0])**2+(goal[1]-cand[1])**2)
                    heappush(q, (heuristic, cand))
    
    c = goal
    reverse_path = []
    while c in parents.keys():
        reverse_path.append(c)
        c = parents[c]
    return reversed(reverse_path)

        
class Planning(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, goal):
        super(Planning, self).__init__(name)
        
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard

        px, py = world2map(goal[0], goal[1])
        self.goal = (px, py)

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        # initialize and enable sensors
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        self.logger.debug(" %s [Planning::setup()]" % self.name)

    def initialise(self):
        self.logger.debug(" %s [Planning::initialise()]" % self.name)
        self.map = np.zeros((200, 300))
    
    def update(self):
        self.logger.debug(" %s [Planning::update()]" % self.name)
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        start = world2map(xw, yw)

        # compute shortest path using A* from start to goal
        path = get_shortest_path(self.map, start, self.goal)

        # convert map pixels to world coordinates
        world_path = [map2world(point) for point in path]

        # write shortest path to waypoints so navigation can access
        self.blackboard.write('WP', world_path)

        print(f"Route planned to {self.goal}")
        return py_trees.common.Status.SUCCESS