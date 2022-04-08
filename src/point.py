from collections import namedtuple
from math import sqrt

def class Point(namedtuple):

    def __init__(self, pose):
        self.x = pose[0]
        self.y = pose[1]

    
    def close_enough(self, goal_loc):
        return self.x - goal_loc.x < .1 and self.y - goal_loc.y < .1

    def distance(self, other_point):
        return sqrt(self.x ** 2 - other_point.x ** 2 + self.y ** 2 - other_point.y ** 2)
