#The planner class is intended to manage the decision-making when reaching a goal
from detector import Detector
class Planner():
    def __init__(self, map):
        self.reached = False
        self.map = map
        self.goal = None
        self.lost = False
        pass
    def set_goal(self, goal_id):
        pass
    def update(self, detector: Detector):
        pass
    def next_step(self):
        pass