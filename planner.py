# The planner sends commands to the driver and stores the actual position of the robot

from tf import Pose
class Planner():
    def __init__(self, map = None):
        self.current_pose = Pose()
        self.current_goal = Pose()
        self.cmd = None
    
    def set_goal(self, goal: Pose):
        self.current_goal = goal
    
    def update(self, manager):
        if self.check_goal():
            return 0, 0
        return 0, 0

    def check_goal(self):
        return False
