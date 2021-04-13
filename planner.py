# The planner sends commands to the driver and stores the actual position of the robot

from tf import Pose
from detect import getGuideLinePosition, checkObstacleDetection

class Planner():
    def __init__(self):
        self.current_pose = Pose()
        self.current_goal = Pose()

    def set_goal(self, goal: Pose):
        self.current_goal = goal
        
    def update(self, current_pose: Pose):
        self.current_pose = current_pose
        if self.check_goal():
            return 0, 0
        return 0, 0

    def check_goal(self):
        return False
