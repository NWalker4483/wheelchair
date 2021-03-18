from tf import Pose
from detect import 
class Planner():
    def __init__(self):
        self.current_pose = Pose()
        pass
    def set_goal(self, goal: Pose):
        pass
    def update(self, frame, current_pose: Pose):
        self.current_pose = current_pose
        if self.check_goal():
            return 0,0
        return 0,0
    def check_goal(self):
        return False