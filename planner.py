from detector import Detector
from driver import Driver
# The planner class is intended to manage the decision-making when reaching a goal

class Planner():
    def __init__(self, map):
        self.map = map
        self.goal = None
        self.finished = True
        self.step = 0
        self.plan = []

    def set_goal(self, goal_id):
        # Validate Goal 

        if self.goal == goal_id:
            print(f"Current Goal: {goal_id}")
            return
        else:
            self.exit_plan()
            self.goal = goal_id

    def update(self):
        self.detector.update()
        if self.detector.state_info.get("marker"):
            marker_id = self.detector.state_info.get("marker")["id"]
            if marker_id == self.goal:
                self.exit_plan()
                print("Goal Reached")
                self.driver.stop()
            elif marker_id == self.plan[self.step]:
                self.driver.stop()
                if len(self.plan) == 1:
                    self.plan = self.map.get_plan(marker_id, self.goal)
                self.step += 1
                #direction = self.map.get_connection_direction(self.plan[self.step - 1], self.plan[self.step]) # Face Direction 
                # driver.face(direction, detector, marker_id)
                # driver.follow()
                pass
        if "line" not in self.detector.state_info:
            pass
    def exit_plan(self):
        self.goal = None
        self.plan = [-1]
        self.step = 0
        self.finished = True