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
        if len(current_path) == 0: 
            current_step = -1
            current_path = [-1]
        if (current_goal != current_path[-1]):
            try:
                current_path = map.get_plan(current_path[current_step], current_goal)
                current_step = 0
            except Exception as e:
                print("Invalid Goal Set")
                raise(e)

    def update(self):
        self.detector.update()
        if self.detector.state_info.get("marker"):
            marker_id = self.detector.state_info.get("marker")["id"]
            if marker_id == self.goal:
                # self.plan = self.map.get_plan
                self.driver.stop()
                self.goal = None

    def exit_plan(self):
        self.goal = None
        self.plan = []
        self.step = 0
        self.finished = True
            

#     elif marker_id == current_path[current_step]: # We're on the next step
#         current_step += 1
#         if (current_step == len(current_path)) or (marker_id == current_goal):
#             current_path = []
#             current_goal = -1
#             current_step = 0
#             print("Goal Reached")
#             continue
#         direction = map.get_connection_direction(current_path[current_step], current_path[current_step + 1]) # Face Direction 
#         driver.face(direction, detector, marker_id)
#     elif (current_step == 0) or (marker_id not in current_path): # We still havent found the first marker so just 
#         current_path = [marker_id]
#         current_step = 0
#         current_goal = -1
#         continue    # Just Wait