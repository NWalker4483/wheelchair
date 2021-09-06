#The planner class is intended to manage the decision-making when reaching a goal
from detector import Detector
class Planner():
    def __init__(self, map):
        self.reached = False
        self.map = map
        self.goal = None
        self.lost = False

    def set_goal(self, goal_id):
        pass

    def update(self, detector: Detector):
        state_info = detector.state_info
        pass

    def next_step(self):
        pass

#  elif topic == goal_update_topic:
#                 if current_goal != int(data):
#                     current_goal = int(data)
#                     print(f"Current Goal: {current_goal}")
#                     if len(current_path) == 0: 
#                         lost = True
#                         current_step = -1
#                         current_path = [-1]
#                     if (current_goal != current_path[-1]):
#                         try:
#                             current_path = map.get_plan(current_path[current_step], current_goal)
#                             current_step = 0
#                         except Exception as e:
#                             print("Invalid Goal Set")
#                             raise(e)
            
#             elif topic == "e": # For Exit
#                 running = False 
#             if (len(current_path) > 0) or lost:
#                 local_line_form, marker_id, local_marker_pose = detector.update()
                
#                 if marker_id != None: # Marker in frame
#                     if (lost): # Setup the other loop to calculate the set path
#                         current_path = [marker_id]
#                         current_step = 0
#                     elif marker_id == current_path[current_step]: # We're on the next step
#                         current_step += 1
#                         if (current_step == len(current_path)) or (marker_id == current_goal):
#                             current_path = []
#                             current_goal = -1
#                             current_step = 0
#                             print("Goal Reached")
#                             continue
#                         direction = map.get_connection_direction(current_path[current_step], current_path[current_step + 1]) # Face Direction 
#                         driver.face(direction, detector, marker_id)
#                     elif (current_step == 0) or (marker_id not in current_path): # We still havent found the first marker so just 
#                         current_path = [marker_id]
#                         current_step = 0
#                         current_goal = -1
#                         continue    # Just Wait
#                     # else:
#                     #     # TODO Following the line
#                     #     if local_line_form[0] > 0:
#                     #         driver.send_cmd(70, driver.angular + 2)
#                     #     else:
#                     #         driver.send_cmd(70, driver.angular - 2)
#         except Exception as e: