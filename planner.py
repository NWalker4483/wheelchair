from detector import Detector
from driver import Driver
import test_following as tf2
import test_facing as tf
from threading import Thread
# The planner class is intended to manage the decision-making when reaching a goal

class Planner():
    def __init__(self, driver, detector, map):
        self.driver = driver
        self.detector = detector
        self.map = map
        self.goal = None
        self.started = False
        self.finished = True
        self.step = 0
        self.plan = []

    def set_goal(self, goal_id):
        if self.map.node_exists(goal_id):
            if self.goal == goal_id:
                return
            else:
                self.exit_plan()
                self.goal = goal_id
        else:
            print(f"QR Code {goal_id} does not exist in the provided map file")

    def update(self):
        self.detector.update()
        if self.detector.state_info.get("marker"):
            marker_id = self.detector.state_info.get("marker")["id"]
            if self.started:
                if marker_id == self.goal:
                    self.driver.stop()
                    self.exit_plan()
                    print("Goal Reached")
                elif marker_id == self.plan[self.step]:
                    self.driver.stop()
                    self.step += 1
                    direction = self.map.get_connection_direction(self.plan[self.step - 1], self.plan[self.step])
                    
                    # Execute Tested Behaviors
                    def take_step(detector, driver, Q1, Q2, direction):
                        tf.main(driver, detector, Q1, direction)
                        tf2.main(driver, detector, Q1, Q2)
                  
                    self.process = Thread(target=take_step, args=[self.detector, self.driver, self.plan[self.step - 1], self.plan[self.step], direction])
                    self.process.start()
            else:
                if self.goal != None:
                    self.start_plan(marker_id, self.goal)
        if "line" not in self.detector.state_info:
            # Stop after a while of no detections
            pass

    def start_plan(self, start, stop):
        self.goal = stop
        self.plan = self.map.get_plan(start, self.goal) 
        self.step = 0
        self.finished = False
        self.started = True

    def exit_plan(self):
        self.goal = None
        self.plan = [-1]
        self.step = 0
        self.finished = True
        self.started = False