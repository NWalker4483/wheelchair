from detector import Detector
from driver import Driver
import test_following as tf2
import test_turning as tf
from utils import thread_with_exception

# The planner class is intended to manage the decision-making when reaching a goal

class Planner():
    def __init__(self, driver, detector, map_):
        self.driver = driver
        self.detector = detector
        self.map = map_
        self.process = None
        
        self.goal = None
        self.last_seen = None
        self.started = False
        self.finished = True
        self.plan = []

    def set_goal(self, goal_id):
        if self.map.node_exists(goal_id):
            if self.goal == goal_id:
                return
            else:
                self.exit_plan()
                self.goal = goal_id
                self.finished = False
                self.started = False
        else:
            print(f"QR Code {goal_id} does not exist in the provided map file")

    def travel_path(self):
        path = self.plan
        print(f"Starting Path from {path[0]} to {path[-1]}")
        for i in range(len(path) - 1):
            direction = self.map.get_connection_direction(path[i], path[i+1])   
            tf.main(self.driver, self.detector, path[i], direction) # Face next destination 
            tf2.main(self.driver, self.detector, path[i], path[i + 1]) # Follow Line
        self.exit_plan()
        print(f"Goal {path[-1]} Reached")
    
    def update(self):
        if not self.started: # TODO: Make Detector.update Thread Safe 
            self.detector.update()
        marker_info = self.detector.state_info.get("marker", None)
        if marker_info != None:
            self.last_seen = marker_info["id"]
            if not self.started and self.goal != None:
                self.start_plan(self.last_seen, self.goal)
        if self.started and self.process != None:
            if not self.process.is_alive():
                self.exit_plan()

    def start_plan(self, start, stop):
        print("Start Plan Called")
        self.goal = stop
        self.plan = self.map.get_plan(start, self.goal) 
        self.finished = False
        self.started = True
        self.driver.stop()
        self.process = thread_with_exception(target=self.travel_path, args=[])
        self.process.start()

    def exit_plan(self):
        print("Exit Plan Called")
        self.goal = None
        self.plan = []
        if self.process != None:
            self.process.raise_exception()
            #self.process.join()
            self.process = None
        self.finished = True
        self.started = False
            
def main(planner, start, stop):
    first_pass = True
    planner.set_goal(stop)
    while True:
        planner.update()
        if planner.finished and first_pass:
            planner.set_goal(start)
            first_pass = False
            print("Running in Reverse")
        elif planner.finished:
            print("Test Completed")
            return 
            
if __name__ == "__main__":
    from driver import Driver
    from detector import Detector
    from utils.map import QrMap

    driver = Driver()
    detector = Detector(debug=not True)
    map_ = QrMap()

    """
      2__3
    __|  |
      1  4
    """
    map_.add_connection(1, "right", 2, "bottom")
    map_.add_connection(2, "left", 3, "bottom")
    map_.add_connection(3, "left", 4, "bottom")

    planner = Planner(driver, detector, map_)
    while True:
        main(planner, 1 , 4)
