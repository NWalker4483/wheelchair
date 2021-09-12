def main(planner, start, stop):
    first_pass = True
    planner.set_goal(stop)
    while True:
        planner.update()
        if planner.finished and first_pass:
            planner.set_goal(start)
            print("Running in Reverse")
        elif planner.finished:
            print("Test Completed")
            return 
            
if __name__ == "__main__":
    from driver import Driver
    from detector import Detector
    from planner import Planner
    from utils.map import QrMap

    driver = Driver()
    detector = Detector(debug = True)
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

    main(planner, 1 , 4)