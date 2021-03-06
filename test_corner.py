import test_following as tf
import test_turning as tt

def main(driver, detector, Q1, Q2, Q3, joint_dir):
    tf.main(driver, detector, start_marker=Q1, stop_marker=Q2)
    tt.main(driver, detector, marker_id=Q2, direction=joint_dir)
    tf.main(driver, detector, start_marker=Q2, stop_marker=Q3)

if __name__ == "__main__":
    from driver import Driver
    from detector import Detector

    driver = Driver()
    detector = Detector(debug = True)

    main(driver, detector, 1, 4, 2, "left")