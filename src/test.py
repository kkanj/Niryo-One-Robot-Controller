#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
import rospy
import time

rospy.init_node('niryo_one_test_move_up')

print("--- Start")

n = NiryoOne()

def calibrate_robot():
    retry_count = 5
    while retry_count > 0:
        try:
            print("Calibrating robot...")
            n.calibrate_auto(timeout=30)  # Increase timeout to 30 seconds
            print("Calibration finished!\n")
            return True
        except NiryoOneException as e:
            print("Error during calibration: {}".format(e))
            retry_count -= 1
            time.sleep(2)
    return False

try:
    # Calibrate robot first
    if not calibrate_robot():
        print("Failed to calibrate robot after multiple attempts.")
        exit(1)

    time.sleep(1)

    # Deactivate learning mode
    n.activate_learning_mode(False)
    print("Learning mode deactivated")

    # Move up
    n.move_pose(0, 0, 0.1, 0, 0, 0)  # Move up by 0.1 meters

except NiryoOneException as e:
    print(e)

print("--- End")