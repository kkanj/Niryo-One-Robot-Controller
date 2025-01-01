#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
import rospy
import time

rospy.init_node('niryo_one_test_move_up')

print("--- Start")

n = NiryoOne()

try:
    # Calibrate robot first
    print("Calibration finished!\n")

    time.sleep(1)

    # Deactivate learning mode
    n.activate_learning_mode(False)
    print("Learning mode deactivated")

    # Move up
    n.move_pose(0, 0, 0.1, 0, 0, 0)  # Move up by 0.1 meters

except NiryoOneException as e:
    print(e)

print("--- End")