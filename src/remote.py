#!/usr/bin/env python

import rospy
import time
import curses
from niryo_one_python_api.niryo_one_api import NiryoOne, NiryoOneException

# Initialize ROS node
rospy.init_node('niryo_one_remote_control')
n = NiryoOne()

def calibrate_robot():
    try:
        print("Calibrating robot...")
        n.calibrate_auto()
        print("Calibration finished!")
    except NiryoOneException as e:
        print("Error during calibration: {}".format(e))

def cool_dance(stdscr):
    stdscr.clear()
    stdscr.nodelay(True)
    stdscr.addstr("Cool dance in progress! Press X to stop.\n")

    dance_moves = [
        [0, 0.3, 0, 0.3, 0, 0],
        [0, -0.3, 0, -0.3, 0, 0],
        [0, 0, 0.3, 0, 0.3, 0],
        [0, 0, -0.3, 0, -0.3, 0]
    ]

    while True:
        key = stdscr.getch()
        if key == ord('x'):
            print("Stopping!")
            break
        for move in dance_moves:
            try:
                joints = n.get_joints()
                new_joints = [j + m for j, m in zip(joints, move)]
                n.move_joints(new_joints)
            except NiryoOneException as e:
                print("Error during dance movement:", e)

            time.sleep(1)
            # Check again if user pressed 'x'
            key = stdscr.getch()
            if key == ord('x'):
                print("Stopping!")
                return

def main(stdscr):
    calibrate_robot()
    cool_dance(stdscr)

if __name__ == "__main__":
    curses.wrapper(main)