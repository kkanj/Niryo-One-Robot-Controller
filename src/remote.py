#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
import rospy
import math
import threading
import sys

# Initialize ROS node
rospy.init_node('niryo_one_remote_control')

# Initialize Niryo One API
n = NiryoOne()

def calibrate_robot():
    try:
        print("Calibrating robot...")
        n.calibrate_auto()
        print("Calibration finished!")
    except NiryoOneException as e:
        print(f"Error during calibration: {e}")

# Movement functions
def move_joint(index, delta):
    try:
        joints = n.get_joints()
        joints[index] += delta
        n.move_joints(joints)
    except NiryoOneException as e:
        print(f"Error during joint movement: {e}")

def move_pose(dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0):
    try:
        pose = n.get_arm_pose()
        new_pose = [
            pose[0] + dx,
            pose[1] + dy,
            pose[2] + dz,
            pose[3] + droll,
            pose[4] + dpitch,
            pose[5] + dyaw,
        ]
        n.move_pose(*new_pose)
    except NiryoOneException as e:
        print(f"Error during pose movement: {e}")

def handle_input():
    print("Remote control activated. Use WASDQE keys for movement, X to exit.")
    print("W/S: Move up/down\nA/D: Move left/right\nQ/E: Move forward/backward\n")

    while True:
        key = input("Command: ").lower()
        if key == 'w':
            move_pose(dz=0.01)
        elif key == 's':
            move_pose(dz=-0.01)
        elif key == 'a':
            move_pose(dx=-0.01)
        elif key == 'd':
            move_pose(dx=0.01)
        elif key == 'q':
            move_pose(dy=0.01)
        elif key == 'e':
            move_pose(dy=-0.01)
        elif key == 'x':
            print("Exiting remote control.")
            break
        else:
            print("Invalid key. Use WASDQE for movement, X to exit.")

def main():
    try:
        calibrate_robot()
        handle_input()
    except KeyboardInterrupt:
        print("\nRemote control terminated by user.")
    finally:
        n.activate_learning_mode(True)
        print("Robot set to learning mode.")

if __name__ == "__main__":
    main()
