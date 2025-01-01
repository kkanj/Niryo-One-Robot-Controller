#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
import rospy
import time

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
        print("Error during calibration: {}".format(e))

# Movement functions
def move_joint(index, delta):
    try:
        joints = n.get_joints()
        joints[index] += delta
        n.move_joints(joints)
    except NiryoOneException as e:
        print("Error during joint movement: {}".format(e))

def move_pose(dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0):
    try:
        pose = n.get_arm_pose()
        new_pose = [
            pose.position.x + dx,
            pose.position.y + dy,
            pose.position.z + dz,
            pose.rpy.roll + droll,
            pose.rpy.pitch + dpitch,
            pose.rpy.yaw + dyaw,
        ]
        n.move_pose(*new_pose)
    except NiryoOneException as e:
        print("Error during pose movement: {}".format(e))

def handle_input():
    print("Remote control activated. Use WASDQE keys for movement, X to exit.")
    print("W/S: Move up/down\nA/D: Move left/right\nQ/E: Move forward/backward\n")

    while True:
        key = raw_input("Command: ").lower()
        if key == 'w':
            move_pose(dz=0.01)
        elif key == 's':
            move_pose(dz=-0.01)
        elif key == 'a':
            move_pose(dx=-0.01)
        elif key == 'd':
            move_pose(dx=0.01)
        elif key == 'q':
            move_pose(dy=-0.01)
        elif key == 'e':
            move_pose(dy=0.01)
        elif key == 'x':
            break

def main():
    try:
        calibrate_robot()
        handle_input()
    except KeyboardInterrupt:
        print("\nRemote control terminated by user.")
    finally:
        retry_count = 5
        while retry_count > 0:
            try:
                n.activate_learning_mode(True)
                print("Robot set to learning mode.")
                break
            except NiryoOneException as e:
                print("Error setting learning mode: {}".format(e))
                retry_count -= 1
                time.sleep(2)
        if retry_count == 0:
            print("Failed to set robot to learning mode after multiple attempts.")

if __name__ == "__main__":
    main()