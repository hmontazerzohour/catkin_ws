#!/usr/bin/env python
import sys
from time import sleep
import rospy
import moveit_commander
from sensor_msgs.msg import JointState

def joint_pick_and_place():
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('joint_pick_and_place', anonymous=True)

    # Instantiate a RobotCommander object, a PlanningSceneInterface object, and a MoveGroupCommander object
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Define the home, start, and drop zone joint positions (in radians)
    home_joint_positions = [1.57, -1.57, 1.57, 1.57, 1.57, 0.0]
    start_joint_positions = [0.0, -1.2, 1.5, -1.5, 1.5, 0.5]
    pick_joint_positions = [0.2, -0.7, 1.57, -2.5, -1.57, 0.5]
    place_joint_positions = [2.5, -0.7, 1.57, -2.5, -1.57, 0.5]

    # Function to move to joint positions
    def move_to_joint_positions(joint_positions):
        group.go(joint_positions, wait=True)
        group.stop()
        current_joints = group.get_current_joint_values()
        rospy.loginfo("Current joint positions: %s", current_joints)

    # Move to home position
    rospy.loginfo("Moving to home position")
    move_to_joint_positions(home_joint_positions)

    # # Move to start position
    # rospy.loginfo("Moving to start position")
    # move_to_joint_positions(start_joint_positions)

    # Pick up the cube
    rospy.loginfo("Moving to pick position")
    move_to_joint_positions(pick_joint_positions)
    rospy.loginfo("Picking up the cube (simulate gripper control)")
    sleep(3)

    # Move to place position
    rospy.loginfo("Moving to place position")
    move_to_joint_positions(place_joint_positions)
    rospy.loginfo("Placing the cube (simulate gripper control)")

    # # Return to home position
    # rospy.loginfo("Returning to home position")
    # move_to_joint_positions(home_joint_positions)

    rospy.spin()

if __name__ == '__main__':
    try:
        joint_pick_and_place()
    except rospy.ROSInterruptException:
        pass
