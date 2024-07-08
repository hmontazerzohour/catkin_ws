#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from sensor_msgs.msg import JointState

def joint_position_control():
    # Initialize the moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('joint_position_control', anonymous=True)

    # Instantiate a RobotCommander object, a PlanningSceneInterface object, and a MoveGroupCommander object
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Define the joint positions (in radians) we want to move to
    joint_goal = group.get_current_joint_values()
    joint_goal[0] =  0.2  # Joint 1
    joint_goal[1] = -0.7  # Joint 2
    joint_goal[2] =  1.57  # Joint 3
    joint_goal[3] =  -2.5  # Joint 4
    joint_goal[4] =  -1.57  # Joint 5
    joint_goal[5] =  0.5   # Joint 6


    # Move to the joint position goal
    group.go(joint_goal, wait=True)

    # Ensure there is no residual movement
    group.stop()

    # Get and print the final joint values
    current_joints = group.get_current_joint_values()
    rospy.loginfo("Current joint positions: %s", current_joints)

    # Shut down moveit_commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        joint_position_control()
    except rospy.ROSInterruptException:
        pass
