#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys

def cartesian_position_control():
    # Initialize the moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_position_control', anonymous=True)

    # Instantiate a RobotCommander object, a PlanningSceneInterface object, and a MoveGroupCommander object
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Set the reference frame for pose targets
    group.set_pose_reference_frame("base_link")

    # Allow replanning to increase the odds of a solution
    group.allow_replanning(True)

    # Set the start state to the current state
    group.set_start_state_to_current_state()

    # Define the home, pick, and place positions in Cartesian coordinates
    home_pose = geometry_msgs.msg.Pose()
    home_pose.position.x = 0.0
    home_pose.position.y = 0.5
    home_pose.position.z = 0.6
    home_pose.orientation.w = 1.0

    pick_pose = geometry_msgs.msg.Pose()
    pick_pose.position.x = 0.8
    pick_pose.position.y = 0.3
    pick_pose.position.z = 0.2
    pick_pose.orientation.w = 1.0

    place_pose = geometry_msgs.msg.Pose()
    place_pose.position.x = -0.8
    place_pose.position.y = 0.3
    place_pose.position.z = 0.2
    place_pose.orientation.w = 1.0

    # Function to move to Cartesian positions
    def move_to_cartesian_pose(pose):
        group.set_pose_target(pose)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        current_pose = group.get_current_pose().pose
        rospy.loginfo("Current pose: %s", current_pose)

    # Move to home position
    rospy.loginfo("Moving to home position")
    move_to_cartesian_pose(home_pose)

    # Move to pick position
    rospy.loginfo("Moving to pick position")
    move_to_cartesian_pose(pick_pose)
    rospy.loginfo("Picking up the cube (simulate gripper control)")

    # Move to place position
    rospy.loginfo("Moving to place position")
    move_to_cartesian_pose(place_pose)
    rospy.loginfo("Placing the cube (simulate gripper control)")

    # Return to home position
    rospy.loginfo("Returning to home position")
    move_to_cartesian_pose(home_pose)

    rospy.sleep(5)

if __name__ == '__main__':
    try:
        cartesian_position_control()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
