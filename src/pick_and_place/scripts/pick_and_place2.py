#!/usr/bin/env python
import sys
#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
import tf
import sys

def pick_and_place():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Define positions
    home_position = geometry_msgs.msg.Pose()
    home_position.position.x = 0.0
    home_position.position.y = 0.0
    home_position.position.z = 0.5
    home_position.orientation.w = 1.0

    start_position = geometry_msgs.msg.Pose()
    start_position.position.x = 0.5
    start_position.position.y = 0.0
    start_position.position.z = 0.3

    drop_zone = geometry_msgs.msg.Pose()
    drop_zone.position.x = 0.8
    drop_zone.position.y = 0.0
    drop_zone.position.z = 0.3

    # Function to log position and orientation
    def log_position_and_orientation(pose, label):
        rospy.loginfo(f"{label} - Position: {pose.position.x}, {pose.position.y}, {pose.position.z}")
        rospy.loginfo(f"{label} - Orientation: {pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w}")

    # Move to home position
    group.set_pose_target(home_position)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    log_position_and_orientation(home_position, "Home Position")

    # Move to start position
    group.set_pose_target(start_position)
    plan1 = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    log_position_and_orientation(start_position, "Starting Position")

    # Pick up the cube (add gripper control here)

    # Rotate cube 180 degrees
    quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)
    start_position.orientation.x = quat[0]
    start_position.orientation.y = quat[1]
    start_position.orientation.z = quat[2]
    start_position.orientation.w = quat[3]

    group.set_pose_target(start_position)
    plan2 = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    log_position_and_orientation(start_position, "Rotated Position")

    # Move to drop zone
    group.set_pose_target(drop_zone)
    plan3 = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    log_position_and_orientation(drop_zone, "Drop Zone Position")

    # Place the cube (add gripper release logic here)

    # Return to home position
    group.set_pose_target(home_position)
    plan4 = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    log_position_and_orientation(home_position, "Return to Home Position")

    rospy.spin()

if __name__ == '__main__':
    try:
        pick_and_place()
    except rospy.ROSInterruptException:
        pass
