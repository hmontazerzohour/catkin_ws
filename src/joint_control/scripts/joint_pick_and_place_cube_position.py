#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Vector3
import threading

class PickAndPlace:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('joint_pick_and_place', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        self.cube_pose_pub = rospy.Publisher('cube_pose', Pose, queue_size=10)
        self.cube_up_vector_pub = rospy.Publisher('cube_up_vector', Vector3, queue_size=10)

        self.cube_pose = Pose()
        self.cube_up_vector = Vector3()

        self.stop_thread = False
        self.thread = threading.Thread(target=self.publish_cube_info)
        self.thread.start()

    def move_to_joint_positions(self, joint_positions):
        self.group.go(joint_positions, wait=True)
        self.group.stop()
        current_joints = self.group.get_current_joint_values()
        rospy.loginfo("Current joint positions: %s", current_joints)

    def update_cube_info(self, position, orientation):
        self.cube_pose.position = position
        self.cube_pose.orientation = orientation

        # Assume the "up" vector is the z-axis of the orientation
        self.cube_up_vector.x = 0.0
        self.cube_up_vector.y = 0.0
        self.cube_up_vector.z = 1.0

    def publish_cube_info(self):
        rate = rospy.Rate(2)  # 2 Hz, which is every 500 ms
        while not rospy.is_shutdown() and not self.stop_thread:
            self.cube_pose_pub.publish(self.cube_pose)
            self.cube_up_vector_pub.publish(self.cube_up_vector)
            rate.sleep()

    def pick_and_place_task(self):
        home_joint_positions = [1.57, -1.57, 1.57, 1.57, 1.57, 0.0]
        start_joint_positions = [0.0, -1.2, 1.5, -1.5, 1.5, 0.5]
        pick_joint_positions = [0.2, -0.7, 1.57, -2.5, -1.57, 0.5]
        place_joint_positions = [2.5, -0.7, 1.57, -2.5, -1.57, 0.5]



        rospy.loginfo("Moving to home position")
        self.move_to_joint_positions(home_joint_positions)

        rospy.loginfo("Moving to start position")
        self.move_to_joint_positions(start_joint_positions)
        self.update_cube_info(self.group.get_current_pose().pose.position, self.group.get_current_pose().pose.orientation)

        rospy.loginfo("Moving to pick position")
        self.move_to_joint_positions(pick_joint_positions)
        rospy.loginfo("Picking up the cube (simulate gripper control)")
        self.update_cube_info(self.group.get_current_pose().pose.position, self.group.get_current_pose().pose.orientation)

        rospy.loginfo("Moving to place position")
        self.move_to_joint_positions(place_joint_positions)
        rospy.loginfo("Placing the cube (simulate gripper control)")
        self.update_cube_info(self.group.get_current_pose().pose.position, self.group.get_current_pose().pose.orientation)

        rospy.loginfo("Returning to home position")
        self.move_to_joint_positions(home_joint_positions)

        self.stop_thread = True
        self.thread.join()

if __name__ == '__main__':
    try:
        pick_and_place = PickAndPlace()
        pick_and_place.pick_and_place_task()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
