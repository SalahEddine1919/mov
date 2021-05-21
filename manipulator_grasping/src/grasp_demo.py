#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf import TransformListener

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

arm_group = moveit_commander.MoveGroupCommander("arm")
# Put the arm in the start position
arm_group.set_named_target("bow")
plan1 = arm_group.go()

hand_group = moveit_commander.MoveGroupCommander("gripper")
# Open the gripper
hand_group.set_named_target("opened")
plan2 = hand_group.go()

# get the pose
t = TransformListener ()

if t.frameExists("world") and t.frameExists("object_1"):
    (translation,rotation) = t.lookupTransform("world", "object_1", rospy.Time())

    # put the arm at the 1st grasping position
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation = rotation
    pose_target.position = translation
    pose_target.position.z -= 0.15
    arm_group.set_pose_target(pose_target)
    plan1 = arm_group.go()

    # put the arm at the 2nd grasping position
    pose_target.position.z += 0.15
    arm_group.set_pose_target(pose_target)
    plan1 = arm_group.go()

    # close the gripper
    hand_group.set_named_target("closed")
    plan2 = hand_group.go()

    # put the arm at the 3rd grasping position
    pose_target.position.z = 1.5
    arm_group.set_pose_target(pose_target)
    plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
