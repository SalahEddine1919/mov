#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
 
class MoveItFkDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
 
                 # Initialize ROS node
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
                 # Initialize the arm group in the robot arm that needs to be controlled by the move group
        arm = moveit_commander.MoveGroupCommander('arm')
        
                 # Initialize the gripper group in the robot arm controlled by the move group
        gripper = moveit_commander.MoveGroupCommander('gripper')
        
                 # Set the allowable error value of the robot arm and gripper
        arm.set_goal_joint_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)
        
                 # Control the robot arm to return to the initial position first
        '''
        arm.set_named_target('bow')
        arm.go()
        rospy.sleep(2)
        '''
                 # Set the target position of the gripper and control the gripper movement
        
        gripper.set_named_target("opened")
        gripper.go()
        rospy.sleep(1)
        
                 # Set the target position of the robot arm and use six-axis position data to describe (unit: radians)
        joint_positions = [0.0,1.57,1.0,0.0,-1.0]
        result=arm.set_joint_value_target(joint_positions)
        rospy.loginfo(str(result))
                 
                 # Control the robot arm to complete the movement
        arm.go()
        joint=arm.get_current_joint_values()
        print("final joint=",joint)
        pose=arm.get_current_pose('arm_link5')
        print('pose=',pose)
        rospy.sleep(1)
        
        gripper.set_named_target("closed")
        gripper.go()
        rospy.sleep(1)
        
        arm.set_named_target('bow')
        arm.go()
        rospy.sleep(2)

                 # Close and exit moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass