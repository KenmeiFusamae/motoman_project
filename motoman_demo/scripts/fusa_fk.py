#!/usr/bin/env python

import math
import time
import moveit_commander
import moveit_msgs.msg
import rospy
import sys
import geometry_msgs.msg
import copy
import tf2_ros
import tf
from std_msgs.msg import Int32
from ar_pose.msg import ARMarker
from control_msgs.msg import GripperCommand

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)

        GRIPPER_OPEN = [0.08]
        GRIPPER_CLOSED = [-0.05]
        GRIPPER_NEUTRAL = [0.01]

        # Connect to the right_arm move group
        right_arm = moveit_commander.MoveGroupCommander('arm')

        # Connect to the right_gripper move group
        right_gripper = moveit_commander.MoveGroupCommander('gripper')

        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()

        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))

        # Set a small tolerance on joint angles
        # right_arm.set_goal_joint_tolerance(0.001)
        right_gripper.set_goal_joint_tolerance(0.001)

        # Start the arm target in "resting" pose stored in the SRDF file
        #right_arm.set_named_target('resting')
        #
        # # Plan a trajectory to the goal configuration
        # traj = right_arm.plan()
        #
        # # Execute the planned trajectory
        # right_arm.execute(traj)
        #
        # # Pause for a moment
        # rospy.sleep(1)
        #
        # Set the gripper target to neutal position using a joint value target
        print "gripper   neutral"
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)

        # Plan and execute the gripper motion
        right_gripper.go()
        rospy.sleep(2)

        # Set target joint values for the arm: joints are in the order they appear in
        # the kinematic tree.
        # joint_positions = [-0.0867, -1.274, 0.02832, 0.0820, -1.273, -0.003]

        # Set the arm's goal configuration to the be the joint positions
        # right_arm.set_joint_value_target(joint_positions)
        #
        # # Plan and execute the motion
        # right_arm.go()
        # rospy.sleep(1)

        # Save this configuration for later
        #right_arm.remember_joint_values('saved_config', joint_positions)

        # Close the gripper as if picking something up
        print "gripper   closed"
        right_gripper.set_joint_value_target(GRIPPER_CLOSED)
        right_gripper.go()
        rospy.sleep(2)

        # Set the arm target to the named "straight_out" pose stored in the SRDF file
        #right_arm.set_named_target('straight_forward')

        # Plan and execute the motion
        # right_arm.go()
        # rospy.sleep(1)

        # Set the goal configuration to the named configuration saved earlier
        # right_arm.set_named_target('saved_config')
        #
        # # Plan and execute the motion
        # right_arm.go()
        # rospy.sleep(1)

        # Open the gripper as if letting something go
        print "gripper   open"
        right_gripper.set_joint_value_target(GRIPPER_OPEN)
        right_gripper.go()
        rospy.sleep(2)

        # Return the arm to the named "resting" pose stored in the SRDF file
        # right_arm.set_named_target('resting')
        # right_arm.go()
        # rospy.sleep(1)

        # Return the gripper target to neutral position
        # right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        # right_gripper.go()
        # rospy.sleep(1)

        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass











#--------------------------------------------------------------------------

def callback(message):
    print message.pose.pose.position.x


def main():
    #======== init node ========#
    rospy.init_node('ar_lisner')


    sub = rospy.Subscriber('/ar_pose_marker',ARMarker,callback)

    #======== moveit init ========#
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")

    arm.set_planner_id('RRTConnectkConfigDefault')
    arm_initial_pose = arm.get_current_pose().pose
    target_pose = geometry_msgs.msg.Pose()
    #arm.set_planner_id('RRTConnectkConfigDefault')

    #======== tf lisner ========#
    tf_buffer = tf2_ros.Buffer()
    tf_listner = tf2_ros.TransformListener(tf_buffer)
    get_tf_flg = False

    count = 0

    while not get_tf_flg :
        try :
            start = time.time()
            print "count = " , count
            count = count +1




            target_pose.position.x = 0
            target_pose.position.y = 0
            target_pose.position.z = 0
            q = (0,
                 0,
                0,
                 0)
            (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(q)
            # roll -= math.pi/6.0
            pitch += math.pi/2.0
            # yaw += math.pi/4.0
            tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            target_pose.orientation.x = tar_q[0]
            target_pose.orientation.y = tar_q[1]
            target_pose.orientation.z = tar_q[2]
            target_pose.orientation.w = tar_q[3]

            arm.set_pose_target(target_pose)
            print "====== arm go ====== "
            arm.go()
            arm.clear_pose_targets()
            #
            # elapsed_time = time.time() - start
            # print("elapsed_time:{0}".format(elapsed_time))
            # print "====== arm clear ====== "

#compute_cartesian_path



        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) :
            continue


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
