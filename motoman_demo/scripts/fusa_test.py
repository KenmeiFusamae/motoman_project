#!/usr/bin/env python

import math
import time
import moveit_commander
import moveit_msgs.msg
import rospy
import geometry_msgs.msg
import copy
import tf2_ros
import tf
from std_msgs.msg import Int32
from ar_pose.msg import ARMarker


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
