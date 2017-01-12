#!/usr/bin/env python
# coding: UTF-8

import math
import rospy, sys
import moveit_commander
import tf2_ros
import tf
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

REFERENCE_FRAME = 'world'

def printHello(string):
    print 'hello, %s' % string

def grasp_pose_1(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x
    grasp_pose.pose.position.y = y
    grasp_pose.pose.position.z = z
    rolll = 0
    pitch = math.pi/2
    yaw =  0
    tar_q = tf.transformations.quaternion_from_euler(rolll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_1 -----------"
    print grasp_pose.pose.orientation.x
    print grasp_pose.pose.orientation.y
    print grasp_pose.pose.orientation.z
    print grasp_pose.pose.orientation.w

    return grasp_pose

def grasp_pose_2(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x
    grasp_pose.pose.position.y = y
    grasp_pose.pose.position.z = z
    rolll = 0
    pitch = math.pi/2
    yaw =  math.pi/2
    tar_q = tf.transformations.quaternion_from_euler(rolll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_1 -----------"
    print grasp_pose.pose.orientation.x
    print grasp_pose.pose.orientation.y
    print grasp_pose.pose.orientation.z
    print grasp_pose.pose.orientation.w

    return grasp_pose

def grasp_pose_3(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x
    grasp_pose.pose.position.y = y
    grasp_pose.pose.position.z = z
    rolll = 0
    pitch = 0
    yaw =  math.pi/2  #-3*math.pi/2
    tar_q = tf.transformations.quaternion_from_euler(rolll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_1 -----------"
    print grasp_pose.pose.orientation.x
    print grasp_pose.pose.orientation.y
    print grasp_pose.pose.orientation.z
    print grasp_pose.pose.orientation.w

    return grasp_pose

def grasp_pose_4(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x
    grasp_pose.pose.position.y = y
    grasp_pose.pose.position.z = z
    rolll = 0
    pitch = 0
    yaw =  0
    tar_q = tf.transformations.quaternion_from_euler(rolll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_1 -----------"
    print grasp_pose.pose.orientation.x
    print grasp_pose.pose.orientation.y
    print grasp_pose.pose.orientation.z
    print grasp_pose.pose.orientation.w

    return grasp_pose

def grasp_pose_5(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x
    grasp_pose.pose.position.y = y
    grasp_pose.pose.position.z = z
    rolll = 0
    pitch = 0
    yaw =  -math.pi/2
    tar_q = tf.transformations.quaternion_from_euler(rolll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_1 -----------"
    print grasp_pose.pose.orientation.x
    print grasp_pose.pose.orientation.y
    print grasp_pose.pose.orientation.z
    print grasp_pose.pose.orientation.w

    return grasp_pose

def grasp_pose_6(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x
    grasp_pose.pose.position.y = y
    grasp_pose.pose.position.z = z
    rolll = 0
    pitch = 0
    yaw =  math.pi
    tar_q = tf.transformations.quaternion_from_euler(rolll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_1 -----------"
    print grasp_pose.pose.orientation.x
    print grasp_pose.pose.orientation.y
    print grasp_pose.pose.orientation.z
    print grasp_pose.pose.orientation.w

    return grasp_pose
