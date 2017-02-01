#!/usr/bin/env python
# coding: UTF-8

import math
import numpy as np
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
    grasp_pose.pose.position.z = z + 0.06
    roll = 0
    pitch = math.pi/2
    yaw =  0
    tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_1 -----------"
    print "(roll, pitch, yaw) = (%f, %f, %f)" % (roll, pitch, yaw)

    return grasp_pose, roll, pitch, yaw

def grasp_pose_2(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x
    grasp_pose.pose.position.y = y
    grasp_pose.pose.position.z = z + 0.06
    roll = 0
    pitch = math.pi/2
    yaw =  math.pi/2
    tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_2 -----------"
    print "(roll, pitch, yaw) = (%f, %f, %f)" % (roll, pitch, yaw)

    return grasp_pose, roll, pitch, yaw

def grasp_pose_3(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x
    grasp_pose.pose.position.y = y - 0.015
    grasp_pose.pose.position.z = z
    roll = 0
    pitch = 0
    yaw =  math.pi/2  #-3*math.pi/2
    tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_3 -----------"
    print "(roll, pitch, yaw) = (%f, %f, %f)" % (roll, pitch, yaw)

    return grasp_pose, roll, pitch, yaw

def grasp_pose_4(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x - 0.02
    grasp_pose.pose.position.y = y
    grasp_pose.pose.position.z = z
    roll = 0
    pitch = 0
    yaw =  0
    tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_4 -----------"
    print "(roll, pitch, yaw) = (%f, %f, %f)" % (roll, pitch, yaw)

    return grasp_pose, roll, pitch, yaw

def grasp_pose_5(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x
    grasp_pose.pose.position.y = y + 0.015
    grasp_pose.pose.position.z = z
    roll = 0
    pitch = 0
    yaw =   -math.pi/2   #piまでで表現したほうがいいかも  # 3*math.pi/2
    tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_5 -----------"
    print "(roll, pitch, yaw) = (%f, %f, %f)" % (roll, pitch, yaw)

    return grasp_pose, roll, pitch, yaw

def grasp_pose_6(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x + 0.02
    grasp_pose.pose.position.y = y
    grasp_pose.pose.position.z = z
    roll = 0
    pitch = 0
    yaw =  math.pi
    tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_6 -----------"
    print "(roll, pitch, yaw) = (%f, %f, %f)" % (roll, pitch, yaw)

    return grasp_pose, roll, pitch, yaw

def grasp_pose_7(x, y, z):
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = REFERENCE_FRAME
    grasp_pose.pose.position.x = x + 0.02
    grasp_pose.pose.position.y = y
    grasp_pose.pose.position.z = z
    roll = math.pi/2
    pitch = 0
    yaw =  0
    tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    grasp_pose.pose.orientation.x = tar_q[0]
    grasp_pose.pose.orientation.y = tar_q[1]
    grasp_pose.pose.orientation.z = tar_q[2]
    grasp_pose.pose.orientation.w = tar_q[3]
    print "------- grasp_pose_7 -----------"
    print "(roll, pitch, yaw) = (%f, %f, %f)" % (roll, pitch, yaw)

    return grasp_pose, roll, pitch, yaw

def calc_dist(x1, y1, z1, x2, y2, z2):
    a = np.array([x1,y1,z1])
    b = np.array([x2,y2,z2])
    dist = np.linalg.norm(b-a)
    #print "dist = %f " % dist

    return dist

def calc_e_dist(dist, dist_min, dist_max):
    e_dist = 1 -(dist - dist_min)/(dist_max - dist_min)
    return e_dist

# def calc_deg(init_roll, init_pitch, init_yaw, grasp_roll, grasp_pitch, grasp_yaw):
#     deg = abs(init_roll - grasp_roll)+abs(init_pitch - grasp_pitch)+abs(init_yaw - grasp_yaw)
#     return deg

def calc_e_deg(deg):
    e_deg = 1 - (deg / math.pi)
    return e_deg

def calc_e_deg(init_roll, init_pitch, init_yaw, grasp_roll, grasp_pitch, grasp_yaw):
    deg = abs(init_roll - grasp_roll)+abs(init_pitch - grasp_pitch)+abs(init_yaw - grasp_yaw)
    e_deg = 1 - (deg / (3*math.pi))
    return e_deg



def extraction_pose(num, rpy_list):
    if 89 < rpy_list[0] < 91 or -91 < rpy_list[0] < -89 or -91 < rpy_list[1] < -89:
        return 0
    else:
        return num

def choose_pose(can_list, e_list):
    get_list  = []
    for  i in range(len(e_list)):
        if e_list[i][0] in can_list:
            get_list.append(e_list[i][0])
    return get_list
