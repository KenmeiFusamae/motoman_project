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

# def calc_e_deg(deg):
#     e_deg = 1 - (deg / math.pi)
#     return e_deg

def calc_e_deg(init_roll, init_pitch, init_yaw, grasp_roll, grasp_pitch, grasp_yaw):
    deg = abs(init_roll - grasp_roll)+abs(init_pitch - grasp_pitch)+abs(init_yaw - grasp_yaw)
    e_deg = 1 - (deg / (3*math.pi))
    return e_deg

def calc_e_deg7(start_list, end_list):
    deg = 0
    for i in range(len(start_list)):
        d = abs(end_list[i] - start_list[i])

        deg += d
    e_deg = 1 - (deg / (7*180))
    return e_deg


def rotation_direc_vector(Rti, Rtj, Rtk, Rhi, Rhj, Rhk, r_or_s):
    v0 = np.array([1,0,0,1])
    v1 = v0.transpose()

    w0 = np.array([0,0,1,1])
    w1 = w0.transpose()
    #print v1
    #                                   yaw , pitch, roll
    Rt= tf.transformations.euler_matrix(Rti, Rtj, Rtk, r_or_s)
    Rtlist = tf
    Rh = tf.transformations.euler_matrix(Rhi, Rhj, Rhk,r_or_s)
    Rhlist = list(tf.transformations.euler_from_matrix(Rh,r_or_s))
    # print Rh# v1 = tf.transformations.unit_vector(v0)
    # print tf.transformations.euler_from_matrix(Rh,'rzyx')
    R = Rt.dot(Rh)

    v2 = Rh.dot(v1)
    v3 = Rt.dot(v2)
    v3t = v3.transpose()
    v3list = v3t.tolist()

    w2 = Rh.dot(w1)
    w3 = Rt.dot(w2)
    w3t = w3.transpose()
    w3list = w3t.tolist()

    for i in range(len(v3list)):
        if -0.01 < v3list[i] < 0.01:
            v3list[i] = 0
        if -0.01 < w3list[i] < 0.01:
            w3list[i] = 0

    return v3list, w3list


def extraction_direc_vector(num, v, w):
    if 0 < v[2]:
        return 0
    elif v[2] == 0 and  w[0] is not 0:
        return 0
    elif v[2] == 0 and  w[1] is not 0:
        return 0
    else:
        return num


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

def find_IK(Pose_stamp, end_effector_link,right_arm): #moveitのset_joint_value_targetが返り値を返すように書き換えてる
    p = False
    while not p:  #ikが見つかるまで、繰り返す
        p = right_arm.set_joint_value_target(Pose_stamp, end_effector_link)
    # print "joint value  from ik"
    # print p
    # print right_arm.get_joint_value_target()

    deg_np_list = np.rad2deg(right_arm.get_joint_value_target())
    deg_list = deg_np_list.tolist()
    #print "ik degree " +str(deg_list)
    return deg_list
