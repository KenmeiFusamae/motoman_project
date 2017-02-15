#!/usr/bin/env python
# coding: UTF-8

import cv2
import math
import numpy as np
from numpy import sin, cos
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
# dist = 1
# dist_list = []
# for num in range(1,7):
#     exec('dist_{} = {}'.format(num,dist))
#     print dist_2
# #    print eval('dist_' + str('num'))
#     dist = dist +1
#     #dist_list.append(dis)

lis = []
tp = (1,2,3)
tp2 = (2,3,4)

lis.append(tp)
lis.append(tp2)
print len(lis)


ren = [1,3,5,7,9]
# print ren
# for num in range(1,10):
#     for i in ren:
#         print i
#     # if num == ren:
#     #     continue
#     # print "num = %d" % num

for x in lis:
    print x[0],x[1],x[2]
    #print x[0]



pose = PoseStamped()
pose.header.frame_id = REFERENCE_FRAME
pose.pose.position.x = 1
pose.pose.position.y = 1
pose.pose.position.z = 1
roll = 0
pitch = 0
yaw = math.pi
q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
pose.pose.orientation.x = q[0]
pose.pose.orientation.y = q[1]
pose.pose.orientation.z = q[2]
pose.pose.orientation.w = q[3]
print "---------- pose --------"
print pose

roll = math.pi
pitch = math.pi
yaw = math.pi
q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
pose.pose.orientation.x = q[0]
pose.pose.orientation.y = q[1]
pose.pose.orientation.z = q[2]
pose.pose.orientation.w = q[3]
print "---------pose 2 ------------"
print pose

# r = np.pi/2
# p = math.pi/2
# y = math.pi/2

# R  = np.matrix((
#     (np.cos(r),sin(0),1),
#     (2*2,2,2),
#     (3*2,3,3)
# ))
#
# print R
# print r
# print np.cos(3.14/2)



def R_mat (r,p,y):
    R = np.matrix( [
        [cos(p)*cos(y), sin(r)*sin(p)*sin(y)-cos(r)*sin(y), sin(r)*sin(y)+cos(r)*sin(p)*cos(y) ],
        [cos(p)*cos(y), sin(r)*sin(p)*sin(y)+cos(r)*cos(y), -sin(r)*cos(y)+cos(r)*sin(p)*sin(y)],
        [-sin(p), sin(r)*cos(p), cos(r)*cos(p)]])
    return R

def R_return(R):
    print R

# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :

    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])



    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])


    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R) :

    #assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

# Rt = R_mat(math.pi/2,0,0)
# Rh = R_mat(0,0,-math.pi/2)
# Rr = np.dot(Rh,Rt)
# #math.atan2(y,x) y/x
# pitch = math.atan2(-Rr[2,0],math.sqrt(Rr[0,0]**2+Rr[1,0]**2))
# print pitch
# print math.degrees(pitch)


Pit = R_mat(0,math.pi/2,math.pi/2)
pitch = math.atan2(-Pit[2,0],math.sqrt(Pit[0,0]**2+Pit[1,0]**2))
print pitch
pitch = math.asin(-Pit[2,0])
roll = math.atan2(Pit[2,1],Pit[2,2])

print "--------------------"
R_return(Pit)
print pitch
print roll
print "=========================="
theta = [0,math.pi/2,math.pi/2]
M = eulerAnglesToRotationMatrix(theta)
l = rotationMatrixToEulerAngles(M)
print M
print "^^^^^^^^^^^^^^^^^^^^^^^^^^"
print "rzyx Rt"
Rt= tf.transformations.euler_matrix(math.pi/2,0,0,'rzyx')
Rtlist = tf.transformations.euler_from_matrix(Rt,'rzyx')
print Rt
print np.rad2deg(Rtlist)
print "~~~~~~~~~~~~~~~~~~~~~~~~"
print "rzyx  Rh"
Rh = tf.transformations.euler_matrix(0, 0, -math.pi/2,'rzyx')
Rhlist = tf.transformations.euler_from_matrix(Rh,'rzyx')
print Rh
print np.rad2deg(Rhlist)
print "============================="
H2 = Rh.dot(Rt)
H2list = list(tf.transformations.euler_from_matrix(H2,'rzyx'))
print "rad " + str(np.rad2deg(H2list))


# rrr =  np.round(H2list,5)
# print "round "  + str(rrr)
# print "rad " + str(np.deg2rad(np.rad2deg(rrr)))
#
# for i in  range(len(H2list)):
#     if  -0.01 < H2list[i] < 0.01:
#         H2list[i] = 0
# print "clearn list " + str(H2list)

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
    # print "======= direction vector ======="
    print tf.transformations.euler_from_matrix(R,r_or_s)

    w2 = Rh.dot(w1)
    w3 = Rt.dot(w2)
    w3t = w3.transpose()
    w3list = w3t.tolist()
    print w3list

    # for i in range(len(v3list)):
    #     if -0.01 < v3list[i] < 0.01:
    #         v3list[i] = 0
    #     if -0.01 < w3list[i] < 0.01:
    #         w3list[i] = 0
    # print v
    # print w3list


print "======== grasp 1 ============"
rotation_direc_vector(math.pi/2, 0 , 0,  0, math.pi/2, 0, 'sxyz')

print "======== grasp 2 ============"
rotation_direc_vector(math.pi/2, 0 , 0,  0, math.pi/2, math.pi/2, 'sxyz')

print "======== grasp 3 ============"
rotation_direc_vector(math.pi/2, 0 , 0,  0, 0, math.pi/2, 'sxyz')

print "======== grasp 4 ============"
rotation_direc_vector(math.pi/2, 0 , 0,  0, 0, 0, 'sxyz')

print "======== grasp 5 ============"
rotation_direc_vector(math.pi/2, 0 , 0,  0, 0, -math.pi/2, 'sxyz')

print "======== grasp 6 ============"
rotation_direc_vector(math.pi/2, 0 , 0,  0, 0, math.pi, 'sxyz')

z= 1
if  z:
    print "positive"
elif z < 0:
    print "minasu"

# grasp_pose = PoseStamped()
# grasp_pose.header.frame_id = REFERENCE_FRAME
# grasp_pose.pose.position.x = 0+ 0.02
# grasp_pose.pose.position.y = 0
# grasp_pose.pose.position.z = 0
# roll = math.pi/2
# pitch = 0
# yaw =  0
# tar_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
# grasp_pose.pose.orientation.x = tar_q[0]
# grasp_pose.pose.orientation.y = tar_q[1]
# grasp_pose.pose.orientation.z = tar_q[2]
# grasp_pose.pose.orientation.w = tar_q[3]
# print "-------  none -----------"
# print grasp_pose.pose.orientation.x
# print grasp_pose.pose.orientation.y
# print grasp_pose.pose.orientation.z
# print grasp_pose.pose.orientation.w
#
# grasp_pose = PoseStamped()
# grasp_pose.header.frame_id = REFERENCE_FRAME
# grasp_pose.pose.position.x = 0 + 0.02
# grasp_pose.pose.position.y = 0
# grasp_pose.pose.position.z = 0
# roll = math.pi/2
# pitch = 0
# yaw =  0
# tar_q = tf.transformations.quaternion_from_euler(yaw, pitch, roll,'rzyx')
# grasp_pose.pose.orientation.x = tar_q[0]
# grasp_pose.pose.orientation.y = tar_q[1]
# grasp_pose.pose.orientation.z = tar_q[2]
# grasp_pose.pose.orientation.w = tar_q[3]
# print "------- rzyx  -----------"
# print grasp_pose.pose.orientation.x
# print grasp_pose.pose.orientation.y
# print grasp_pose.pose.orientation.z
# print grasp_pose.pose.orientation.w


# print "Rt v0"
# #print np.inner(Rt,v1)
#
# print "Rh Rt v0"
# #print Rh.dot(Rt).dot(v0)
#
# R = Rh.dot(Rt)
#
# v2 = R.dot(v1)
# print "Rh Rt・ v1"
# print v2
#
# print "Rt Rh Rt v1"
#
# print Rt.dot(v2)



# print "======= numpy test ======="
# a = np.array([[1,2],[3,4]])
# b = np.array([1,2])
# print a
# print b
# print "dot "
# print np.dot(a,b.transpose())
# print "a.dot(b)"
# print a.dot(b)
# print "inner"
# print np.inner(a,b)




# li = [0,2,0,0,5,0]
# li_uniq = []
# for x in li:
#     if x not in li_uniq:
#         li_uniq.append(x)
# print li_uniq
# li_uniq.remove(0)
# print li_uniq
#
#
# print "====== list test ========"
# li_all = [3,4,1,2,6,5]
# t_list = [2,5]
# for i in li_all:
#     if i not in t_list:
#         li_all.remove(i)
# print li_all
# print t_list
# print "======= list test youso ver ======="
# li_all2 = [3,4,1,2,6,5]
# li_ture = []
# for i in range(len(li_all2)):
#     if  li_all2[i]  in t_list:
#         li_ture.append(li_all2[i])
# print li_ture
#
# print "======== list tuple が絡んだとき ========"
# tp1 = (1,434)
# tp2 = (2,23424)
# tp3 = (3,554)
# tp4 = (4, 4334)
# tp5 = (5, 212)
# tp6 = (6, 870)
# E_list = []
# E_list.append(tp3)
# E_list.append(tp4)
# E_list.append(tp1)
# E_list.append(tp2)
# E_list.append(tp6)
# E_list.append(tp5)
# print E_list
# print E_list[1][0]
# print E_list[1][1]
# print len(E_list)
# get_list = []
# for i in range(len(E_list)):
#     if E_list[i][0] in t_list:
#         get_list.append(E_list[i][0])
# print "get list "
# print get_list
