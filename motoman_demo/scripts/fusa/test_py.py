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
print "rzyx"
Rt= tf.transformations.euler_matrix(math.pi/2,0,0,'rzyx')
Rtlist = tf.transformations.euler_from_matrix(Rt,'rzyx')
print Rt
print np.rad2deg(Rtlist)
print "~~~~~~~~~~~~~~~~~~~~~~~~"
print "szyx"
Rh = tf.transformations.euler_matrix(0, 0, math.pi,'rzyx')
Rhlist = tf.transformations.euler_from_matrix(Rh,'rzyx')
print Rh
print np.rad2deg(Rhlist)

H2 = Rh.dot(Rt)
H2list = tf.transformations.euler_from_matrix(H2,'rzyx')
print "rad " + str(H2list)
print "degree " + str(np.rad2deg(H2list))
print np.deg2rad(np.round(np.rad2deg(H2list), 5))

form = [0.000000001, 1.2222232, 3.13232]
print np.round(form,2)
