#!/usr/bin/env python
# coding: UTF-8

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
        [cos(p)*cos(y), sin(r)*sin(p)*sin(y)-cos(r)*cos(y), sin(r)*sin(y)+cos(r)*sin(p)*cos(y) ],
        [cos(p)*cos(y), sin(r)*sin(p)*sin(y)+cos(r)*cos(y), -sin(r)*sin(y)+cos(r)*sin(p)*sin(y)],
        [-sin(p), sin(r)*cos(p), cos(r)*cos(p)]])
    return R

def R_return(R):
    print R

# Rt = R_mat(math.pi/2,0,0)
# Rh = R_mat(0,0,-math.pi/2)
# Rr = np.dot(Rh,Rt)
# #math.atan2(y,x) y/x
# pitch = math.atan2(-Rr[2,0],math.sqrt(Rr[0,0]**2+Rr[1,0]**2))
# print pitch
# print math.degrees(pitch)


Pit = R_mat(0,math.pi/2,0)
pitch = math.atan2(-Pit[2,0],math.sqrt(Pit[0,0]**2+Pit[1,0]**2))
print pitch
pitch = math.asin(-Pit[2,0])
roll = math.atan2(Pit[2,1],Pit[2,2])

print "--------------------"
R_return(Pit)
print pitch
print roll



sp = sin(math.pi/6)
print math.asin(-sp)
