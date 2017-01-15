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

dist = 1
dist_list = []
for num in range(1,7):
    exec('dist_{} = {}'.format(num,dist))
    print dist_2
#    print eval('dist_' + str('num'))
    dist = dist +1
    #dist_list.append(dis)
