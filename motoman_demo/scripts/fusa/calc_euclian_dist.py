#!/usr/bin/env python

import rospy
import tf2_ros
import math
import numpy as np

if __name__ == '__main__':
    rospy.init_node('calc_euclidean')
    tf_buffer = tf2_ros.Buffer()
    tf_listner = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10.0)
    count = 0

    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('world', 'right_gripper_link',
                                                rospy.Time())
            if count ==0:
                x1 = trans.transform.translation.x
                y1 = trans.transform.translation.y
                z1 = trans.transform.translation.z
                a = np.array([x1, y1, z1])
                print "first roop"

            x2 = trans.transform.translation.x
            y2 = trans.transform.translation.y
            z2 = trans.transform.translation.z
            b = np.array([x2, y2, z2])


            count = 1

            print "========================================"
            print trans.transform.translation.x

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                            tf2_ros.ExtrapolationException):
               rospy.logwarn('tf not found')

        rate.sleep()
