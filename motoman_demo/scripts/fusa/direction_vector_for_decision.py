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

import fusa_func as fuf

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'right_gripper'

GRIPPER_FRAME = 'right_gripper_link'

GRIPPER_OPEN = [0.03]
GRIPPER_CLOSED = [-0.02]
GRIPPER_NEUTRAL = [0.01]

GRIPPER_JOINT_NAMES = ['right_gripper_finger_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'world'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_demo')

        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)

        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)

        # Create a dictionary to hold object colors
        self.colors = dict()

        # Initialize the move group for the right arm
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)
        right_arm.set_planner_id('RRTConnectkConfigDefault')

        # Initialize the move group for the right gripper
        right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()

        # Allow some leeway in position (meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.05)
        right_arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)

        # Set the right arm reference frame
        right_arm.set_pose_reference_frame(REFERENCE_FRAME)

        # Allow 5 seconds per planning attempt
        right_arm.set_planning_time(5)

        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 5

        # Set a limit on the number of place attempts
        max_place_attempts = 5

        # Give the scene a chance to catch up
        rospy.sleep(2)

        # Give each of the scene objects a unique name
        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'
        target_id = 'target'
        tool_id = 'tool'

        # Remove leftover objects from a previous run
        scene.remove_world_object(table_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)
        scene.remove_world_object(target_id)
        scene.remove_world_object(tool_id)

        # Remove any attached objects from a previous session
        scene.remove_attached_object(GRIPPER_FRAME, target_id)

        # Give the scene a chance to catch up
        rospy.sleep(1)

        # Start the arm in the "resting" pose stored in the SRDF file
        right_arm.set_named_target('default')
        right_arm.go()

#----------------gripper 向き変更----------------------------------------------
        print "-----------  change gripper pose------------"
        fuf.printHello('world')
        # target_pose = PoseStamped()
        # target_pose.header.frame_id =  REFERENCE_FRAME
        # target_pose.pose.position.x = 0.3
        # target_pose.pose.position.y = 0
        # target_pose.pose.position.z = 0.2
        # target_pose.pose.orientation.w = 1.0
        # rolll = 0
        # pitch = 0
        # yaw = math.pi
        # tar_q = tf.transformations.quaternion_from_euler(rolll, pitch, yaw)
        # target_pose.pose.orientation.x = tar_q[0]
        # target_pose.pose.orientation.y = tar_q[1]
        # target_pose.pose.orientation.z = tar_q[2]
        # target_pose.pose.orientation.w = tar_q[3]
        # # Set the start state to the current state
        # right_arm.set_start_state_to_current_state()
        # # Set the goal pose of the end effector to the stored pose
        # right_arm.set_pose_target(target_pose, end_effector_link)
        # right_arm.go()

#--------------------------------------------------------------
        # Open the gripper to the neutral position
        right_gripper.set_joint_value_target(GRIPPER_OPEN)
        right_gripper.go()

        rospy.sleep(1)

        # Set the height of the table off the ground
        table_ground = 0.2

        # Set the dimensions of the scene objects [l, w, h]
        table_size = [0.5, 0.7, 0.01]
        box1_size = [0.1, 0.05, 0.05]
        box2_size = [0.05, 0.05, 0.15]

        # Set the target size [l, w, h]
        target_size = [0.04, 0.03, 0.12] #0.04, 0.03, 0.12  #0.02, 0.02, 0.02

        # Add a table top and two boxes to the scene
        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = 0.4 +0.1
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)

        box1_pose = PoseStamped()
        box1_pose.header.frame_id = REFERENCE_FRAME
        box1_pose.pose.position.x = 0.36 + 0.1
        box1_pose.pose.position.y = -0.1
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0
#------------------   box の向きを変える　　ｚ座標は調整する必要あり-------------------------------------------------------------------
        rolll = 0
        pitch = math.pi
        yaw = 0
        tar_q = tf.transformations.quaternion_from_euler(rolll, pitch, yaw)
        box1_pose.pose.orientation.x = tar_q[0]
        box1_pose.pose.orientation.y = tar_q[1]
        box1_pose.pose.orientation.z = tar_q[2]
        box1_pose.pose.orientation.w = tar_q[3]
        # box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[0] / 2.0
        # print "---------------------------------------------------------"
        # print box1_pose
#------------------------------------------------------------------------------------------------
        # scene.add_box(box1_id, box1_pose, box1_size)

        box2_pose = PoseStamped()
        box2_pose.header.frame_id = REFERENCE_FRAME
        box2_pose.pose.position.x = 0.34 + 0.1
        box2_pose.pose.position.y = 0.13
        box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
        box2_pose.pose.orientation.w = 1.0
        # scene.add_box(box2_id, box2_pose, box2_size)

        # Set the target pose in between the boxes and on the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.47 -0.05
        target_pose.pose.position.y = 0.0  + 0.1
        target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        target_roll = 0
        target_pitch = 0
        target_yaw = 0
        target_q = tf.transformations.quaternion_from_euler(target_roll, target_pitch, target_yaw)
        target_pose.pose.orientation.x = target_q[0]
        target_pose.pose.orientation.y = target_q[1]
        target_pose.pose.orientation.z = target_q[2]
        target_pose.pose.orientation.w = target_q[3]
        # Add the target object to the scene
        scene.add_box(target_id, target_pose, target_size)

        # Make the table red and the boxes orange
        self.setColor(table_id, 0.4, 0.4, 0.4, 1.0)
        self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_id, 0.8, 0.4, 0, 1.0)
        # Make the target yellow
        self.setColor(target_id, 0.9, 0.9, 0, 1.0)
        # Send the colors to the planning scene
        self.sendColors()

        # Set the support surface name to the table object 衝突を無視って言ってるけど、テーブルにアームが当たる状態だと無理なんだけど
        right_arm.set_support_surface_name(table_id)

        # Specify a pose to place the target after being picked up
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x = 0.33 + 0.1
        place_pose.pose.position.y = -0.18
        place_pose.pose.position.z = table_ground + table_size[2] + target_size[1] / 2.0 +0.1
        place_roll =  math.pi/2 #math.pi#math.pi/2
        place_pitch =  0
        place_yaw = 0
        pla_q = tf.transformations.quaternion_from_euler(place_roll, place_pitch, place_yaw)
        place_pose.pose.orientation.x = pla_q[0]
        place_pose.pose.orientation.y = pla_q[1]
        place_pose.pose.orientation.z = pla_q[2]
        place_pose.pose.orientation.w = pla_q[3]


#--------------- target_pose, place_poseから、つかめる grasp を厳選する --------------------------------------------
        #物体のgoal - startの各角度　で　変換角度をもとめる
        trans_roll = place_roll - target_roll
        trans_pitch = place_pitch - target_pitch
        trans_yaw = place_yaw - target_yaw
        print "Rt rpy  = %f, %f, %f" %(trans_roll, trans_pitch, trans_yaw)
        #Rt = tf.transformations.euler_matrix(trans_roll, trans_pitch, trans_yaw,'rzyx')
        # Rt = tf.transformations.euler_matrix(trans_roll, trans_pitch, trans_yaw,'rzyx')

        grasp_pose = PoseStamped()
        ok_pose_idlist = []
        for num in range(1,7):
            grasp_pose, grasp_roll, grasp_pitch, grasp_yaw = eval('fuf.grasp_pose_'+str(num))(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z)
            v, w = fuf.rotation_direc_vector(trans_roll, trans_pitch, trans_yaw, grasp_roll, grasp_pitch, grasp_yaw, 'sxyz')

            print "---v--"
            print v
            print "---w--"
            print w
            buf = fuf.extraction_direc_vector(num,v,w)
            if buf is not 0:
                ok_pose_idlist.append(buf)

            print "ok pose id  = " + str(ok_pose_idlist)
        #     ok_pose_idlist.append(fuf.extraction_direc_vector(num, v, w))
        #     print ok_pose_idlist
        #
        # ok_pose_re_0 = []
        # for i in ok_pose_idlist:
        #     if i not in ok_pose_re_0:
        #         ok_pose_re_0.append(i)
        # #ok_pose_re_0.remove(0)   #0が残ってるとき
        # print "OK pose list " + str(ok_pose_re_0)  #place での物体の姿勢を達成できるハンド姿勢のID





#--------------- 距離、角度から最短のgraspを決める (初期姿勢からpick_poseまで)----------------------------------------
        # アーム初期位置でのハンドの位置
        init_pose = PoseStamped()
        init_pose = right_arm.get_current_pose() #.pose
        init_pose_quat = (init_pose.pose.orientation.x,
                          init_pose.pose.orientation.y,
                          init_pose.pose.orientation.z,
                          init_pose.pose.orientation.w,)
        (init_roll,init_pitch,init_yaw) = tf.transformations.euler_from_quaternion(init_pose_quat)


        #-------距離のmaxとminを求めていくで---------
        grasp_pose = PoseStamped()
        dist_list_init_pick = [] #dist_min  dist_maxのため
        dist_list_pick_place = []
        for num in range(1,6):  #grasp_pose_6はIKが求まらんこと多いから、どかしてみる　=１〜６　　ホントは(1,7)
            grasp_pose, grasp_roll, grasp_pitch, grasp_yaw = eval('fuf.grasp_pose_'+str(num))(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z)
            dist_init_pick = fuf.calc_dist(init_pose.pose.position.x, init_pose.pose.position.y, init_pose.pose.position.z,
                                           grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z)
            dist_list_init_pick.append(dist_init_pick)

            dist_pick_place = fuf.calc_dist(grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z,
                                                place_pose.pose.position.x, place_pose.pose.position.y, place_pose.pose.position.z)
            dist_list_pick_place.append(dist_pick_place)



        print dist_list_init_pick
        dist_list_init_pick.sort()
        print dist_list_init_pick
        #print "dist_list_init_pick[0] = %f" % dist_list_init_pick[0]
        #print "dist_list_init_pick[6] = %f" % dist_list_init_pick[4]
        dist_min_init_pick = dist_list_init_pick[0]
        dist_max_init_pick = dist_list_init_pick[-1]

        dist_list_pick_place.sort()
        dist_min_pick_place = dist_list_pick_place[0]
        dist_max_pick_place = dist_list_pick_place[-1]

        #-------求めた距離max,minを使って、評価値を計算---------
        E_list = []
        for num in range(1,6):
            #print "num = %d" % num
            grasp_pose, grasp_roll, grasp_pitch, grasp_yaw = eval('fuf.grasp_pose_'+str(num))(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z)
            dist_init_pick = fuf.calc_dist(init_pose.pose.position.x, init_pose.pose.position.y, init_pose.pose.position.z,
                                           grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z)
            e1_dist = fuf.calc_e_dist(dist_init_pick, dist_min_init_pick, dist_max_init_pick )

            init_deg_list = fuf.find_IK(init_pose, end_effector_link, right_arm)  #７軸の角度が入ってる (deg)
            pick_deg_list = fuf.find_IK(grasp_pose, end_effector_link, right_arm)
            e1_deg = fuf.calc_e_deg7(init_deg_list, pick_deg_list)

            # p = False
            # while not p:  #ikが見つかるまで、繰り返す
            #     p = right_arm.set_joint_value_target(grasp_pose, end_effector_link)
            # print "joint value  from ik"
            # print p
            # print right_arm.get_joint_value_target()


            #e1_deg = fuf.calc_e_deg(init_roll,init_pitch,init_yaw,grasp_roll, grasp_pitch, grasp_yaw) #rpyを全部足しこんだもの
            E1 = 0.3*e1_dist + 0.7*e1_deg
            print "e1_dist =  %f" %e1_dist
            print "e1_deg  =  %f" %e1_deg
            print "E1  = %f" %E1

            dist_pick_place = fuf.calc_dist(grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z,
                                            place_pose.pose.position.x, place_pose.pose.position.y, place_pose.pose.position.z)
            e2_dist = fuf.calc_e_dist(dist_pick_place, dist_min_pick_place, dist_max_pick_place)

            place_deg_list = fuf.find_IK(place_pose, end_effector_link, right_arm)
            e2_deg = fuf.calc_e_deg7(pick_deg_list, place_deg_list)

            E2 = 0.3*e2_dist + 0.7*e2_deg
            print "e2_dist =  %f" %e2_dist
            print "e2_deg  =  %f" %e2_deg
            print "E2  = %f" %E2

            E = E1 + E2

            tp =(num, E) #タプルをつくってリストにぶっこむ
            E_list.append(tp)
            print "E = %f" % E
            print E_list

        E_list = sorted(E_list, key = lambda x:x[1], reverse = True)
        print "----------  E_list -----------"
        print E_list
        print E_list[0][0]

        choosed_pose_list = []   #つかめる姿勢かつ評価がいい順にならぶ　入るのはposeのid
        #choosed_pose_list = fuf.choose_pose(ok_pose_re_0,E_list)
        choosed_pose_list = fuf.choose_pose(ok_pose_idlist,E_list)

        print "choosed list "
        print choosed_pose_list

        grasp_pose, grasp_roll, grasp_pitch, grasp_yaw = eval('fuf.grasp_pose_'+str(choosed_pose_list[0]))(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z)

        #grasp_pose, grasp_roll, grasp_pitch, grasp_yaw = eval('fuf.grasp_pose_'+str(E_list[0][0]))(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z)
        #grasp_pose, grasp_roll, grasp_pitch, grasp_yaw = fuf.grasp_pose_4(target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z)

        # right_arm.set_pose_target(target_pose, end_effector_link)
        # right_arm.go()
        # print "===== set pose target ======="
        # print " get cuurent joint "
        # print right_arm.get_current_joint_values()
        # print " get joint value target "
        # print right_arm.get_joint_value_target()
        # rospy.sleep(3)


        # right_arm.set_joint_value_target(target_pose, end_effector_link)
        # right_arm.go()
        # print "===== set joint value target  ======="
        # print " get cuurent joint "
        # print right_arm.get_current_joint_values()
        # print " get joint value target "
        # print right_arm.get_joint_value_target()

        #joint_positions = [-0.2109587991228755, 0.6748677438425055, -0.37672985083633426, 0.6108336739791288, -0.23917114662678646, -1.6158254911883256, 0.5652131601423924]
        #joint_positions = [-0.13796497631186252, 0.4428416445747007, -0.3637849503885936, 0.2747198389029563, -0.1546085992064018, -1.4299920016570258, 0.4911570240154483]

        # right_arm.set_joint_value_target(joint_positions)
        # right_arm.go()
        # print " ======= arm gone ======="
        # print right_arm.get_joint_value_target()
        # print right_arm.get_current_joint_values()
        # rospy.sleep(3)



        # Shift the grasp pose by half the width of the target to center it  真ん中でつかむため
        #grasp_pose.pose.position.y -= target_size[1] / 2.0
        print "---------- 	execute_grasp_pose ---------------"



#---------------------------------------------------------------------------------

        # Generate a list of grasps
        grasps = self.make_grasps(grasp_pose, [target_id])

        # Publish the grasp poses so they can be viewed in RViz
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)

        # Track success/failure and number of attempts for pick operation
        result = None
        n_attempts = 0

        # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = right_arm.pick(target_id, grasps)
            rospy.sleep(0.2)

        # If the pick was successful, attempt the place operation
        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0

            # Generate valid place poses
            places = self.make_places(place_pose)

            # Repeat until we succeed or run out of attempts
            while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                for place in places:
                    result = right_arm.place(target_id, place)
                    if result == MoveItErrorCodes.SUCCESS:
                        break
                rospy.sleep(0.2)

            if result != MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")

        # Return the arm to the "resting" pose stored in the SRDF file
        right_arm.set_named_target('default')
        right_arm.go()

        # Open the gripper to the neutral position
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        right_gripper.go()

        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)

    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()

        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES

        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()

        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions

        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT

        tp.time_from_start = rospy.Duration(1.0)

        # Append the goal point to the trajectory points
        t.points.append(tp)

        # Return the joint trajectory
        return t

    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()

        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME

        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired

        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()

        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)

        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])

        # Set the first grasp pose to the input pose  ここクオータニオン
        g.grasp_pose = initial_pose_stamped

        #まずクオータニオンをオイラーにするやで
        quat = (g.grasp_pose.pose.orientation.x,
                g.grasp_pose.pose.orientation.y,
                g.grasp_pose.pose.orientation.z,
                g.grasp_pose.pose.orientation.w,)
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quat)

        # Pitch angles to try 受けたっとposeに足す形にすればいいんちゃうか  ここオイラー
        pitch_vals = [pitch, pitch+0.1, pitch-0.1, pitch+0.2, pitch-0.2, pitch+0.3, pitch-0.3]
        # Yaw angles to try
        yaw_vals = [yaw, yaw+0.1, yaw-0.1, yaw+0.2, yaw-0.2, yaw+0.3, yaw-0.3]
        # roll angles to try
        roll_vals = [roll, roll+0.1, roll-0.1, roll+0.2, roll-0.2, roll+0.3, roll-0.3]

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for y in yaw_vals:
            for p in pitch_vals:
                #for r in roll_vals:
                    # Create a quaternion from the Euler angles (roll, pitch, yaw)
                    q = quaternion_from_euler(roll, p, y)  # r or roll

                    # Set the grasp pose orientation accordingly
                    g.grasp_pose.pose.orientation.x = q[0]
                    g.grasp_pose.pose.orientation.y = q[1]
                    g.grasp_pose.pose.orientation.z = q[2]
                    g.grasp_pose.pose.orientation.w = q[3]

                    # Set and id for this grasp (simply needs to be unique)
                    g.id = str(len(grasps))

                    # Set the allowed touch objects to the input list
                    g.allowed_touch_objects = allowed_touch_objects

                    # Don't restrict contact force
                    g.max_contact_force = 0

                    # Degrade grasp quality for increasing pitch angles
                    g.grasp_quality = 1.0 - abs(p)

                    # Append the grasp to the list
                    grasps.append(deepcopy(g))

        # Return the list
        return grasps

    # Generate a list of possible place poses  place_poseを受け取る
    def make_places(self, init_pose):
        # Initialize the place location as a PoseStamped message  Grasp()をつかってない、物体の姿勢だけ
        place = PoseStamped()

        # Start with the input place pose ここクオータニオン
        place = init_pose

        # A list of x shifts (meters) to try
        x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        # A list of y shifts (meters) to try
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]

        # ここオイラー
        quat = (place.pose.orientation.x,
                place.pose.orientation.y,
                place.pose.orientation.z,
                place.pose.orientation.w,)
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quat)
        pitch_vals = [pitch, pitch+0.1, pitch-0.1, pitch+0.2, pitch-0.2, pitch+0.3, pitch-0.3]
        # A list of yaw angles to try
        yaw_vals = [yaw, yaw+0.1, yaw-0.1, yaw+0.2, yaw-0.2, yaw+0.3, yaw-0.3]
        roll_vals = [roll, roll+0.1, roll-0.1, roll+0.2, roll-0.2, roll+0.3, roll-0.3]

        # A list to hold the places
        places = []

        # Generate a place pose for each angle and translation
        for ya in yaw_vals:
            for p in pitch_vals:
                for r in roll_vals:
                    for y in y_vals:
                        for x in x_vals:
                            place.pose.position.x = init_pose.pose.position.x + x
                            place.pose.position.y = init_pose.pose.position.y + y

                            # Create a quaternion from the Euler angles
                            q = quaternion_from_euler(r, p, ya) #r or roll

                            # Set the place pose orientation accordingly
                            place.pose.orientation.x = q[0]
                            place.pose.orientation.y = q[1]
                            place.pose.orientation.z = q[2]
                            place.pose.orientation.w = q[3]

                            # Append this place pose to the list
                            places.append(deepcopy(place))

        # Return the list
        return places

    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()

        # Set the id to the name given as an argument
        color.id = name

        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff
        p.is_diff = True

        # Append the colors from the global color dictionary
        for color in self.colors.values():
            p.object_colors.append(color)

        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    MoveItDemo()
