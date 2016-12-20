#!/usr/bin/env python

import sys
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
from std_msgs.msg import String
#
# def callback(message):
#     print message.pose.pose.position.x
#
# def main():
#     #======== init node ========#
#     rospy.init_node('ar_lisner')
#
#
#     sub = rospy.Subscriber('/ar_pose_marker',ARMarker,callback)
#
#     #======== moveit init ========#
#     robot = moveit_commander.RobotCommander()
#     arm = moveit_commander.MoveGroupCommander("arm")
#     arm_initial_pose = arm.get_current_pose().pose
#     target_pose = geometry_msgs.msg.Pose()
#     #arm.set_planner_id('RRTConnectkConfigDefault')
#
#     #======== tf lisner ========#
#     tf_buffer = tf2_ros.Buffer()
#     tf_listner = tf2_ros.TransformListener(tf_buffer)
#     get_tf_flg = False
#
#     count = 0
#
# #compute_cartesian_path
#     waypoints =[]
#     waypoints.append(arm.get_current_pose().pose)
#
#     wpose = geometry_msgs.msg.Pose()
#     wpose.position.x = waypoints[0].position.x
#     wpose.position.y = waypoints[0].position.y -0.15
#     wpose.position.z = waypoints[0].position.z
#     wpose.orientation.w = 1.0
#     waypoints.append(copy.deepcopy(wpose))
#
#     wpose.position.z -= 0.1
#     waypoints.append(copy.deepcopy(wpose))
#
#     (plan1, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0)
#     print "=" * 10, " plan4..."
#     print plan1
#     # arm.execute(plan)
#     # arm.set_pose_target(plan)
#     # arm.go()
#     rospy.sleep(5)
#
#
#
#
# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass
#


# ===================================================================






from std_msgs.msg import String

def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
    group = moveit_commander.MoveGroupCommander("arm")


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

    cartesian = rospy.get_param('~cartesian', True)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    rospy.sleep(10)
    print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
    print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"




  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints
  ## for the end-effector to go through.
    waypoints = []

  # start with the current pose
    waypoints.append(group.get_current_pose().pose)

  # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = waypoints[0].orientation.w
    wpose.position.x = waypoints[0].position.x + 0.1
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))

  # second move down
    wpose.position.z -= 0.10
    waypoints.append(copy.deepcopy(wpose))

  # third move to the side
    wpose.position.y += 0.05
    waypoints.append(copy.deepcopy(wpose))

  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
    (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold

    print "============ Waiting while RVIZ displays plan3..."
    print plan3
    rospy.sleep(3)
    print "============ Visualizing plan3"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan3)
    display_trajectory_publisher.publish(display_trajectory);
    rospy.sleep(5)


  ## Adding/Removing Objects and Attaching/Detaching Objects
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## First, we will define the collision object message
    collision_object = moveit_msgs.msg.CollisionObject()



  ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

    print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass