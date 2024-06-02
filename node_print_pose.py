#! /usr/bin/env python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

#Include the necessary libraries 
import rospy
import sys
import copy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


class MyRobot:

    # Defaut Constructor
    def __init__(self):
        #initialize the rospy node
        rospy.init_node('node_print_pose', anonymous=True)

        #Instantiate a RobotCommander object. This object is the outer-level interface to the robot
        self._robot = moveit_commander.RobotCommander()
        #Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
        self._scene = moveit_commander.PlanningSceneInterface()
        
        #define the movegoup for the robotic 
        #Replace this value with your robots planning group name that you had set in Movit Setup Assistant
        self._planning_group = "arm_group"
        #Instantiate a MoveGroupCommander Object. This Object is an interface to the one group of joints. this interface can be used to plan and execute the motions on the robotic arm
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group._g.start_state_monitor(1.0)  # wait = 1.0
        
        #We create a DisplayTrajectory publisher which is used later to publish trajectories for RViz to visualize
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        
         #Get the planning frame, end effector link and the robot group names
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        #print the info
        #here the '\033[95m' represents the standard colour "LightMagenta" in terminals. For details, refer: https://pkg.go.dev/github.com/whitedevops/colors
        #The '\033[0m' is added at the end of string to reset the terminal colours to default
        rospy.loginfo('\033[95m' + "Planning Frame: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def print_end_effector_pose(self):
        #for moveit_commander member functions in Python 3 (For Noetic), please refer: https://docs.ros.org/en/noetic/api/moveit_commander/html/functions_func.html
        #for moveit_commander member functions in Python 2, please refer(For Kinetic or Melodic): https://docs.ros.org/en/kinetic/api/moveit_commander/html/functions_func.html
        #Python file with function definitions: https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
        #Python file with function definitions (for Noetic): https://docs.ros.org/en/noetic/api/moveit_commander/html/move__group_8py_source.html
        #Python file with function definitions (for Kinetic or Melodic): https://docs.ros.org/en/kinetic/api/moveit_commander/html/move__group_8py_source.html
        
        #Get the current position of end effector link
        pose_values = self._group.get_current_pose(end_effector_link = self._eef_link).pose
        
        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        #Store the quaternion position values in list
        quaternion_list = [q_x, q_y, q_z, q_w]
        #Covert the quaternion values to roll, pitch and yaw
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
        
        #transformations documentation (For Noetic): https://docs.ros.org/en/noetic/api/tf2_ros/html/python/
        #transformations documentation (For Kinetic or Melodic): https://docs.ros.org/en/melodic/api/tf/html/python/transformations.html
        
        #Print the values
        rospy.loginfo('\033[32m' + 
                                "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) + 
                                "x: {}\n".format(pose_values.position.x) +  "y: {}\n".format(pose_values.position.y) +    "z: {}\n\n".format(pose_values.position.z) + 
                                "roll: {}\n".format(roll) + "pitch: {}\n".format(pitch) + "yaw: {}\n".format(yaw) +
                                '\033[0m')

    def print_joint_angle_values(self):
        #get the values of all the joint of the arm
        list_joint_values = self._group.get_current_joint_values()
        
         #Print the values
        rospy.loginfo('\033[32m' + "\nArm Joint Values: \n\n" +
                      "base_joint: {}\n".format(math.degrees(list_joint_values[0])) +
                      "joint_1: {}\n".format(math.degrees(list_joint_values[1])) +
                      "joint_2: {}\n".format(math.degrees(list_joint_values[2])) +
                      "joint_3: {}\n".format(math.degrees(list_joint_values[3])) +
                      "joint_4 {}\n".format(math.degrees(list_joint_values[4])) +
                      '\033[0m')

    # Class Destructor
    def __del__(self):
        #When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')


def main():
    
    #Create a new arm object from the MyRobot class
    arm = MyRobot()

    while not rospy.is_shutdown():
        arm.print_end_effector_pose()
        arm.print_joint_angle_values()
        rospy.sleep(1)

    del arm


if __name__ == '__main__':
    main()


