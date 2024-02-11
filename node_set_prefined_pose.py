#! /usr/bin/python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

#Include the necessary libraries 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi
    
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MyRobot:

    # Default Constructor
    def __init__(self, Group_Name):

        #Initialize the moveit_commander and rospy node
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_predefined_pose', anonymous=True)

        
        #Instantiate a RobotCommander object. This object is the outer-level interface to the robot
        self._robot = moveit_commander.RobotCommander()
        #Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
        self._scene = moveit_commander.PlanningSceneInterface()
        
        #define the movegoup for the robotic 
        #Move group name is taken as input when initializing an object from the MyRobot Class
        self._planning_group = Group_Name
        #Instantiate a MoveGroupCommander Object. This Object is an interface to the one group of joints. this interface can be used to plan and execute the motions on the robotic arm
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        
        #Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        #Create action client for the "Execute Trajectory" action server
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        #Get the planning frame, end effector link and the robot group names
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        #print the info
        #here the '\033[95m' represents the standard colour "LightMagenta" in terminals. For details, refer: https://pkg.go.dev/github.com/whitedevops/colors
        #The '\033[0m' is added at the end of string to reset the terminal colours to default
        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        
        #for moveit_commander member functions in Python 3 (For Noetic), please refer: https://docs.ros.org/en/noetic/api/moveit_commander/html/functions_func.html
        #for moveit_commander member functions in Python 2, please refer(For Kinetic or Melodic): https://docs.ros.org/en/kinetic/api/moveit_commander/html/functions_func.html
        #Python file with function definitions: https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
        #Python file with function definitions (for Noetic): https://docs.ros.org/en/noetic/api/moveit_commander/html/move__group_8py_source.html
        #Python file with function definitions (for Kinetic or Melodic): https://docs.ros.org/en/kinetic/api/moveit_commander/html/move__group_8py_source.html

        #Set the predefined position as the named joint configuration as the goal to plan for the move group. The predefined positions are defined in the Moveit Setup Assistant
        self._group.set_named_target(arg_pose_name)
        
        #Plan to the desired joint-space goal using the default planner
        #The below line of code is different for melodic and noetic
        plan_success, plan, planning_time, error_code = self._group.plan()
        
        #Create a goal message object for the action server
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        #Update the trajectory in the goal message
        goal.trajectory = plan
        #Send the goal to the action server
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        #Print the current pose
        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    

    # Class Destructor
    def __del__(self):
        #When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')


def main():

    #Create a new arm object from the MyRobot class
    arm = MyRobot("arm_group")
    hand =  MyRobot("hand")
    
    #call the function to set the position to "zero_pose"
    arm.set_pose("zero_pose")
    #Wait for 2 seconds
    rospy.sleep(1)
    
    arm.set_pose("straight_up")
    rospy.sleep(1)
    
    #Open the gripper or end effector
    hand.set_pose("hand_opened")
    rospy.sleep(1)

    #Here, we will repeat the cycle of setting to various positions, simulating the pick and place action
    while not rospy.is_shutdown():
    
    	#Go to Lift Object Pose
        arm.set_pose("lift_object")
        rospy.sleep(1)
        
        #Go to Pick Object Pose
        arm.set_pose("pick_object")
        rospy.sleep(1)
        
        #Close the gripper or end effector
        hand.set_pose("hand_closed")
        rospy.sleep(1)       
        
        #Go to drop Object Pose
        arm.set_pose("drop_object")
        rospy.sleep(1)
        
        #Open the gripper or end effector
        hand.set_pose("hand_opened")
        rospy.sleep(1)        

    #delete the arm object at the end of code
    del arm
    del hand
	


if __name__ == '__main__':
    main()



