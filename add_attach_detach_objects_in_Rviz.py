#! /usr/bin/env python3

#Author https://www.youtube.com/@Age.of.Robotics

#The code in this script is based on Moveit Tutorials
#https://moveit.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html

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


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MyRobot:

    # Default Constructor
    def __init__(self, Group_Name,Grasping_Group_Name):

        #Initialize the moveit_commander
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        #https://moveit.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
        
        #Instantiate a RobotCommander object. This object is the outer-level interface to the robot
        self._robot = moveit_commander.RobotCommander()
        #https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html#a0a1cde6d736eeb08c07e995bda9d7671s
        
        #Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
        self._scene = moveit_commander.PlanningSceneInterface()
        #https://docs.ros.org/en/noetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1PlanningSceneInterface.html

        #define the movegoup for the robotic 
        #Move group name is taken as input when initializing an object from the MyRobot Class
        self._planning_group = Group_Name
        #Enf effector planning group name
        self._grasping_group = Grasping_Group_Name
        
        #Instantiate a MoveGroupCommander Object. This Object is an interface to the one group of joints. this interface can be used to plan and execute the motions on the robotic arm
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        #https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
        
        #Get the planning frame, end effector link and the robot group names
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        
        self.box_name = ""

        #print the info
        #here the '\033[95m' represents the standard colour "LightMagenta" in terminals. For details, refer: https://pkg.go.dev/github.com/whitedevops/colors
        #The '\033[0m' is added at the end of string to reset the terminal colours to default
        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')
        
    #Start of code for box
    def wait_for_state_update(self,name, box_is_known=False, box_is_attached=False, timeout=4):
       #box_name = self.box_name
       box_name = name
       scene = self._scene
       start = rospy.get_time()
       seconds = rospy.get_time()
       
       while (seconds - start < timeout) and not rospy.is_shutdown():
           # Test if the box is in attached objects
           attached_objects = scene.get_attached_objects([box_name])
           is_attached = len(attached_objects.keys()) > 0
           
           # Test if the box is in the scene.
           # Note that attaching the box will remove it from known_objects
           is_known = box_name in scene.get_known_object_names()
           # Test if we are in the expected state
           
           if (box_is_attached == is_attached) and (box_is_known == is_known):
              return True
           
           # Sleep so that we give other threads time on the processor
           rospy.sleep(0.1)
           seconds = rospy.get_time()
           
       # If we exited the while loop without returning then we timed out
       return False

    def add_box(self,name,x,y,z,sx,sy,sz, timeout=4):
	    
	    #box_name = self.box_name
	    box_name = name
	    scene = self._scene
	    
	    ## First, create an message object of type pose to deifne box positions
	    box_pose = geometry_msgs.msg.PoseStamped()
	    box_pose.header.frame_id = "world"
	    
	    #set the position massage arguments
	    box_pose.pose.position.x = x # above the panda_hand frame
	    box_pose.pose.position.y = y # above the panda_hand frame
	    box_pose.pose.position.z = z # above the panda_hand frame
	    box_pose.pose.orientation.x = 0
	    box_pose.pose.orientation.y = 0
	    box_pose.pose.orientation.x = 0
	    box_pose.pose.orientation.w = 1.0
	    #Add the box in the scean
	    scene.add_box(box_name, box_pose, size=(sx, sy, sz))
	    #https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
	    #https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html

	    #Set the box name for box_name of the class
	    self.box_name=box_name
	    #Check if the box is added sucessfully
	    return self.wait_for_state_update(name, box_is_known=True, timeout=timeout)
	    
    def attach_box(self,name, timeout=4):
	    #box_name = self.box_name
	    box_name = name
	    robot = self._robot
	    scene = self._scene
	    eef_link = self._eef_link

	    grasping_group = self._grasping_group
	    touch_links = robot.get_link_names(group=grasping_group)
	    rospy.loginfo('\033[95m' + "Touch Link: {}".format(touch_links) + '\033[0m')
	    scene.attach_box(eef_link, box_name, touch_links=touch_links)
	    #https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
	    #https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html

	    # We wait for the planning scene to update.
	    return self.wait_for_state_update(name,box_is_known=False, box_is_attached=True,  timeout=timeout)

    def detach_box(self,name, timeout=4):
	    #box_name = self.box_name
	    box_name = name
	    scene = self._scene
	    eef_link = self._eef_link
        
	    scene.remove_attached_object(eef_link, name=box_name)
	    #https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
	    #https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html
        
	    return self.wait_for_state_update(name, box_is_known=True, box_is_attached=False, timeout=timeout)
        
    def remove_box(self,name, timeout=4):
       #box_name = self.box_name
       box_name = name
       scene = self._scene
       scene.remove_world_object(box_name)
       #https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
       #https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html
       
       return self.wait_for_state_update(name, box_is_known=False, box_is_attached=False,  timeout=timeout)
            
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
            
def main():
    #Initialize a ROS Node
    rospy.init_node('add_attach_detach_objects_in_Rviz', anonymous=True)

    robotArm = MyRobot("arm_group", "hand") 
    
    while not rospy.is_shutdown():
        task = input("Enter add to add object, attch to attach object, detach to detach object and remove to remove object: ")
        
        if task == "add":
            response = robotArm.add_box("package$1",0.00,0.6318,0.015,0.03,0.03,0.03)
            if response == True:
                rospy.loginfo('\033[32m' + "Added Object in Rviz: {}".format(robotArm.box_name) + '\033[0m')
            
        if task == "attach":
            response = robotArm.attach_box("package$1")
            if response == True:
                rospy.loginfo('\033[32m' + "Attached Object in Rviz: {}".format(robotArm.box_name) + '\033[0m')
        if task == "detach":
            response = robotArm.detach_box("package$1")
            if response == True:
                rospy.loginfo('\033[32m' + "Detached Object from Rviz: {}".format(robotArm.box_name) + '\033[0m')
        if task == "remove":
            response = robotArm.remove_box("package$1")
            if response == True:
                rospy.loginfo('\033[32m' + "Removed Object from Rviz: {}".format(robotArm.box_name) + '\033[0m')
        if task == "x":
            break
        
    #rospy.spin()
    quit()
    #delete the robotArm object at the end of code
    del robotArm
    
if __name__ == '__main__':
    main()      
