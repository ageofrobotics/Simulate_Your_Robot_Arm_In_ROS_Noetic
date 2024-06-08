#! /usr/bin/env python3
#Author https://www.youtube.com/@Age.of.Robotics

#This code is modified based on the below package:
#https://github.com/pal-robotics/gazebo_ros_link_attacher

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
#https://github.com/pal-robotics/gazebo_ros_link_attacher

class RoboticGripper():

    # Constructor
    def __init__(self):
    
        self._Robot_Name = ""
        self._EE_Link_Name = ""
        
        self._Box_Model_Name = ""
        self._Box_Link_Name = ""
    
        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
        self._attach_service = rospy.ServiceProxy('/link_attacher_node/attach',Attach)
        self._attach_service.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        self._detach_service = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
        self._detach_service.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

        rospy.loginfo( '\033[94m' + " >>> RoboticGripper init done." + '\033[0m')

    def attach_Object(self):
      rospy.loginfo("Attach object request received")
      req = AttachRequest()
      req.model_name_1 = self._Robot_Name
      req.link_name_1 = self._EE_Link_Name
      req.model_name_2 = self._Box_Model_Name
      req.link_name_2 = self._Box_Link_Name
      self._attach_service.call(req)
      rospy.loginfo('\033[32m' + "Attached Object to End Effector in Gazebo".format(self._Box_Model_Name) + '\033[0m')

    
    
    def detach_Object(self):
      rospy.loginfo("Detach object request received")
      req = AttachRequest()
      req.model_name_1 = self._Robot_Name
      req.link_name_1 = self._EE_Link_Name
      req.model_name_2 = self._Box_Model_Name
      req.link_name_2 = self._Box_Link_Name
      self._detach_service.call(req)
      rospy.loginfo('\033[32m' + "Deatched Object from End Effector in Gazebo".format(self._Box_Model_Name) + '\033[0m')
      
    # Destructor
    def __del__(self):
        rospy.loginfo( '\033[94m' + " >>> RoboticGripper delete" + '\033[0m')

# Defining main function 
def main():

    #Initialize a ROS Node
    rospy.init_node('attach_detach_objects_in_Gazebo', anonymous=True)
    
    robot_arm_gripper = RoboticGripper()
    robot_arm_gripper._Robot_Name = "robot_arm_urdf"
    robot_arm_gripper._EE_Link_Name = "link_5"
    robot_arm_gripper._Box_Model_Name = "package$1"
    robot_arm_gripper._Box_Link_Name = "link"
    
    while not rospy.is_shutdown():
        task = input("Enter attch to attach object and detach to detach object: ")
        
        if task == "attach":
            robot_arm_gripper.attach_Object()
        elif task == "detach":
            robot_arm_gripper.detach_Object()
        elif task == "x":
            break
    
    #rospy.spin()
    quit()
    
    #delete the robotArm object at the end of code
    del robot_arm_gripper
        
if __name__=="__main__":
    main() 



