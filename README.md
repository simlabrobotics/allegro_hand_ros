allegro_hand_ros
================

RUNNING THE ALLEGRO HAND
=========================

Defaults:
roslaunch allegro_hand.launch CONTROLLER:=grasp HAND:=right GROOVY:=false


ROS stack for SimLab's Allegro Hand

This ROS Stack includes code and tools useful
to users of SimLab Co., Ltd.'s Allegro Hand.

Package Descriptions:
=========================

*AllegroHand [deprecated]
 - Currently contains an application that executes both CAN communication and Allegro Hand grasping library motions and grasps. This is currently being split into two seperate nodes: CAN communication & Grasping library communication


++ COMMUNICATION ++

AllegroHand_CAN
 - This is the CAN communication layer for any Allegro Hand controller. This node publishes current joint positions (radians) and subscribes to joint torque commands (N*m). Any controller designed for the hand should input and output these position and torque values, respectively.
 - Both messages are of type sensor_msgs/JointState.


++ CONTROLLERS ++	

AllegroHand_ctrl_graspLib
 - Interfaces with the Allegro Hand grasping library making available many torque controlled grasping modes such as finger tip grasps and full hand grasping. Also available it joint PD position control.
  - Includes a keyboard input handler for executing each grasp. grasps and PD control can also be executed by publishing to the std_msgs/String and sensor_msgs/JointState messages respectively.
	

++ TOOLS ++

AllegroHand_urdf
 - This package includes a kinematic model of the Allegro Hand. This model can be used to confirm the validity of joint positions commands before executing such commands on the Allegro Hand hardware.
 - Included launch files will also launch Allegro Hand control and communication nodes with output to put the the actual and virtual Allegro Hands.
 
Note:
 Control loops utilizing both timer callbacks and ROS' Rate object are included. Generally, the timer callback is more reliable as it will run the control code in a new thread if the previous iteration fails to end within the given crol period. The rate object, aling with a spinOnce() and a sleep, generally does a fine job but some anomolies have been recognized where the loop toaks much too long and remains blocked until finishing. Using the rate/sleep method allows for simpler code.
 
