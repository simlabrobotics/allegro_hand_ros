allegro_hand_ros beta2
======================

This ROS Stack includes code and tools useful to users of SimLab Co., Ltd.'s Allegro Hand.<br>


**Note:** This stack works with Allegro Hands v2.0 and up by befault.
See the end of this document for how to use this package with an older Allegro Hand.


For more information visit SimLab's [Allegro Hand wiki](http://www.simlab.co.kr/AllegroHand/wiki).<br>
ROS specific information can be found on the [ROS wiki](http://www.ros.org/wiki/allegro_hand_ros).


**Announcement:** The next major release of this stack will include the integration of certain sensor systems


Contents
--------
* **allegro_hand_common:** Contains files common to multiple Allegro Hand controllers like CAN communication code and SimLab/KITECH's Allegro Hand Grasping Library.

* **allegro_hand_controllers:** Contains example controllers. 
    * PD Joint Space control
    * Velocity Saturation Joint Space Control
    * Grasping Library Interface
    * Etc
    
  This folder also contains templates for easily testing your own control code.
  
  **Note:** The PD and Grasp controllers, along with the controller templates, include two control loop timing examples each. One method uses a timer interrupt while the other uses a sleep function to eat up remaining control loop time. Please research the benefits and downfalls of each of these on ROS.org.
  
* **allegro_hand_keyboard:** Contains code for the keyboard node used to command different grasps. Only usefulwhen running the allegro_hand_core_grasp or allegro_hand_core_grasp_slp controllers.

* **allegro_hand_description:** Contains the robot description URDF for the Allegro Hand. Currently only the right hand is complete.
      
  
* **parameters:**
 * **gains_pd.yaml:** Contains PD gains used in allegro_hand_core_pd and allegro_hand_core_pd_slp.
 * **gains_velSat.yaml:** Contains PD gains and velocity limits used in allegro_hand_core_velSat
 * **initial_position.yaml:** Contains the initial position for the joints to got to when joint space controllers like *pd*, *pd_slp* and *velSat* are used. By default, this is the Allegro Hand *Home position*.
 * **zero.yaml:** Offsets and directions for each of the 16 joints. Read in by the CAN communication code. Also includes Allegro Hand info specific to each hand like version number and serial number.
  
  **Note:** If any of the gains files or the initial positions file are missing, the three jpint space controllers (*pd*, *pd_slp* and *velSat*) will load default values sepcified in the srespective allegroNode.cpp file. If zero.yaml is not loaded, the Allegro Hand will shut down automatically.
  
Launchers
---------
  * **allegro_hand.launch:** Launches Allegro Hand with specified controller. Also launches keyboard controller and rviz visualizer.
  * **allegro_hand_noRviz.launch:** Launches Allegro Hand with specified controller. Also launches keyboard controller and DOES NOT launch rviz visualizer.  
  * **allegro_hand_joint_gui.launch:** Launches Allegro Hand with specified controller. Also launches a GUI interface for controlling each joint within its limits and rviz visualizer.
  * **allegro_hand_joint_gui_virtual.launch:** Launches Allegro Hand with specified controller. Also launches a GUI interface for controlling each joint within its limits and rviz visualizer.
  
**Note:** All four (4) of the launch files have three arguments that can be specified
  
  * **CONTROLLER:=**
    * grasp (default)
    * grasp_slp
    * pd
    * pd_slp
    * velSat
    * template
    * template_slp
      
  * **HAND:=**
    * right (default)
    * left
      
  * **GROOVY:=**
    * false (default, used for ROS Fuerte)
    * true  
    

<br>
**Examples:**<br>
* To launch the default grasp controller for the right Allegro Hand using Fuerte you can use either of the following two commands:

```
roslaunch allegro_hand.launch CONTROLLER:=grasp HAND:=right GROOVY:=false
```
```
roslaunch allegro_hand.launch
```  
 
 <br> 
* For the PD controller on a right hand using Groovy:

```
roslaunch allegro_hand.launch CONTROLLER:=pd GROOVY:=true
```    
    
Thanks
------    
Please be advised, this is the first beta release of the ROS stack for SimLab's Allegro Hand platform. There will likely be bugs and there is much room for improvement. 

Please share you improvements to the code included in this release. Also, we would love to include the interesting controllers you may create as part of the package. Please keep an open line of contact as we continue to develop this software.

Thanks.

Alex Alspach <alexalspach@simlab.co.kr>



<br>
 
**Note:** As stated above, control loops utilizing both timer callbacks and ROS' Rate object are included. Generally, the timer callback is more reliable as it will run the control code in a new thread if the previous iteration fails to end within the given crol period. The rate object, aling with a spinOnce() and a sleep, generally does a fine job but some anomolies have been recognized where the loop toaks much too long and remains blocked until finishing. Using the rate/sleep method allows for simpler code.
 

For Allegro Hand v1.0
---------------------

TODO
