Allegro Hand Glove Control
======================
This is a fork of the original Allegro Hand ROS repository found here: https://github.com/simlabrobotics/allegro_hand_ros
This fork's purpose is to trim down as much functionality as possible and leave a bare human-robot interface to the Allegro Hand using a glove with sensors.

Contents
--------
* **arduino_code:** Contains the Arduino codes neccesary for this project.
    * Position Control
    * Calibration

* **allegro_hand_common:** Contains files common to multiple Allegro Hand controllers like CAN communication code.

* **allegro_hand_controllers:** Contains example controllers.
    * PD Joint Space control
    * Velocity Saturation Joint Space Control
    * Grasping Library Interface

* **allegro_hand_keyboard:** Contains code for the keyboard node used to command different grasps.

* **allegro_hand_description:** Contains the robot description URDF for the Allegro Hand. Currently only the right hand is complete.
 
* **parameters:**
 * **gains_pd.:** Contains PD gains used in allegro_hand_core_pd.
 * **gains_velSat:** Contains PD gains and velocity limits used in allegro_hand_core_velSat
 * **initial_position:** Contains the initial position for the joints to got to when joint space controllers like *pd* and *velSat* are used. By default, this is the Allegro Hand *Home position*.
 * **zero:** Offsets and directions for each of the 16 joints (Read in by the CAN communication code) Also includes Allegro Hand info specific to each hand like version number and serial number.

Launchers
---------
  * **allegro_hand.launch:** Launches Allegro Hand controller.
  * **allegro_hand_virtual.launch:** Launches Allegro Hand visualizer.
  
**Note:** The Allegro Hand controller launch file has the following arguments that can be specified:
      
  * **HAND:=** (Used to specify which hand is being controlled)
    * right (default)
    * left
    
  * **NUM:=** (Used to enumerate the hands when multiple will be controlled at once / Prevents naming and data conflicts)
    * 0 (default)
    * Any Integer (1, 2, 3, ... )   
      
  * **VIZ:=** (Enable vizualization)
    * false
    * true (default) 

  * **CONT:=** (Specify the controller)
    * grasp
    * pd (default)
    * velSat

  * **POLL:=** (Specify if CAN communication is done by polling)
    * true (default)
    * false
     
  * **ZEROS:=** (Specify the encoder/motor offsets and directions parameter file)
    * zero.yaml (default)
    * path to zero*.yaml file (ex. "parameters/zero_files/zero_SAH020CR020.yaml")
    
  * **CAN_CH:=** (Specify the CAN channel to which the hand is connected)
    * /dev/pcan32 (default)
    * another PEAK CAN Channel (see in /dev/*)

**Examples:**<br>
roslaunch allegro_hand.launch
roslaunch allegro_hand.launch CONT:=grasp HAND:=left VIZ:=false POLL:=false
