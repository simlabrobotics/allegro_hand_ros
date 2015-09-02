allegro_hand_ros beta2, unofficial fork
=======================================

This is an unofficial fork of SimLab's allegro [hand ros package][1].

It provides the same code in a catkin-ized version, merges all of the launch
files into two, updates some of the package/node names to have a more consistent
structure, and improves the build process by creating a common library.

It also provides the BHand library directly in this package (including both
32-bit and 64-bit versions, though 32-bit systems will need to update the
symlink manually).

Non-compatible changes between the two version are:

 - Put all of the controllers into one *package* (allegro_hand_controllers) and
   made each controller a different node (allegro_node_XXXX): grasp, pd, velsat.
 - Single launch file with arguments instead of multiple launch files with
   repeated code.
 - Both the parameter and description files are now ROS packages, so that
   `rospack find` works with them.
 - These packages will likely not work with pre-hydro versions (only tested on
   indigo so far, please let me know if this works on other distributions).

There are probably many more.

Launch file instructions:
------------------------

    roslaunch allegro_hand_controllers allegro_hand.launch HAND:=right

Optional (recommended) arguments:

          ZEROS:=/path/to/zeros_file.yaml
          CONTROLLER:=grasp|pd|velsat
          RESPAWN:=true|false   Respawn controller if it dies.
          KEYBOARD:=true|false  (default is true)
          CAN_DEVICE:=/dev/pcanusb1 | /dev/pcanusbNNN  (ls -l /dev/pcan* to see open CAN devices)
          VISUALIZE:=true|false  (Launch rviz)

The second launch file is for visualization, it is included in
`allegro_hand.launch` if `VISUALIZE:=true`. Otherwise, it can be useful to run
it separately (with `VISUALIZE:=false`) if you want to start rviz separately
(and keep it running):

    roslaunch allegro_hand_controllers allegro_viz.launch HAND:=right

[1]: https://github.com/simlabrobotics/allegro_hand_ros

Below is the original Simlab README (some sections that are no longer useful removed).


----------


This ROS Stack includes code and tools useful to users of SimLab Co., Ltd.'s
Allegro Hand.

**Note:** This stack works with Allegro Hands v3.0 and up by befault.
See the end of this document for how to use this package with an older Allegro Hand.

For more information visit SimLab's [Allegro Hand wiki](http://www.simlab.co.kr/AllegroHand/wiki).
ROS specific information can be found on the [ROS wiki](http://www.ros.org/wiki/allegro_hand_ros).

**Announcement:** The next major release of this stack will include the integration of certain sensor systems

**Note:** If controlling multiple allegro hands at once, please see the appropriate section below.


Contents
--------
* **allegro_hand_common:** Contains files common to multiple Allegro Hand controllers like CAN communication code.
  **Note:** libBHand (SimLab/KITECH's Allegro Hand Grasping Library) has been moved as a separate library available in SimLab's [Allegro Hand wiki](http://www.simlab.co.kr/AllegroHand/wiki).

* **allegro_hand_controllers:** Contains example controllers.
    * PD Joint Space control
    * Velocity Saturation Joint Space Control
    * Grasping Library Interface

   Please refer to PD Joint Space control folder for easily testing your own control code.

  **Note:** The PD and Grasp controllers, along with the Velocity Saturation controller, include two control loop timing examples each. One method uses a timer interrupt while the other loops. Please research the benefits and downfalls of each of these on ROS.org.

  **Note:** The prefered sampling method is utilizing the Hand's own real time clock running @ 333Hz by polling the CAN communication. In fact, ROS's inturrupt/sleep combination might cause instability in CAN communication resulting unstable hand motions. The solution is to have the main control loop polling the readDevice(). readDevice() will return when 16 encoder values are availbe in the CAN bus. Allegro Hand automatically writes 16 encoder values in every 1/333 sec. Notice that this polling method provides the stable sampling rate of 333 Hz!!!

* **allegro_hand_keyboard:** Contains code for the keyboard node used to command different grasps. All commands are available when running the allegro_hand_core_grasp and only few commands are available when running allegro_hand_core_pd or running allegro_hand_core_pd controllers.

* **allegro_hand_description:** Contains the robot description URDF for the Allegro Hand. Currently only the right hand is complete.


* **parameters:**
 * **gains_pd.yaml:** Contains PD gains used in allegro_hand_core_pd and allegro_hand_core_pd_slp.
 * **gains_velSat.yaml:** Contains PD gains and velocity limits used in allegro_hand_core_velSat
 * **initial_position.yaml:** Contains the initial position for the joints to got to when joint space controllers like *pd*, *pd_slp* and *velSat* are used. By default, this is the Allegro Hand *Home position*.
 * **zero.yaml:** Offsets and directions for each of the 16 joints (Read in by the CAN communication code) Also includes Allegro Hand info specific to each hand like version number and serial number.
 **Note:** The inclusion of "which_hand" (right/left) in this file has been deprecated. This must be specified as an argument when launching any of the allegro_hand* launchers.

  **Note:** If any of the gains files or the initial positions file are missing, the two joint space controllers (*pd* and *velSat*) will load default values sepcified in the srespective allegroNode.cpp file. If zero.yaml is not loaded, the Allegro Hand will shut down automatically.

Launchers
---------
  * **allegro_hand.launch:** Launches Allegro Hand with specified controller (default = grasp). Also launches keyboard controller and rviz visualizer.
  * **allegro_hand_noRviz.launch:** Launches Allegro Hand with specified controller (default = grasp). Also launches keyboard controller and DOES NOT launch rviz visualizer.
  * **allegro_hand_joint_gui.launch:** Launches Allegro Hand with PD controller by defauly. Also launches a GUI interface for controlling each joint within its limits and rviz visualizer.
  * **allegro_hand_joint_gui_virtual.launch:** Launches Allegro Hand kinematic model. Also launches a GUI interface for controlling each joint within its limits and rviz visualizer.

**Note:** All launch files (including virtual and actual hand) have the following arguments that can be specified:

  * **HAND:=** (Used to specify which hand is being controlled)
    * right
    * left
    * **Note:** There is no default. This arg must be specified.

  * **NUM:=** (Used to enumerate the hands when multiple will be controlled at once / Prevents naming and data conflicts)
    * 0 (default)
    * Any Integer (1, 2, 3, ... )

Also, the following arguments can be specified for actual hardware launch file:

  * **CONTROLLER:=** (Specify the controller / Grasp library or simple joint space controller)
    * grasp (default)
    * pd
    * velsat

  * **pollling:=** (Specify if CAN communication is done by polling)
    * true (default)
    * false

  * **ZEROS:=** (Specify the encoder/motor offsets and directions parameter file)
    * zero.yaml (default)
    * path to zero*.yaml file (ex. "parameters/zero_files/zero_SAH020CR020.yaml")

  * **CAN_CH:=** (Specify the CAN channel to which the hand is connected)
    * /dev/pcan32 (default)
    * another PEAK CAN Channel (see in /dev/*)



Controlling More Than One Hand
------------------------------

*When running more than one hand using ROS, you must specify the number of the hand when launching.

```
  roslaunch allegro_hand.launch HAND:=right ZEROS:=parameters/zero0.yaml NUM:=0 CAN_CH:=/dev/pcan0
  roslaunch allegro_hand.launch HAND:=left  ZEROS:=parameters/zero1.yaml NUM:=1 CAN_CH:=/dev/pcan1
```
