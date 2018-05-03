# baxter-klampt-motion
Integrate Klamp’t motion planning library with a physical Baxter through ROS, then implement code to compare motion and control algorithms to complete a “pick and place” task.

## Setup
In order to run this code, you need to have [ROS](http://wiki.ros.org/kinetic/Installation) and [klamp't](http://motion.pratt.duke.edu/klampt/tutorial_install.html) installed. This repo is a package in your ROS catkin workspace.

## Run
There are 3 main files in this repo:
* scripts/baxter_klampt_interface.py: this is a convenience class that translates between baxter_interface of the Baxter and Klamp't library. The class includes many functions but mainly a function to plan a path and a function to command a physical baxter
* scripts/baxter_stack.py: this script uses the class above (baxter_klampt_interface). It mainly sets the parameters for the baxter and the motion planning library.
* resources/dataTest.json: this file contains hardcoded configurations for the library to plan and the robot to follow
