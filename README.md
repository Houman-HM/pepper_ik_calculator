# pepper_robot

##### This repository contains the packages for visualizing Softbank pepper as well as its inverse kinematic solver.

# Installation

Clone the repository into your catkin workspace.

``` 
git clone https://github.com/Houman-HM/pepper_robot.git
```
## Installing dependecies and building the packages.
``` 
cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y
cd ~/catkin_ws && catkin_make
```
### Launching Rviz visualizer along with associated move_groups.

From pepper_moveit_config_modified, launch the demo.launch file. 
``` 
roslaunch pepper_moveit_config_modified demo.launch
```
### Running the IK solver.
``` 
rosrun pepper_ik_calculator ik_calculator
```
###### Note: The IK solver sets the joints to some random positions and tries to calculate the IK for them. However, since the numbers are random, it might happen that it would not find the right solution. Thus, you might need to run the node couple times until a valid positon is set for the joints so that the IK solver be able to calculate the joint angles.
