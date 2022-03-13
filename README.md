# F1Tenth Lab

## Requirements
* ROS Noetic
* Ubuntu 20.04
* git

## Dependecies
* official f1tenth simulator -> step 2 in setup

## Setup

1.) Create catkin workspace <br>
```
mkdir f1tenth_ws 
cd f1tenth_ws
mkdir src
catkin_make
```

2.) Clone Github repositories and build <br>
```
cd src
git clone https://github.com/f1tenth/f1tenth_simulator.git
git clone https://github.com/zejiekong/f1tenth_lab_solution.git
cd ..
catkin_make
```

## Launch
``` 
roslaunch lab_<num> lab_<num>.launch
```

## Video 
https://youtube.com/playlist?list=PLhlMWo4pPNLsUVuUtq6kH3wVx0keUvklJ 