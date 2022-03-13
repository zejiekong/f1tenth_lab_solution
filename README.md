# F1Tenth Lab

## Requirements
* ROS Noetic
* Ubuntu 20.04
* git

## Dependecies
* official F1Tenth simulator

## Setup

1.) Create catkin workspace <br>
```
mkdir f1tenth_ws 
cd f1tenth_ws
mkdir src
catkin_make
```

2.) Clone official F1Tenth simulator <br>
```
cd src
git clone https://github.com/f1tenth/f1tenth_simulator.git
```

3.) Clone Github repositories and build <br>
```
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