# group5_rwa1
# Package for ENPM 663-RWA1. 

Authors:
Darshan Jain, Pulkit Mehta, Jeffin Kachappilly

This package requires:
* ROS Melodic
* Gazebo 9.15+
* ARIAC and its dependencies 
* C++
--

- Clone the package into the workspace and build the package using,
```
$ catkin build group5_rwa1
```

- Source the workspace and then run,
```
$ roslaunch nist_gear sample_environment.launch 
```

- In a seperate terminal run:
```
$ rosrun group5_rwa1 My_node
```
