ROS (catkin) package for Udacity/Didi Competition data visualization and modelling. Any contribution is welcome!

### Dependencies
This requires ROS Kinetic. Please refer to the [ROS Documentation](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) on how to build catkin packages.

Other dependencies:
* numpy
* ROS Velodyne driver https://bitbucket.org/DataspeedInc/ros_binaries

### Usage
Launch the visualiver:
```
roslaunch didi_pipeline visualize.launch
```

Play a bag file:
```
roslaunch didi_pipeline play.launch bag:=absolute_path_to_bag_file
```
