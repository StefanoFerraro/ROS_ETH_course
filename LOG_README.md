# [ROS - ETH course](https://rsl.ethz.ch/education-students/lectures/ros.html)

This a log file for the introductory course of ROS, held at ETH, RSL department. In the following chapters I will cover the main points of each exercise given at the end of each lecture.

The aim of this repo is to demonstrate what I have learned during this course, giving also the possibility to share my solutions with other students.

## [Exercise 2](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/ROS2020/Exercise%20Session%202.pdf)

**1.** 
Use command `catkin_create_pkg` to create a new package (add dependencies). We then open CMake ans package.xml files and uncomment the lines needed (lecture 2, slide 5-7). Next step is to create .cpp/.h files. First we build the project, then we import to Eclipse, now we are ready to create the new files.

`include/ROS_ETH_course/ControllerHeader.hpp` is the header file, it contains all the declarations, useful when dealing with a multi file program.

`src/ROS_ETH_course_node.cpp` contains the main loop (node file).

`src/ControllerMain.cpp` contains the main program with all the instruction. This program is called by `ROS_ETH_course_node.cpp`.

**4.** 
When creating the subscriber, pay attention to pointer/reference in the line, use `this` pointer when pointing to a function

**5.** 
Create a .yaml file with the specified parameters, in the launch file specify to load these parameters in the node. Then in the actual program, we get the desired params with `NodeHandle.getParam`.

**6.** 
When addressing to a member of a class that is a pointer, we use the `->` notation, instead when referring directly to a class object we use the `.` notation. Moreover if we want to refer directly to a class member (not the object) we have use `::` notation.

Init a vector using `std::vector` and then go through (for loop) all the ranges to find the minimum one.  

**7-8.**
When passing a node in the .launch file, pay attention to the type, this is the name of the file within the package we want to launch.

**9**
Before calling a package remember to build it.

After setting the first time the configuration in RViz, we can set it as default config by typing `ctrl+s`. This way next time we start RViz everything will be environment will be ready.

