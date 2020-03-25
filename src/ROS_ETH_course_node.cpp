/*
 *
 *  Created on: Mar 23, 2020
 *      Author: Stefano Ferraro
 */

#include <ros/ros.h>
#include "ROS_ETH_course/Controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_husky_controller");
  ros::NodeHandle NH("~");

  my_hushy_controller::Controller MyHuskyController(NH);

  ros::spin();
  return 0;
  }




