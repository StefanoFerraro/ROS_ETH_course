/*
 *
 *  Created on: Mar 23, 2020
 *      Author: Stefano Ferraro
 */

#include "ROS_ETH_course/Controller.hpp"

namespace my_hushy_controller
{

  Controller::Controller(ros::NodeHandle& NH):
      nodeHandle_(NH)
  {
    nodeHandle_.getParam("husky/topic_name", topic);
    nodeHandle_.getParam("husky/queue_size", queue_size);
    Scan_Sub = nodeHandle_.subscribe(topic, queue_size, &Controller::ScanCallback, this ); // & before a function is used as a pointer to the address of the function
    // this is a object instance for which we want the callback function called.
  }


  Controller::~Controller()
  {
  }

  void Controller::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_data) // `::` notation is used we need to access the a static variable or a method of a class/struct/namespace
  {
    ranges = scan_data->ranges; //scan_data is a pointer, so we use `->` notation
    Min_distance = scan_data->range_max;
    Min_index = 0;

    for(int i=0; i < ranges.size(); i++) //ranges is an object, so we use `.` notation
    {
      if(ranges[i] < Min_distance)
      {
        Min_distance = ranges[i];
        Min_index = i;
      }
    }

    ROS_INFO_STREAM_THROTTLE(1, "Min distance: " << Min_distance); //Send a message to the output method of chosen in the launch file every 1s

  }


} /* namespace */



