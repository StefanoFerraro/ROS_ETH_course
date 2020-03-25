/*
 *
 *  Created on: Mar 23, 2020
 *      Author: Stefano Ferraro
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using std::string;
using std::vector;

namespace my_hushy_controller
{
  class Controller
  {
  public:

    // Constructor
    Controller(ros::NodeHandle& nodeHandle);
    // Destructor (using a virtual desctructor make sure that any object of derived
    // class is desctructed properly (good programming habit))
    virtual~Controller();

  private:
    // initialize all the variable/function to be used in the main program
    ros::NodeHandle& nodeHandle_;
    ros::Subscriber Scan_Sub;

    string topic;
    int queue_size;
    vector<float> ranges; //array of float
    float Min_distance;
    int Min_index;

    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_data); //& used as a reference for passing large data structure without allocation extra memory

  };

}
