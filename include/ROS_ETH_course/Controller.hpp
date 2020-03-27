/*
 *
 *  Created on: Mar 23, 2020
 *      Author: Stefano Ferraro
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>


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
    ros::Publisher Twist_Pub;
    ros::Publisher Marker_Pub;
    tf::TransformListener listener;

    string topic;
    int queue_size;
    float K;
    vector<float> ranges; //array of float
    float Min_distance;
    int Min_index;
    float StartAngle;
    float AngleIncrement;
    float PillarAngle;
    float Linear_vel;
    float Angular_vel;

    geometry_msgs::Twist Twist;
    geometry_msgs::PointStamped PointLF;
    geometry_msgs::PointStamped PointOF;


    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_data); //& used as a reference for passing large data structure without allocation extra memory
    void PillarMarker();
  };

}
