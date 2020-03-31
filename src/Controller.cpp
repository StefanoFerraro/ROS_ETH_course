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
    nodeHandle_.getParam("husky/gain", K);
    Start_Stop_Sub = nodeHandle_.subscribe("/start_stop", 1, &Controller::Start_StopCallback, this );
    Scan_Sub = nodeHandle_.subscribe(topic, queue_size, &Controller::ScanCallback, this ); // & before a function is used as a pointer to the address of the function
    // this is a object instance for which we want the callback function called.
    Twist_Pub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", queue_size);
    Marker_Pub = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  }


  Controller::~Controller()
  {
  }

  void Controller::Start_StopCallback(const std_msgs::Bool& start_stop_data)
  {
    if(start_stop_data.data == true)
    {
      Start_Stop_data = true;
    }
    else
    {
      Start_Stop_data = false;
    }
  }

  void Controller::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_data) // `::` notation is used we need to access the a static variable or a method of a class/struct/namespace
  {
    ranges = scan_data->ranges; //scan_data is a pointer, so we use `->` notation
    Min_distance = scan_data->range_max;
    Min_index = 0;
    StartAngle = scan_data->angle_min;
    AngleIncrement = scan_data->angle_increment;

    for(int i=0; i < ranges.size(); i++) //ranges is an object, so we use `.` notation
    {
      if(ranges[i] < Min_distance)
      {
        Min_distance = ranges[i];
        Min_index = i;
      }
    }

    PillarAngle = StartAngle+AngleIncrement*Min_index;

    ROS_INFO_STREAM_THROTTLE(1, "Min distance: " << Min_distance); //Send a message to the output method of chosen in the launch file every 1s
    ROS_INFO_STREAM_THROTTLE(1, "Angle Pillar: " << PillarAngle);

    // Computation for the P controller

    Linear_vel = K*Min_distance + 1;
    Angular_vel = -K*(PillarAngle);

    if(Start_Stop_data == false)
    {
      Twist.linear.x = Linear_vel;
      Twist.angular.z = Angular_vel;
    }
    else
    {
      Twist.linear.x = 0.0;
      Twist.angular.z = 0.0;
    }

    Twist_Pub.publish(Twist); // Publishing the velocity message

    PointLF.point.x = cos(PillarAngle)*Min_distance;
    PointLF.point.y = sin(PillarAngle)*Min_distance;
    PointLF.header.frame_id = "base_laser"; // set the frame_id for the following trasformation

    try{ // needed for the startup routine

      listener.transformPoint("odom", PointLF, PointOF);
      PillarMarker();
    }

    catch(tf::TransformException& ex){
      ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
    }

  }

  void Controller::PillarMarker()
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = PointOF.point.x;
    marker.pose.position.y = PointOF.point.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    Marker_Pub.publish(marker);
    }

} /* namespace */
