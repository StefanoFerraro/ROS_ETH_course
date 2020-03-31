/*
 * Start_Stop_client.cpp
 *
 *  Created on: Mar 31, 2020
 *      Author: Stefano Ferraro
 */

#include <std_srvs/SetBool.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::ServiceClient client;
std_srvs::SetBool service;

void Crash_Callback(const sensor_msgs::Imu::ConstPtr& imu_data)
{
  float accel_z = imu_data->linear_acceleration.z;

  if(accel_z > 22.0)
  {
    service.request.data = true;
    client.call(service);
    ROS_INFO_STREAM(service.response.message);
    ROS_INFO("Crash detected!");
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "Start_Stop_client");
  ros::NodeHandle NH;

  client = NH.serviceClient<std_srvs::SetBool>("start_stop");
  ros::Subscriber Imu_Sub = NH.subscribe("/imu/data", 1, Crash_Callback);
  ros::spin();
  return 0;
}





