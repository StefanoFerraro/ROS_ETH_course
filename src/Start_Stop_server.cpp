/*
 * Start_Stop_client.cpp
 *
 *  Created on: Mar 31, 2020
 *      Author: Stefano Ferraro
 */

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

ros::Publisher Start_Stop_Pub;

bool Start_Stop_srv_Callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  if(request.data == true)
  {
    response.success = true;
    Start_Stop_Pub.publish(true);
    response.message = "Stop request received";
  }
  else
  {
    response.success = false;
    Start_Stop_Pub.publish(false);
    response.message = "Start request received";
  }
  return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "Start_Stop_server");
  ros::NodeHandle NH;

  Start_Stop_Pub = NH.advertise<std_msgs::Bool>("/start_stop", 1);
  ros::ServiceServer service = NH.advertiseService("start_stop", Start_Stop_srv_Callback);
  ros::spin();
  return 0;
}

