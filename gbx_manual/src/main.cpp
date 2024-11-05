//
// Created by lsy on 24-11-5.
//

#include "ros/ros.h"
#include "gbx_manual/GBXManual.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gbx_manual_node");

  gbx_manual::GBXManual manual_control;

  manual_control.initialize();

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    manual_control.update();

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}