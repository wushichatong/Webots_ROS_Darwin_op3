#include <ros/ros.h>
#include <op3_webots_controller.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "webots");
  ros::NodeHandle nh;
  robotis_op::op3_webots_controller controller;
  controller.init();

  ROS_INFO("Hello world!");
}
