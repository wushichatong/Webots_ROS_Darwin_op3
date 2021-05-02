/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/op3_webots_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_webots_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{
  debug_ = true;
  init();
}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"op3_webots_gui");
  if ( ! ros::master::check() ) {
    ROS_INFO("master is not online");
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle ros_node;

  desired_joint_state_pub_     = ros_node.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 10);

  // Config
  std::string config_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/joints_config.yaml";
  parseJointNameFromYaml(config_path);

  start();
  return true;
}

bool QNode::parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load id_joint table yaml.");
    return false;
  }
  // parse joint names and ids
  YAML::Node id_sub_node = doc["id_joint"];
  for (YAML::iterator _it = id_sub_node.begin(); _it != id_sub_node.end(); ++_it)
  {
    int joint_id;
    std::string joint_name;

    joint_id = _it->first.as<int>();
    joint_name = _it->second.as<std::string>();
    joint_names_[joint_id] = joint_name;

    if (debug_)
      std::cout << "ID : " << joint_id << " - " << joint_name << std::endl;
  }
  return true;
}
int QNode::getJointSize(){
  return joint_names_.size();
}
std::string QNode::getJointNameFromIndex(int joint_index){
  if(joint_index < 1 || joint_index > joint_names_.size()){
    ROS_ERROR("joint index[%d] out of index ", joint_index);
    return "";
  }
  return joint_names_[size_t(joint_index)];
}
void QNode::sendJointValue(int joint_index, double joint_value){
  if(joint_names_.find(joint_index) == joint_names_.end()){
    ROS_ERROR("sendJointValue index error");
    return ;
  }
  sensor_msgs::JointState desired_joint_state;
  desired_joint_state.name.push_back(joint_names_[(size_t)joint_index]);
  desired_joint_state.position.push_back(joint_value  * M_PI / 180);
  desired_joint_state.velocity.push_back(0);
  desired_joint_state.effort.push_back(0);
  desired_joint_state_pub_.publish(desired_joint_state);

}
void QNode::run() {
  ros::Rate loop_rate(1);
  int count = 0;
  while ( ros::ok() ) {

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


}  // namespace op3_webots_gui
