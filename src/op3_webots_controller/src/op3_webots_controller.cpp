#include "op3_webots_controller.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <ros/package.h>

#include <yaml-cpp/yaml.h>

using namespace webots;
using namespace std;
namespace robotis_op
{
op3_webots_controller::op3_webots_controller()
{
  ros::NodeHandle ros_node;
  ros::Publisher current_module_pub_       = ros_node.advertise<robotis_controller_msgs::JointCtrlModule>(
                                                              "/robotis/present_joint_ctrl_modules", 10);
  ros::Publisher goal_joint_state_pub_     = ros_node.advertise<sensor_msgs::JointState>("/robotis/goal_joint_states", 10);
  ros::Publisher present_joint_state_pub_  = ros_node.advertise<sensor_msgs::JointState>("/robotis/present_joint_states", 10);
  robotis_controller_msgs::JointCtrlModule _current_module_msg;
  current_module_pub_.publish(_current_module_msg);
  // Config
  std::string default_config_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/gui_config.yaml";
  std::string config_path = ros_node.param<std::string>("gui_config", default_config_path);
  parseJointNameFromYaml(config_path);

}

bool op3_webots_controller::parseJointNameFromYaml(const std::string &path)
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

  // parse id_joint table
  YAML::Node id_sub_node = doc["id_joint"];
  for (YAML::iterator _it = id_sub_node.begin(); _it != id_sub_node.end(); ++_it)
  {
    int joint_id;
    std::string joint_name;

    joint_id = _it->first.as<int>();
    joint_name = _it->second.as<std::string>();
    joint_names_.push_back(joint_name);

//    id_joint_table_[joint_id] = joint_name;
//    joint_id_table_[joint_name] = joint_id;

//    if (debug_)
      std::cout << "ID : " << joint_id << " - " << joint_name << std::endl;
  }

  // parse module
  std::vector<std::string> modules = doc["module_list"].as<std::vector<std::string> >();

//  int module_index = 0;
  for (std::vector<std::string>::iterator modules_it = modules.begin(); modules_it != modules.end(); ++modules_it)
  {
    std::string module_name = *modules_it;

//    index_mode_table_[module_index] = module_name;
//    mode_index_table_[module_name] = module_index++;

//    using_mode_table_[module_name] = false;
  }

  // parse module_joint preset
  YAML::Node sub_node = doc["module_button"];
  for (YAML::iterator yaml_it = sub_node.begin(); yaml_it != sub_node.end(); ++yaml_it)
  {
    int key_index;
    std::string module_name;

    key_index = yaml_it->first.as<int>();
    module_name = yaml_it->second.as<std::string>();

//    module_table_[key_index] = module_name;
//    if (debug_)
      std::cout << "Preset : " << module_name << std::endl;
  }
  return true;
}

bool op3_webots_controller::init(){
  double mTimeStep = getBasicTimeStep();
  for(size_t joint_index = 0; joint_index < joint_names_.size(); joint_index++){
    string joint_name = joint_names_[joint_index];
    joint_motor_map_[joint_name] = getMotor(joint_name);
  }
  return true;
}
}
