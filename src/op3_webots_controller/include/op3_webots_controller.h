#ifndef OP3_WEBOTS_CONTROLLER_H
#define OP3_WEBOTS_CONTROLLER_H

#include <webots/Camera.hpp>
#include <webots/LED.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Speaker.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <map>

namespace robotis_op
{
class op3_webots_controller : public webots::Robot
{
public:
  op3_webots_controller();
  bool parseJointNameFromYaml(const std::string &path);
  bool init();

private:
  std::vector<std::string> joint_names_;
  std::map<std::string, webots::Motor*> joint_motor_map_;
};
}


#endif // OP3_WEBOTS_CONTROLLER_H
