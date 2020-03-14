#ifndef WEBOTS_INTERFACE_H
#define WEBOTS_INTERFACE_H

//#include <webots/keyboard.h>
//#include <webots/motor.h>
//#include <webots/robot.h>
//#include <webots/position_sensor.h>
//#include <webots/utils/motion.h>
//#include <webots/touch_sensor.h>

//#include <webots/inertial_unit.h>
//#include <webots/accelerometer.h>
//#include <webots/gyro.h>

#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Speaker.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#include <sensor_msgs/JointState.h>

#include <iostream>

#define NMOTORS 20
class webots_interface : public webots::Robot
{
public:
    webots_interface();
    ~webots_interface();
    bool Init();
    bool Read(double* angle);
    bool Write(double* angle);
    WbDeviceTag RHipYawMotor,RHipRollMotor,RHipPitchMotor,RKneePitchMotor,RAnklePitchMotor,RAnkleRollMotor;
    int mTimeStep ;

private:





};

#endif // WEBOTS_INTERFACE_H
