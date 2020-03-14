#include "webots_interface.h"


using namespace std;
using namespace webots;
static const char *motorNames[20] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

#define ID_R_LEG_START  (6)
#define ID_L_LEG_START  (7)
webots_interface::webots_interface()
{

}
webots_interface::~webots_interface()
{

}
bool webots_interface::Init(){


    cout<<"init:"<<endl;
//    wb_robot_init();

//    time_step=1;

//    //gait::MotorGaitApi::RHipPitchMotor
//    // motor
//    RHipYawMotor = wb_robot_get_device(motorNames[0]);
//    wb_motor_enable_torque_feedback(RHipYawMotor, time_step);
////    wb_position_sensor_enable(RHipYawMotor, time_step);
////wb_position_sensor_enable()

//    wb_motor_set_position(RHipYawMotor, 1);
//    cout<<wb_motor_get_target_position(RHipYawMotor)<<endl;
//    wb_robot_step(1);
//    webots::Robot robot = new Robot();
//    webots::Robot* robot = new webots::Robot();
    mTimeStep = getBasicTimeStep();
//    cout<<"0 value:";
//    webots::Motor* ShoulderRMotor = getMotor("ShoulderR");
//    webots::PositionSensor* ShoulderRPositionSensor = ShoulderRMotor->getPositionSensor();
//    ShoulderRPositionSensor->enable(mTimeStep);
//    cout<<"1 value:"<<ShoulderRPositionSensor->getValue()<<endl;
//    ShoulderRMotor->setPosition(1);
//    webots::LED* headLed = getLED("HeadLed");
//    headLed->set(0xFF00FF);
//    step(mTimeStep);
//    cout<<"2 value:"<<ShoulderRPositionSensor->getValue()<<endl;

    for (int i = 0; i < NMOTORS; i++) {
      string sensorName = motorNames[i];
      getMotor(sensorName)->setPosition(0);
      sensorName.push_back('S');
      getPositionSensor(sensorName)->enable(mTimeStep);
    }

    step(mTimeStep);


}
bool webots_interface::Read(double* angle){
//  cout<<wb_motor_get_target_position(RHipYawMotor)<<endl;
  cout<<"into read"<<endl;
  for (int i = 0; i < NMOTORS; i++) {
    string sensorName = motorNames[i];
    sensorName.push_back('S');
//    getPositionSensor(sensorName)->enable(mTimeStep);
//    cout<<getPositionSensor(sensorName)->getValue()<<endl;
//    cout<<angle[i];
  }

}
bool webots_interface::Write(double* angle){
//  wb_motor_set_position(RHipYawMotor, 1);
  for (int i = 0; i < 12; i++) {
    string sensorName = motorNames[ID_R_LEG_START + i];
    getMotor(sensorName)->setPosition(angle[i]);
  }
  step(mTimeStep);
//  cout<<"write succ"<<endl;

}

