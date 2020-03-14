#include <webots_interface.h>
#include <ros/ros.h>
#include <op3_kinematics_dynamics/op3_kinematics_dynamics.h>
#include <robotis_math/fifth_order_polynomial_trajectory.h>

robotis_op::OP3KinematicsDynamics* op3_kd_;
double leg_angle[12];
using namespace std;

bool standInit(const double T);
bool walking(const double T);
bool swing(const double T);
enum {STAND_INIT, STANDING, WALKING, SWING, IDLE};
int robot_status = STAND_INIT; //
webots_interface webots_interface_;
int main(int argv, char **argc)
{

    ros::init(argv, argc, "motion_control_node");
    ros::NodeHandle* rosnode_ = new ros::NodeHandle();
    double right_leg_angle[6];
    double left_leg_angle[6];
    webots_interface_.Init();
//    double leg_angle[12];


    webots_interface_.Init();
    op3_kd_ = new robotis_op::OP3KinematicsDynamics(robotis_op::WholeBody);
    ros::Rate loop_rate(1000);
    loop_rate.sleep();

    while(ros::ok()){
      switch(robot_status){
      case STAND_INIT:
        standInit(0.1);
        break;
      case WALKING:
        walking(0.5);
        break;
      case SWING:
        swing(0.5);
      default:
        break;
      }
      webots_interface_.Write(leg_angle);
//      webots_interface_.Read(leg_angle);
      loop_rate.sleep();
      ros::spinOnce();
    }


    return 0;

}

bool standInit(const double T){
  double right_leg_angle[6];
  double left_leg_angle[6];
  static double time_count = 0.0;
  time_count += 0.001;
  cout<<time_count<<endl;
  robotis_framework::FifthOrderPolynomialTrajectory* a= new robotis_framework::FifthOrderPolynomialTrajectory(0, 0.2505,0,0,T,0.2005,0,0);
  robotis_framework::FifthOrderPolynomialTrajectory* b= new robotis_framework::FifthOrderPolynomialTrajectory(0,0,0,0,T,-0.05,0,0);
//  if (op3_kd_->calcInverseKinematicsForRightLeg(&right_leg_angle[0], 0.00, 0.0, a->getPosition(time_count), 0, 0, 0) == false)
//  {
//    printf("1 IK not Solved EPR : %f %f %f %f %f %f\n", 0.0, 0.0, a->getPosition(time_count), 0.0, 0.0, 0.0);
//    return false;
//  }

//  if (op3_kd_->calcInverseKinematicsForLeftLeg(&left_leg_angle[0], 0.00, -0.00, a->getPosition(time_count), 0, 0, 0) == false)
//  {
//    printf("2 IK not Solved EPR : %f %f %f %f %f %f\n", 0.0, 0.0, a->getPosition(time_count), 0.0, 0.0, 0.0);
//    return false;
//  }

//  for (int i=0; i<12; i++) {
//    if(i%2==0)
//      leg_angle[i] = right_leg_angle[i/2];
//    else {
//      leg_angle[i] = left_leg_angle[i/2];
//    }
//  }
  if(time_count>T)
    robot_status = SWING;// SWING WALKING

  tf::Pose body_pose = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(0,b->getPosition(time_count),a->getPosition(time_count)));
//  tf::Pose body_pose = new tf::Pose(tf::createQuaternionFromRPY(0,0,0),
//                                      tf::Point(0,0, a->getPosition(time_count)));
  tf::Pose left_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(0, 0.035 - 0.035,0));
  tf::Pose right_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(0, -0.035+ 0.035,0));
  op3_kd_->calcInverseKinematicsForLeg(leg_angle, body_pose, left_foot, right_foot);

  if(time_count>T)
    robot_status = SWING;// SWING WALKING

}


bool walking(const double T){
  double right_leg_angle[6];
  double left_leg_angle[6];
  static double time_count = 0.0;
  time_count += 0.001;
  cout<<time_count<<endl;
  robotis_framework::FifthOrderPolynomialTrajectory* a= new robotis_framework::FifthOrderPolynomialTrajectory(0,-0.2005,0,0,T,-0.1505,0,0);
  robotis_framework::FifthOrderPolynomialTrajectory* b= new robotis_framework::FifthOrderPolynomialTrajectory(0,0.0,0,0,T/2.0,0.05,0,0);
  if (op3_kd_->calcInverseKinematicsForRightLeg(&right_leg_angle[0], 0.00, b->getPosition(time_count), -0.2005, 0, 0, 0) == false)
  {
    printf("1 IK not Solved EPR : %f %f %f %f %f %f\n", 0.0, b->getPosition(time_count),0.0, 0.0, 0.0, 0.0);
    return false;
  }

  if (op3_kd_->calcInverseKinematicsForLeftLeg(&left_leg_angle[0], 0.00, b->getPosition(time_count), a->getPosition(time_count), 0, 0, 0) == false)
  {
    printf("2 IK not Solved EPR : %f %f %f %f %f %f\n", 0.0, 0.0, a->getPosition(time_count), 0.0, 0.0, 0.0);
    return false;
  }

  for (int i=0; i<12; i++) {
    if(i%2==0)
      leg_angle[i] = right_leg_angle[i/2];
    else {
      leg_angle[i] = left_leg_angle[i/2];
    }
  }
  if(time_count>T)
    robot_status = STANDING;

}

bool swing(const double T){
  double right_leg_angle[6];
  double left_leg_angle[6];
  static double time_count = 0.0;
  static int sign = 1;
  time_count += 0.001;
  cout<<time_count<<endl;
  robotis_framework::FifthOrderPolynomialTrajectory* a= new robotis_framework::FifthOrderPolynomialTrajectory(0,-0.2005,0,0,T,-0.1505,0,0);
  robotis_framework::FifthOrderPolynomialTrajectory* b= new robotis_framework::FifthOrderPolynomialTrajectory(0,0.0,0,0,T,0.05,0,0);
  if (op3_kd_->calcInverseKinematicsForRightLeg(&right_leg_angle[0], 0.00, b->getPosition(time_count), -0.2005, 0, 0, 0) == false)
  {
    printf("1 IK not Solved EPR ");
    return false;
  }

  if (op3_kd_->calcInverseKinematicsForLeftLeg(&left_leg_angle[0], 0.00, b->getPosition(time_count), -0.2005, 0, 0, 0) == false)
  {
    printf("2 IK not Solved EPR ");
    return false;
  }



  for (int i=0; i<12; i++) {
    if(i%2==0)
      leg_angle[i] = right_leg_angle[i/2];
    else {
      leg_angle[i] = left_leg_angle[i/2];
    }
  }
  if(time_count>T)
    robot_status = STANDING;

}
