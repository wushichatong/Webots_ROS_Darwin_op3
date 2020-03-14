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
enum {LEFT, RIGHT};
int robot_status = STAND_INIT; //
webots_interface webots_interface_;
int main(int argv, char **argc)
{

    ros::init(argv, argc, "motion_control_node");
    ros::NodeHandle* nh = new ros::NodeHandle();

    webots_interface_.Init();


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
        walking(1);
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

  static double time_count = 0.0;
  time_count += 0.001;
//  cout<<time_count<<endl;
  robotis_framework::FifthOrderPolynomialTrajectory* a= new robotis_framework::FifthOrderPolynomialTrajectory(0, 0.2505,0,0,T,0.2005,0,0);
//  robotis_framework::FifthOrderPolynomialTrajectory* b= new robotis_framework::FifthOrderPolynomialTrajectory(0,0,0,0,T/2,0.05,0,0);

  tf::Pose body_pose = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(0, 0.0, a->getPosition(time_count)));
//  tf::Pose body_pose = new tf::Pose(tf::createQuaternionFromRPY(0,0,0),
//                                      tf::Point(0,0, a->getPosition(time_count)));
  tf::Pose left_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(0, 0.035 - 0.035,0));
  tf::Pose right_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(0, -0.035+ 0.035, 0));
  op3_kd_->calcInverseKinematicsForLeg(leg_angle, body_pose, left_foot, right_foot);

  if(time_count>T)
    robot_status = WALKING;// SWING WALKING

  return true;

}


bool walking(const double T){
  double right_leg_angle[6];
  double left_leg_angle[6];
  static double time_count = 0.0;
  static int swing_leg = RIGHT;
  static bool first_half_step = true;
  double leg_up_phase = 0.2 * T / 2.0;
  double leg_off_phase = 0.4 * T / 2.0;
  double leg_down_phase = 0.6 * T / 2.0;
  tf::Pose body_pose;

  tf::Pose left_foot;
  tf::Pose right_foot;
  double x_dist = 0.06;

  static tf::Pose left_foot_last = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(0, 0.035 - 0.035,0));
  static tf::Pose right_foot_last = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(0, -0.035 + 0.035,0));
  time_count += 0.01;
  cout<<time_count<<endl;
  robotis_framework::FifthOrderPolynomialTrajectory* a;// x axis
  robotis_framework::FifthOrderPolynomialTrajectory* b; //y axis
  robotis_framework::FifthOrderPolynomialTrajectory* c; //z axis

  if(swing_leg == RIGHT){ // swing right leg
    a = new robotis_framework::FifthOrderPolynomialTrajectory(0,right_foot_last.getOrigin().x(),0,0,T/2.0,x_dist,0,0);// x axis
    if(time_count< leg_up_phase){
      b = new robotis_framework::FifthOrderPolynomialTrajectory(0,0,0,0,leg_up_phase,0.03,0,0);
      c = new robotis_framework::FifthOrderPolynomialTrajectory(0,0,0,0,leg_up_phase,0.00,0,0); //z axis
    }else if(time_count< leg_off_phase){
      b = new robotis_framework::FifthOrderPolynomialTrajectory(leg_up_phase,0.03,0,0,
                                                                leg_off_phase,0.04,0,0);
      c = new robotis_framework::FifthOrderPolynomialTrajectory(leg_up_phase,0.00,0,0,
                                                                leg_off_phase,0.05,0,0); //z axis
    }else if(time_count< leg_down_phase) {
      b = new robotis_framework::FifthOrderPolynomialTrajectory(leg_off_phase,0.04,0,0,
                                                                leg_down_phase,0.03,0,0);
      c = new robotis_framework::FifthOrderPolynomialTrajectory(leg_off_phase,0.05,0,0,
                                                                leg_down_phase,0.00,0,0); //z axis
    }else {
      b = new robotis_framework::FifthOrderPolynomialTrajectory(leg_down_phase,0.03,0,0,
                                                                T/2.0,0,0,0);
      c = new robotis_framework::FifthOrderPolynomialTrajectory(leg_down_phase,0,0,0,                                                                T,0.00,0,0); //z axis
    }

    body_pose = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(a->getPosition(time_count) / 2.0, b->getPosition(time_count), 0.2005));
    left_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(left_foot_last.getOrigin().x(), 0.035 - 0.035,0));
    right_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(a->getPosition(time_count), -0.035+ 0.035, c->getPosition(time_count)));

  }else{// swing left leg
    a = new robotis_framework::FifthOrderPolynomialTrajectory(0,left_foot_last.getOrigin().x(),0,0,T/2.0,x_dist,0,0);// x axis
    if(time_count< leg_up_phase){
      b = new robotis_framework::FifthOrderPolynomialTrajectory(0,0,0,0,leg_up_phase,-0.03,0,0);
      c = new robotis_framework::FifthOrderPolynomialTrajectory(0,0,0,0,leg_up_phase,0.00,0,0); //z axis
    }else if(time_count< leg_off_phase){
      b = new robotis_framework::FifthOrderPolynomialTrajectory(leg_up_phase,-0.03,0,0,
                                                                leg_off_phase,-0.04,0,0);
      c = new robotis_framework::FifthOrderPolynomialTrajectory(leg_up_phase,0.00,0,0,
                                                                leg_off_phase,0.05,0,0); //z axis
    }else if(time_count< leg_down_phase) {
      b = new robotis_framework::FifthOrderPolynomialTrajectory(leg_off_phase,-0.04,0,0,
                                                                leg_down_phase,-0.03,0,0);
      c = new robotis_framework::FifthOrderPolynomialTrajectory(leg_off_phase,0.05,0,0,
                                                                leg_down_phase,0.00,0,0); //z axis
    }else {
      b = new robotis_framework::FifthOrderPolynomialTrajectory(leg_down_phase,-0.03,0,0,
                                                               T/2.0,0,0,0);
      c = new robotis_framework::FifthOrderPolynomialTrajectory(leg_down_phase,0,0,0,
                                                                T/2.0,0.00,0,0); //z axis
    }
    body_pose = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(a->getPosition(time_count) / 2.0, b->getPosition(time_count), 0.2005));
    right_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(right_foot_last.getOrigin().x(), 0.035 - 0.035,0));
    left_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(a->getPosition(time_count), -0.035+ 0.035, c->getPosition(time_count)));

  }


  op3_kd_->calcInverseKinematicsForLeg(leg_angle, body_pose, left_foot, right_foot);
//  cout<<"set z:"<<c->getPosition(time_count)<<endl;

  if(time_count>=T/2.0){
    swing_leg = swing_leg == LEFT ? RIGHT : LEFT;
    time_count = 0;
    left_foot_last = left_foot;
    right_foot_last = right_foot;
    cout<<left_foot_last.getOrigin().x()<<" r: "<<right_foot_last.getOrigin().x()<<endl;

  }
//    robot_status = STANDING;
  return true;
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
  return true;
}
