#include <webots_interface.h>
#include <ros/ros.h>
#include <op3_kinematics_dynamics/op3_kinematics_dynamics.h>
#include <robotis_math/fifth_order_polynomial_trajectory.h>
#include <geometry_msgs/Pose.h>
#include <op3_webots_controller/Footstep.h>
robotis_op::OP3KinematicsDynamics* op3_kd_;
double leg_angle[12];
using namespace std;


enum {STAND_INIT, STANDING, WALKING, SWING, IDLE};
enum {LEFT, RIGHT};
int robot_status = STAND_INIT; //oo
double center_y_swing = 0.06;
double l_roll, l_pitch, l_yaw;//定义存储r\p\y的容器
double r_roll, r_pitch, r_yaw;//定义存储r\p\y的容器
geometry_msgs::Pose robot_pose;
geometry_msgs::TwistStamped twist;
ros::Publisher leg_left_pub_;
//Footstep robot_movement;
op3_webots_controller::Footstep robot_movement;
webots_interface webots_interface_;

bool standInit(const double T);
bool walking(const double T);
bool swing(const double T);
double quintic(double startPos, double endPos, double time_ratio);



int main(int argv, char **argc)
{

    ros::init(argv, argc, "motion_control_node");
    ros::NodeHandle* nh = new ros::NodeHandle();

    leg_left_pub_ = nh->advertise<geometry_msgs::TwistStamped>("/Leg/center", 10);
    robot_movement.pose.linear.x = 0.06;
    robot_movement.pose.linear.y = 0.02;
    robot_movement.pose.angular.z = 0.1;
    webots_interface_.Init();
    op3_kd_ = new robotis_op::OP3KinematicsDynamics(robotis_op::WholeBody);
    ros::Rate loop_rate(100);
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
  static const double swing_leg_height = 0.05;
  double double_support_phase = 0.3 * T / 2.0;
  double leg_up_phase = 0.5 * T / 2.0;
  double leg_down_phase = 0.7 * T / 2.0;

  tf::Pose body_pose;
  tf::Pose left_foot;
  tf::Pose right_foot;

  static tf::Pose left_foot_last;
  static tf::Pose right_foot_last;
  static tf::Pose body_last;

  time_count += 0.01;
  cout<<time_count<<endl;
robot_movement.pose.angular.z = 0.1;
robot_movement.pose.linear.y = 0.02;
robot_movement.pose.linear.x = 0.04;
static double x_pos = 0.00;
static double y_pos = 0.00;
static double z_pos = 0.00;
static double yaw_pos = 0.00;
static double center_y_pos = 0.00;
static double center_x_pos = 0.00;
  if(swing_leg == RIGHT){ // swing right leg
    if(robot_movement.pose.angular.z > 0)
      robot_movement.pose.angular.z = 0;
    if(robot_movement.pose.linear.y > 0)
      robot_movement.pose.linear.y = 0;

    if(time_count<double_support_phase){
      x_pos = right_foot_last.getOrigin().x();
      y_pos = right_foot_last.getOrigin().y();
      yaw_pos = r_yaw;
      center_x_pos = 0;
      cout<<"right phase 1"<<endl;
      center_y_pos = quintic(0,center_y_swing, time_count/double_support_phase);
    }else if(time_count< leg_up_phase){
      cout<<"right phase 2"<<endl;
      center_y_pos = center_y_swing;
      center_x_pos = quintic(0, robot_movement.pose.linear.x, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      x_pos = quintic(right_foot_last.getOrigin().x(), robot_movement.pose.linear.x, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      y_pos = quintic(right_foot_last.getOrigin().y(), robot_movement.pose.linear.y, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      z_pos = quintic(0, swing_leg_height, (time_count- double_support_phase) / (leg_up_phase - double_support_phase));
      yaw_pos = quintic(r_yaw, robot_movement.pose.angular.z,(time_count- double_support_phase) / (leg_down_phase - double_support_phase));
    }
    else if(time_count< leg_down_phase){
      cout<<"right phase 3"<<endl;
      center_y_pos = center_y_swing;
      center_x_pos = quintic(0, robot_movement.pose.linear.x, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      x_pos = quintic(right_foot_last.getOrigin().x(), robot_movement.pose.linear.x, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      y_pos = quintic(right_foot_last.getOrigin().y(), robot_movement.pose.linear.y, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      z_pos = quintic(swing_leg_height, 0, (time_count- leg_up_phase) / (leg_down_phase - leg_up_phase));
      yaw_pos = quintic(r_yaw, robot_movement.pose.angular.z,(time_count- double_support_phase) / (leg_down_phase - double_support_phase));
    }else{
      cout<<"right phase 4"<<endl;
      x_pos = robot_movement.pose.linear.x;
      y_pos = robot_movement.pose.linear.y;
      yaw_pos = robot_movement.pose.angular.z;
      center_y_pos = quintic(center_y_swing, 0, (time_count- leg_down_phase)/(T/2.0 - leg_down_phase));
    }

    body_pose = tf::Pose(tf::createQuaternionFromRPY(0,0,yaw_pos),
                         tf::Point(center_x_pos , center_y_pos + y_pos / 2.0, 0.2005));
    left_foot = tf::Pose(tf::createQuaternionFromRPY(0,0.00,0),
                         tf::Point(0,0.035 - 0.035,0));
    right_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,yaw_pos),
                          tf::Point(x_pos,-0.035+ 0.035 + y_pos, z_pos));

  }else{// swing left leg
    if(robot_movement.pose.angular.z < 0)
      robot_movement.pose.angular.z = 0;
    if(robot_movement.pose.linear.y < 0)
      robot_movement.pose.linear.y = 0;

    if(time_count<double_support_phase){
      cout<<"left phase 1"<<endl;
      x_pos = left_foot_last.getOrigin().x();
      y_pos = left_foot_last.getOrigin().y();
      yaw_pos = l_yaw;
      center_x_pos = 0;
      center_y_pos = quintic(0,-center_y_swing, time_count/double_support_phase);
      cout<<"left phase 1"<< center_y_pos <<endl;
    }else if(time_count< leg_up_phase){
      cout<<"left phase 2"<<endl;
      center_x_pos = quintic(0, robot_movement.pose.linear.x, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      x_pos = quintic(left_foot_last.getOrigin().x(), robot_movement.pose.linear.x, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      y_pos = quintic(left_foot_last.getOrigin().y(), robot_movement.pose.linear.y, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      z_pos = quintic(0, swing_leg_height, (time_count- double_support_phase) / (leg_up_phase - double_support_phase));
      yaw_pos = quintic(l_yaw, robot_movement.pose.angular.z,(time_count- double_support_phase) / (leg_down_phase - double_support_phase));
    }
    else if(time_count< leg_down_phase){
      cout<<"left phase 3"<<endl;
      center_x_pos = quintic(0, robot_movement.pose.linear.x, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      x_pos = quintic(left_foot_last.getOrigin().x(), robot_movement.pose.linear.x, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      y_pos = quintic(left_foot_last.getOrigin().y(), robot_movement.pose.linear.y, (time_count- double_support_phase) / (leg_down_phase - double_support_phase));
      z_pos = quintic(swing_leg_height, 0, (time_count- leg_up_phase) / (leg_down_phase - leg_up_phase));
      yaw_pos = quintic(l_yaw, robot_movement.pose.angular.z,(time_count- double_support_phase) / (leg_down_phase - double_support_phase));
    }else{
      cout<<"left phase 4"<<endl;
      x_pos = robot_movement.pose.linear.x;
      y_pos = robot_movement.pose.linear.y;
      yaw_pos = robot_movement.pose.angular.z;
      center_y_pos = quintic(-center_y_swing, 0, (time_count- leg_down_phase)/(T/2.0 - leg_down_phase));
    }

    body_pose = tf::Pose(tf::createQuaternionFromRPY(0,0,yaw_pos),
                         tf::Point(center_x_pos, center_y_pos + y_pos / 2.0, 0.2005));
    left_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,yaw_pos),
                          tf::Point(x_pos,-0.035+ 0.035 + y_pos, z_pos));
    right_foot = tf::Pose(tf::createQuaternionFromRPY(0,0.00,0),
                         tf::Point(0,0.035 - 0.035,0));

  }
  cout<<"x_pos:" <<x_pos<<endl;
  geometry_msgs::TwistStamped leg_ms;
  leg_ms.header.stamp = ros::Time::now();
  leg_ms.twist.linear.x = x_pos;
  leg_ms.twist.linear.y = y_pos;
  leg_ms.twist.linear.z = z_pos;
  leg_ms.twist.angular.x = center_x_pos;
  leg_ms.twist.angular.y = center_y_pos + y_pos / 2.0;
  leg_ms.twist.angular.z = yaw_pos;
  leg_left_pub_.publish(leg_ms);

//    cout<<yaw_direction->getPosition(time_count)<<" left foot: "<<l_yaw<<" right foot: "<<robot_movement.pose.angular.z<<endl;
  op3_kd_->calcInverseKinematicsForLeg(leg_angle, body_pose, left_foot, right_foot);
//cout<<"out"<<left_foot_last.getOrigin().y()<<" r_y: "<<right_foot_last.getOrigin().y()<<endl;
  if(time_count>=T/2.0){
    swing_leg = swing_leg == LEFT ? RIGHT : LEFT;
    time_count = 0;
    left_foot_last = left_foot;
    right_foot_last = right_foot;
//    double b_roll, b_pitch, b_yaw;
//    tf::Matrix3x3(body_pose.getRotation()).getRPY(b_roll, b_pitch, b_yaw);//进行转换
    tf::Matrix3x3(left_foot_last.getRotation()).getRPY(l_roll, l_pitch, l_yaw);//进行转换
    tf::Matrix3x3(right_foot_last.getRotation()).getRPY(r_roll, r_pitch, r_yaw);//进行转换

//    cout<<"change leg: "<<yaw_direction->getPosition(time_count)<<"body "<<b_yaw<<" left foot: "<<l_yaw<<" right foot: "<<r_yaw<<endl;
    cout<<"change"<<left_foot_last.getOrigin().x()<<" r_x: "<<right_foot_last.getOrigin().x()<<endl;

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

double quintic(double startPos, double endPos, double time_ratio){
    robotis_framework::FifthOrderPolynomialTrajectory* a= new robotis_framework::FifthOrderPolynomialTrajectory(0,startPos,0,0,
                                                                                                                1,endPos,0,0);
    if(time_ratio<0)
      time_ratio = 0;
    if(time_ratio>1)
      time_ratio =1;
    return a->getPosition(time_ratio);
}
