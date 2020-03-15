#include <webots_interface.h>
#include <ros/ros.h>
#include <op3_kinematics_dynamics/op3_kinematics_dynamics.h>
#include <robotis_math/fifth_order_polynomial_trajectory.h>
#include <geometry_msgs/Pose.h>
#include <op3_webots_controller/Footstep.h>
#include <op3_webots_controller/Footstep.h>
robotis_op::OP3KinematicsDynamics* op3_kd_;
double leg_angle[12];
using namespace std;

bool standInit(const double T);
bool walking(const double T);
bool swing(const double T);
enum {STAND_INIT, STANDING, WALKING, SWING, IDLE};
enum {LEFT, RIGHT};
int robot_status = STAND_INIT; //oo
double center_y_swing = 0.05;
double l_roll, l_pitch, l_yaw;//定义存储r\p\y的容器
double r_roll, r_pitch, r_yaw;//定义存储r\p\y的容器
geometry_msgs::Pose robot_pose;
geometry_msgs::TwistStamped twist;
ros::Publisher leg_left_pub_;
//Footstep robot_movement;
op3_webots_controller::Footstep robot_movement;
webots_interface webots_interface_;
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
  double double_support_phase = 0.2 * T / 2.0;
  double leg_up_phase = 0.4 * T / 2.0;
  double leg_down_phase = 0.6 * T / 2.0;
  tf::Pose body_pose;

  tf::Pose left_foot;
  tf::Pose right_foot;
//  double x_dist = 0.06;

  static tf::Pose left_foot_last = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(0, 0.035 - 0.035,0));
  static tf::Pose right_foot_last = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(0, -0.035 + 0.035,0));
  time_count += 0.001;
  cout<<time_count<<endl;
  robotis_framework::FifthOrderPolynomialTrajectory* x_direction;// x axis
  robotis_framework::FifthOrderPolynomialTrajectory* center_y_direction; //y axis
  robotis_framework::FifthOrderPolynomialTrajectory* y_direction; //y axis
  robotis_framework::FifthOrderPolynomialTrajectory* z_direction; //z axis
  robotis_framework::FifthOrderPolynomialTrajectory* yaw_direction; //z axis
robot_movement.pose.angular.z = 0.2;
robot_movement.pose.linear.y = 0.02;
  if(swing_leg == RIGHT){ // swing right leg
    if(robot_movement.pose.angular.z > 0)
      robot_movement.pose.angular.z = 0;
    if(robot_movement.pose.linear.y > 0)
      robot_movement.pose.linear.y = 0;
//    center_y_swing = center_y_swing + robot_movement.pose.linear.y;
    x_direction = new robotis_framework::FifthOrderPolynomialTrajectory(double_support_phase,right_foot_last.getOrigin().x(),0,0,leg_down_phase,robot_movement.pose.linear.x,0,0);// x axis
    y_direction = new robotis_framework::FifthOrderPolynomialTrajectory(double_support_phase,0,0,0,
                                                                           leg_down_phase,robot_movement.pose.linear.y,0,0);// x axis
    yaw_direction = new robotis_framework::FifthOrderPolynomialTrajectory(double_support_phase,r_yaw,0,0,leg_down_phase,robot_movement.pose.angular.z,0,0);
    if(time_count< double_support_phase){
      // double support phase
      center_y_direction = new robotis_framework::FifthOrderPolynomialTrajectory(0,0,0,0,double_support_phase,0.75 * center_y_swing,0,0);
      z_direction = new robotis_framework::FifthOrderPolynomialTrajectory(0,0,0,0,double_support_phase,0.00,0,0); //z axis

      yaw_direction = new robotis_framework::FifthOrderPolynomialTrajectory(0,r_yaw,0,0,
                                                                            double_support_phase,r_yaw,0,0);

    }else if(time_count< leg_up_phase){
      // leg up phase
      center_y_direction = new robotis_framework::FifthOrderPolynomialTrajectory(double_support_phase,0.75 * center_y_swing,0,0,
                                                                leg_up_phase, center_y_swing,0,0);
      z_direction = new robotis_framework::FifthOrderPolynomialTrajectory(double_support_phase,0.00,0,0,
                                                                leg_up_phase,swing_leg_height,0,0); //z axis
    }else if(time_count< leg_down_phase) {
      // leg down phase
      center_y_direction = new robotis_framework::FifthOrderPolynomialTrajectory(leg_up_phase,center_y_swing,0,0,
                                                                leg_down_phase,0.75 * center_y_swing,0,0);
      z_direction = new robotis_framework::FifthOrderPolynomialTrajectory(leg_up_phase,swing_leg_height,0,0,
                                                                leg_down_phase,0.00,0,0); //z axis
    }else {
      // double support phase
      center_y_direction = new robotis_framework::FifthOrderPolynomialTrajectory(leg_down_phase,0.75 * center_y_swing,0,0,
                                                                T/2.0,0,0,0);
      z_direction = new robotis_framework::FifthOrderPolynomialTrajectory(leg_down_phase,0,0,0,
                                                                          T/2,0.00,0,0); //z axis
      yaw_direction = new robotis_framework::FifthOrderPolynomialTrajectory(0,robot_movement.pose.angular.z,0,0,
                                                                            double_support_phase,robot_movement.pose.angular.z,0,0);
    }

    body_pose = tf::Pose(tf::createQuaternionFromRPY(0,0,yaw_direction->getPosition(time_count)), tf::Point(x_direction->getPosition(time_count),
                                                                       center_y_direction->getPosition(time_count) + y_direction->getPosition(time_count),
                                                                       0.2005));
    left_foot = tf::Pose(tf::createQuaternionFromRPY(0,0.00,0), tf::Point(left_foot_last.getOrigin().x(),
                                                                       0.035 - 0.035,
                                                                       0));
    right_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,yaw_direction->getPosition(time_count)),
                          tf::Point(x_direction->getPosition(time_count),-0.035+ 0.035 + y_direction->getPosition(time_count),z_direction->getPosition(time_count)));

  }else{// swing left leg
    if(robot_movement.pose.angular.z < 0)
      robot_movement.pose.angular.z = 0;
    if(robot_movement.pose.linear.y < 0)
      robot_movement.pose.linear.y = 0;

    x_direction = new robotis_framework::FifthOrderPolynomialTrajectory(double_support_phase,left_foot_last.getOrigin().x(),0,0, leg_down_phase, robot_movement.pose.linear.x, 0, 0);// x axis
    y_direction = new robotis_framework::FifthOrderPolynomialTrajectory(double_support_phase,0,0,0,
                                                                           leg_down_phase, robot_movement.pose.linear.y, 0, 0);// x axis
    yaw_direction = new robotis_framework::FifthOrderPolynomialTrajectory(double_support_phase,l_yaw,0,0,leg_down_phase,robot_movement.pose.angular.z,0,0);


//    yaw_direction = new robotis_framework::FifthOrderPolynomialTrajectory(0,r_yaw,0,0,T/2.0,robot_movement.pose.angular.z,0,0);
    if(time_count< double_support_phase){
      center_y_direction = new robotis_framework::FifthOrderPolynomialTrajectory(0,0,0,0,double_support_phase,-0.75 * center_y_swing,0,0);
      z_direction = new robotis_framework::FifthOrderPolynomialTrajectory(0,0,0,0,double_support_phase,0.00,0,0); //z axis
      yaw_direction = new robotis_framework::FifthOrderPolynomialTrajectory(0,l_yaw,0,0,
                                                                            double_support_phase,l_yaw,0,0);
    }else if(time_count< leg_up_phase){
      center_y_direction = new robotis_framework::FifthOrderPolynomialTrajectory(double_support_phase,-0.75 * center_y_swing,0,0,
                                                                leg_up_phase,- center_y_swing,0,0);
      z_direction = new robotis_framework::FifthOrderPolynomialTrajectory(double_support_phase,0.00,0,0,
                                                                leg_up_phase,swing_leg_height,0,0); //z axis
    }else if(time_count< leg_down_phase) {
      center_y_direction = new robotis_framework::FifthOrderPolynomialTrajectory(leg_up_phase,- center_y_swing,0,0,
                                                                leg_down_phase,-0.75 * center_y_swing,0,0);
      z_direction = new robotis_framework::FifthOrderPolynomialTrajectory(leg_up_phase,swing_leg_height,0,0,
                                                                leg_down_phase,0.00,0,0); //z axis
    }else {
      center_y_direction = new robotis_framework::FifthOrderPolynomialTrajectory(leg_down_phase,-0.75 * center_y_swing,0,0,
                                                               T/2.0,0,0,0);
      z_direction = new robotis_framework::FifthOrderPolynomialTrajectory(leg_down_phase,0,0,0,
                                                                T/2.0,0.00,0,0); //z axis
      yaw_direction = new robotis_framework::FifthOrderPolynomialTrajectory(leg_down_phase,robot_movement.pose.angular.z,0,0,
                                                                            T/2.0,robot_movement.pose.angular.z,0,0);
    }
    body_pose = tf::Pose(tf::createQuaternionFromRPY(0,0.0,yaw_direction->getPosition(time_count)), tf::Point(x_direction->getPosition(time_count),
                                                                       center_y_direction->getPosition(time_count) + y_direction->getPosition(time_count),
                                                                       0.2005));
    right_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,0), tf::Point(right_foot_last.getOrigin().x(),
                                                                        0.035 - 0.035,
                                                                        0));
    left_foot = tf::Pose(tf::createQuaternionFromRPY(0,0,yaw_direction->getPosition(time_count)),
                         tf::Point(x_direction->getPosition(time_count),-0.035+ 0.035 + y_direction->getPosition(time_count), z_direction->getPosition(time_count)));

  }

  geometry_msgs::TwistStamped leg_ms;
  leg_ms.header.stamp = ros::Time::now();
  leg_ms.twist.linear.x = x_direction->getPosition(time_count);
  leg_ms.twist.linear.y = y_direction->getPosition(time_count);
  leg_ms.twist.linear.z = z_direction->getPosition(time_count);
  leg_ms.twist.angular.x = center_y_direction->getPosition(time_count) + y_direction->getPosition(time_count);
  leg_ms.twist.angular.y = center_y_direction->getPosition(time_count);
  leg_ms.twist.angular.z = yaw_direction->getPosition(time_count);
  leg_left_pub_.publish(leg_ms);

//    cout<<yaw_direction->getPosition(time_count)<<" left foot: "<<l_yaw<<" right foot: "<<robot_movement.pose.angular.z<<endl;
  op3_kd_->calcInverseKinematicsForLeg(leg_angle, body_pose, left_foot, right_foot);
//  cout<<"set z:"<<c->getPosition(time_count)<<endl;
cout<<"out"<<left_foot_last.getOrigin().y()<<" r_y: "<<right_foot_last.getOrigin().y()<<endl;
  if(time_count>=T/2.0){
    swing_leg = swing_leg == LEFT ? RIGHT : LEFT;
    time_count = 0;
    left_foot_last = left_foot;
    right_foot_last = right_foot;
    double b_roll, b_pitch, b_yaw;
    tf::Matrix3x3(body_pose.getRotation()).getRPY(b_roll, b_pitch, b_yaw);//进行转换
    tf::Matrix3x3(left_foot_last.getRotation()).getRPY(l_roll, l_pitch, l_yaw);//进行转换
    tf::Matrix3x3(right_foot_last.getRotation()).getRPY(r_roll, r_pitch, r_yaw);//进行转换
//    cout<<"change leg: "<<yaw_direction->getPosition(time_count)<<"body "<<b_yaw<<" left foot: "<<l_yaw<<" right foot: "<<r_yaw<<endl;
    cout<<"change"<<left_foot_last.getOrigin().y()<<" r_y: "<<right_foot_last.getOrigin().y()<<endl;

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
