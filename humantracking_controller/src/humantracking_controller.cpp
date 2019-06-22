//  May/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "humantracking_controller/humantracking_controller.h"

using namespace Eigen;
using namespace std;
//Constructor
HumanTrackingCtrl::HumanTrackingCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  geometric_controller_(nh, nh_private) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &HumanTrackingCtrl::CmdLoopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &HumanTrackingCtrl::StatusLoopCallback, this); // Define timer for constant loop rate
  
  actuator_setpoint_pub_ = nh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1);
  mount_control_pub_ = nh_.advertise<mavros_msgs::MountControl>("/mavros/mount_control/command", 1);

  gimbal_pitch_ = 0.0;
  dtheta = 0.02;
}
HumanTrackingCtrl::~HumanTrackingCtrl() {
  //Destructor
}

void HumanTrackingCtrl::CmdLoopCallback(const ros::TimerEvent& event){
  // PublishActuatorSetpoints();
  PublishMountControl();

  gimbal_pitch_ = gimbal_pitch_ + dtheta;

  if(gimbal_pitch_ >= 3.14){
    gimbal_pitch_ = 3.14;
    dtheta = -0.2;
  }
  if(gimbal_pitch_ <= -3.14){
    gimbal_pitch_ = -3.14;
    dtheta = 0.2;
  }

}

void HumanTrackingCtrl::StatusLoopCallback(const ros::TimerEvent& event){


}

void HumanTrackingCtrl::PublishActuatorSetpoints(){
  mavros_msgs::ActuatorControl msg;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.group_mix = 2;
  msg.controls[1] = gimbal_pitch_ / 3.14; // Gimbal Pitch
  msg.controls[2] = gimbal_yaw_ / 3.14; // Gimbal  Yaw

  actuator_setpoint_pub_.publish(msg);

}

void HumanTrackingCtrl::PublishMountControl(){
  mavros_msgs::MountControl msg;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.mode = 2;
  msg.pitch = gimbal_pitch_ / 3.14 * 180; // Gimbal Pitch
  msg.roll = 0.0 / 3.14; // Gimbal  Yaw
  msg.yaw = gimbal_yaw_ / 3.14 * 180; // Gimbal  Yaw

  mount_control_pub_.publish(msg);
}

// void HumaanTrackingCtrl::PintGimbalToPoint(Eingen::Vector3d roi){
  
// }