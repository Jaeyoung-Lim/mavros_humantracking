//  May/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "humantracking_controller/humantracking_controller.h"

using namespace Eigen;
using namespace std;
// Constructor
HumanTrackingCtrl::HumanTrackingCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &HumanTrackingCtrl::CmdLoopCallback,
                                   this);  // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &HumanTrackingCtrl::StatusLoopCallback,
                                      this);  // Define timer for constant loop rate

  actuator_setpoint_pub_ = nh_.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);
  mount_control_pub_ = nh_.advertise<mavros_msgs::MountControl>("mavros/mount_control/command", 1);

  point_of_interest_sub_ = nh_private_.subscribe("point_of_interest", 1, &HumanTrackingCtrl::PointOfInterestCallback,
                                                 this, ros::TransportHints().tcpNoDelay());
  mav_pose_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &HumanTrackingCtrl::mavposeCallback, this,
                                ros::TransportHints().tcpNoDelay());
  gimbal_pitch_ = 0.0;
  tracking_pos_ << 0.0, 0.0, 0.0;
}
HumanTrackingCtrl::~HumanTrackingCtrl() {
  // Destructor
}

void HumanTrackingCtrl::CmdLoopCallback(const ros::TimerEvent& event) {
  PointGimbalToPoint(tracking_pos_);
  // PublishActuatorSetpoints();
  PublishMountControl();
}

void HumanTrackingCtrl::StatusLoopCallback(const ros::TimerEvent& event) {}

void HumanTrackingCtrl::PointOfInterestCallback(const geometry_msgs::PointStamped& msg) {
  tracking_pos_(0) = msg.point.x;
  tracking_pos_(1) = msg.point.y;
  tracking_pos_(2) = msg.point.z;
}

void HumanTrackingCtrl::PublishActuatorSetpoints() {
  mavros_msgs::ActuatorControl msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.group_mix = 2;
  msg.controls[1] = gimbal_pitch_ / 3.14;  // Gimbal Pitch
  msg.controls[2] = gimbal_yaw_ / 3.14;    // Gimbal  Yaw

  actuator_setpoint_pub_.publish(msg);
}

void HumanTrackingCtrl::PublishMountControl() {
  mavros_msgs::MountControl msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.mode = 2;
  msg.pitch = gimbal_pitch_ / 3.14 * 180;  // Gimbal Pitch
  msg.roll = 0.0 / 3.14;                   // Gimbal  Yaw
  msg.yaw = gimbal_yaw_ / 3.14 * 180;      // Gimbal  Yaw

  mount_control_pub_.publish(msg);
}

void HumanTrackingCtrl::PointGimbalToPoint(Eigen::Vector3d roi_point) {
  Eigen::Vector3d error_vec;
  double distance_2d;

  error_vec = roi_point - mav_pos_;

  distance_2d = std::sqrt(error_vec(0) * error_vec(0) + error_vec(1) * error_vec(1));

  gimbal_pitch_ = std::atan2(error_vec(2), distance_2d);
  gimbal_yaw_ = -std::atan2(error_vec(1), error_vec(0));
}

void HumanTrackingCtrl::mavposeCallback(const geometry_msgs::PoseStamped& msg) {
  mav_pos_(0) = msg.pose.position.x;
  mav_pos_(1) = msg.pose.position.y;
  mav_pos_(2) = msg.pose.position.z;
}
