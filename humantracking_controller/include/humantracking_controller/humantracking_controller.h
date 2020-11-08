//  May/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HUMANTRACKING_CONTROLLER_H
#define HUMANTRACKING_CONTROLLER_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/MountControl.h>

using namespace std;
using namespace Eigen;
class HumanTrackingCtrl {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher actuator_setpoint_pub_;
  ros::Publisher mount_control_pub_;

  ros::Subscriber point_of_interest_sub_;
  ros::Subscriber mav_pose_sub_;

  ros::Timer cmdloop_timer_, statusloop_timer_;

  void CmdLoopCallback(const ros::TimerEvent& event);
  void StatusLoopCallback(const ros::TimerEvent& event);
  void mavposeCallback(const geometry_msgs::PoseStamped& msg);
  void PublishActuatorSetpoints();
  void PublishMountControl();
  void PointGimbalToPoint(Eigen::Vector3d roi_point);
  void PointOfInterestCallback(const geometry_msgs::PointStamped& msg);

  double gimbal_pitch_;
  double gimbal_yaw_;

  Eigen::Vector3d mav_pos_, mav_vel_, mav_bodyrate_;
  Eigen::Vector3d tracking_pos_;  //
  Eigen::Vector4d mav_att_;

 public:
  HumanTrackingCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~HumanTrackingCtrl();
};

#endif
