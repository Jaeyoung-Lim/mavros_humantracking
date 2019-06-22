//  May/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HUMANTRACKING_CONTROLLER_H
#define HUMANTRACKING_CONTROLLER_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include "mavros_msgs/ActuatorControl.h"
#include "mavros_msgs/MountControl.h"

#include "geometric_controller/geometric_controller.h"

using namespace std;
using namespace Eigen;
class HumanTrackingCtrl
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher actuator_setpoint_pub_;
    ros::Publisher mount_control_pub_;

    ros::Timer cmdloop_timer_, statusloop_timer_;

    void CmdLoopCallback(const ros::TimerEvent& event);
    void StatusLoopCallback(const ros::TimerEvent& event);
    void PublishActuatorSetpoints();
    void PublishMountControl();

    geometricCtrl geometric_controller_;

    double gimbal_pitch_;
    double gimbal_yaw_;
    double dtheta;

  public:
    HumanTrackingCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ HumanTrackingCtrl();

};


#endif
