//  May/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "humantracking_controller/humantracking_controller.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"humantracking_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  HumanTrackingCtrl humantrackingcontroller(nh, nh_private);
  ros::spin();
  return 0;
}
