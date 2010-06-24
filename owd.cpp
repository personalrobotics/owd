#include <ros/ros.h>
#include <ros/time.h>
#include <pr_msgs/WAMState.h>
#include <pr_msgs/IndexedJointValues.h>
#include <pr_msgs/WamSetupSeaCtrl.h>
#include "openwamdriver.h"
#include <sys/mman.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, std::string("owd"));

#ifndef OWDSIM
#ifndef NO_RT
  if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1){
    ROS_FATAL("owd: mlockall failed: ");
    return NULL;
  }
#endif
#endif


  // read parameters and set wam options

  ros::NodeHandle n("~");
  std::string calibration_filename;
  int canbus_number;
  n.param("calibration_file",calibration_filename,std::string("wam_joint_calibrations"));
  n.param("canbus_number",canbus_number,0);
  ROS_DEBUG("Using CANbus number %d",canbus_number);

  WamDriver wam(canbus_number);
  wam.Init(calibration_filename.c_str());

  wam.AdvertiseAndSubscribe(n);

#ifdef BUILD_FOR_SEA
  usleep(100000);
  wam.publishAllSeaSettings();
#endif // BUILD_FOR_SEA

  ROS_DEBUG("Creating timer and spinner threads");
  ros::Timer wam_timer = n.createTimer(ros::Duration(0.1), &WamDriver::Pump, &wam);
  ros::MultiThreadedSpinner s(3);
  ROS_DEBUG("Spinning");
  ros::spin(s);
  ROS_DEBUG("Done spinning; exiting");

}
