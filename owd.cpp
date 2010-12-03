/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Corporation *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

#include <ros/ros.h>
#include <ros/time.h>
#include <pr_msgs/WAMState.h>
#include <pr_msgs/IndexedJointValues.h>
#include <pr_msgs/WamSetupSeaCtrl.h>
#include "openwamdriver.h"
#include "bhd280.hh"
#include <sys/mman.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, std::string("owd"));

#ifdef OWD_RT
  if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1){
    ROS_FATAL("owd: mlockall failed: ");
    return NULL;
  }
#endif


  // read parameters and set wam options

  ros::NodeHandle n("~");
  std::string calibration_filename;
  int canbus_number;
  n.param("calibration_file",calibration_filename,std::string("wam_joint_calibrations"));
  n.param("canbus_number",canbus_number,0);
  ROS_DEBUG("Using CANbus number %d",canbus_number);

  WamDriver wam(canbus_number);
  try {
    if (! wam.Init(calibration_filename.c_str())) {
      ROS_FATAL("WamDriver::Init() returned false; exiting.");
#ifdef CAN_RECORD
      wam.bus.~CANbus();
      ROS_FATAL("dumped CANbus logs to candata.log");
#endif
      exit(1);
    }
  } catch (int error) {
    ROS_FATAL("Error during WamDriver::Init(); exiting.");
#ifdef CAN_RECORD
    wam.bus.~CANbus();
    ROS_FATAL("dumped CANbus logs to candata.log");
#endif
    exit(1);
  } catch (char *errmsg) {
    ROS_FATAL("Error during WamDriver::Init(): %s",errmsg);
#ifdef CAN_RECORD
    wam.bus.~CANbus();
    ROS_FATAL("dumped CANbus logs to candata.log");
#endif
    exit(1);
  }
  try {
    wam.AdvertiseAndSubscribe(n);
  } catch (int error) {
    ROS_FATAL("Error during WamDriver::AdvertiseAndSubscribe(); exiting.");
#ifdef CAN_RECORD
    wam.bus.~CANbus();
    ROS_FATAL("dumped CANbus logs to candata.log");
#endif
    exit(1);
  }

#ifdef BH280
  BHD_280 bhd(&(wam.bus));
#endif // BH280

#ifdef BUILD_FOR_SEA
  usleep(100000);
  wam.publishAllSeaSettings();
#endif // BUILD_FOR_SEA

  ROS_DEBUG("Creating timer and spinner threads");
  ros::Timer wam_timer = n.createTimer(ros::Duration(0.1), &WamDriver::Pump, &wam);
#ifdef BH280
  ros::Timer bhd_timer = n.createTimer(ros::Duration(0.1), &BHD_280::Pump, &bhd);
#endif
  ros::MultiThreadedSpinner s(3);
  ROS_DEBUG("Spinning");
  ros::spin(s);
  ROS_DEBUG("Done spinning; exiting");
  wam.bus.~CANbus();
  ROS_FATAL("dumped CANbus logs to candata.log");
  exit(0);
}
