#include <ros/ros.h>
#include <ros/time.h>
#include <pr_msgs/WAMState.h>
#include <pr_msgs/IndexedJointValues.h>
#include <pr_msgs/WamSetupSeaCtrl.h>
#include "openwamdriver.h"
#include <sys/mman.h>

int main(int argc, char** argv)
{
  char *robotname;
  if (argc > 1) {
    robotname=strdup(argv[1]);
  } else {
    robotname=strdup("WAM");
  }

  ros::init(argc, argv, robotname, 0);

  if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1){
    ROS_FATAL("canbus_handler: mlockall failed: ");
    return NULL;
  }

  ROS_DEBUG("Creating robot %s",robotname);
  WamDriver wam(robotname);

  // read parameters and set wam options

  ros::NodeHandle n;
  std::string calibration_filename;
  n.param("~calibration_file",calibration_filename,std::string("wam_joint_calibrations"));
  wam.Init(calibration_filename.c_str());

  wam.AdvertiseAndSubscribe(n);

#ifdef BUILD_FOR_SEA
  usleep(100000);
  wam.publishAllSeaSettings();
#endif // BUILD_FOR_SEA

  //
  // Create a ros::Duration that will let us sleep for a specified period of time (in seconds)
  //
  ros::Duration sleep_duration(0.1);

  while (n.ok())
  {
    // publish our state info
    wam.Publish();

    // let the driver update
    wam.Update();
    
    //
    // Sleep for the duration we specified above
    //
    sleep_duration.sleep();
  }
}
