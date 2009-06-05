#include <ros/node.h>
#include <ros/time.h>
#include <owd/WAMState.h>
#include "openwamdriver.h"
#include <sys/mman.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv);

  char *robotname;
  if (argc > 1) {
    robotname=strdup(argv[1]);
  } else {
    robotname=strdup("WAM");
  }

  if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1){
    ROS_FATAL("canbus_handler: mlockall failed: ");
    return NULL;
  }

  ROS_DEBUG("Creating robot %s",robotname);
  WamDriver wam(robotname);

  // read parameters and set wam options

  wam.Init("wam_joint_calibrations");

  ros::Node n(robotname);
  n.advertise<owd::WAMState>("wamstate", 10);
  n.advertiseService("AddTrajectory",&WamDriver::AddTrajectory,&wam);
  n.advertiseService("SetStiffness",&WamDriver::SetStiffness,&wam);
#ifndef FAKE7
  n.advertiseService("DeleteTrajectory",&WamDriver::DeleteTrajectory,&wam);
  n.advertiseService("PauseTrajectory",&WamDriver::PauseTrajectory,&wam);
  n.advertiseService("ReplaceTrajectory",&WamDriver::ReplaceTrajectory,&wam);
  n.advertiseService("SetSpeed",&WamDriver::SetSpeed,&wam);
  n.advertiseService("SetExtraMass",&WamDriver::SetExtraMass,&wam);
#endif // FAKE7
  n.advertiseService("GetArmDOF",&WamDriver::GetDOF,&wam);
  owd::Servo servocmd;
  n.subscribe("wamservo", servocmd, &WamDriver::wamservo_callback,&wam,&servocmd,10);
  //   n.advertiseService("CalibrateJoints",WamDriver::CalibrateJoints,&wam);
  //
  // Create a ros::Duration that will let us sleep for a specified period of time (in seconds)
  //
  ros::Duration sleep_duration(0.1);

  while (n.ok())
  {
    // publish our state info
    wam.Publish(n);

    // let the driver update
    wam.Update();
    
    //
    // Sleep for the duration we specified above
    //
    sleep_duration.sleep();
  }
}
