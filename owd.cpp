#include <ros/node.h>
#include <ros/time.h>
#include <owd/WAMState.h>
#include <owd/IndexedJointValues.h>
#include <owd/WamSetupSeaCtrl.h>
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

  ros::Node n(robotname);
  wam.Init("wam_joint_calibrations");

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
  n.advertiseService("CalibrateJoints", &WamDriver::CalibrateJoints, &wam);
  owd::Servo servocmd;
  n.subscribe("wamservo", servocmd, &WamDriver::wamservo_callback,&wam,&servocmd,10);

  // LLL
  owd::IndexedJointValues jtcmd;
  n.subscribe("wam_joint_targets", jtcmd, &WamDriver::wamjointtargets_callback,&wam,&jtcmd,10);
 
#ifdef BUILD_FOR_SEA
  owd::WamSetupSeaCtrl tlcmd;
  n.subscribe("wam_seactrl_settl", tlcmd, &WamDriver::wam_seactrl_settl_callback, &wam, &tlcmd, 10); 
  n.advertise<owd::IndexedJointValues>("wam_seactrl_curtl", 10);
  n.advertiseService("WamRequestSeaCtrlTorqLimit",&WamDriver::WamRequestSeaCtrlTorqLimit,&wam);

  owd::WamSetupSeaCtrl kpcmd;
  n.subscribe("wam_seactrl_setkp", kpcmd, &WamDriver::wam_seactrl_setkp_callback, &wam, &kpcmd, 10); 
  n.advertise<owd::IndexedJointValues>("wam_seactrl_curkp", 10);
  n.advertiseService("WamRequestSeaCtrlKp",&WamDriver::WamRequestSeaCtrlKp,&wam);

  owd::WamSetupSeaCtrl kdcmd;
  n.subscribe("wam_seactrl_setkd", kdcmd, &WamDriver::wam_seactrl_setkd_callback, &wam, &kdcmd, 10); 
  n.advertise<owd::IndexedJointValues>("wam_seactrl_curkd", 10);
  n.advertiseService("WamRequestSeaCtrlKd",&WamDriver::WamRequestSeaCtrlKd,&wam);

  owd::WamSetupSeaCtrl kicmd;
  n.subscribe("wam_seactrl_setki", kicmd, &WamDriver::wam_seactrl_setki_callback, &wam, &kicmd, 10); 
  n.advertise<owd::IndexedJointValues>("wam_seactrl_curki", 10);
  n.advertiseService("WamRequestSeaCtrlKi",&WamDriver::WamRequestSeaCtrlKi,&wam);

  usleep(100000);

  wam.publishAllSeaSettings();
#endif

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
