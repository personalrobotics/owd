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

  ros::Publisher wamstate_publisher = 
    n.advertise<pr_msgs::WAMState>("wamstate", 10);
  n.advertiseService("AddTrajectory",&WamDriver::AddTrajectory,&wam);
  n.advertiseService("SetStiffness",&WamDriver::SetStiffness,&wam);
  n.advertiseService("DeleteTrajectory",&WamDriver::DeleteTrajectory,&wam);
  n.advertiseService("PauseTrajectory",&WamDriver::PauseTrajectory,&wam);
  n.advertiseService("ReplaceTrajectory",&WamDriver::ReplaceTrajectory,&wam);
  n.advertiseService("SetSpeed",&WamDriver::SetSpeed,&wam);
  n.advertiseService("SetExtraMass",&WamDriver::SetExtraMass,&wam);
  n.advertiseService("GetArmDOF",&WamDriver::GetDOF,&wam);
  n.advertiseService("CalibrateJoints", &WamDriver::CalibrateJoints, &wam);
  n.subscribe("wamservo", 1, &WamDriver::wamservo_callback,&wam);

  // LLL
  pr_msgs::IndexedJointValues jtcmd;
  n.subscribe("wam_joint_targets", 10, &WamDriver::wamjointtargets_callback,&wam);
 
#ifdef BUILD_FOR_SEA
  n.subscribe("wam_seactrl_settl", 10, &WamDriver::wam_seactrl_settl_callback, &wam);
  n.advertise<pr_msgs::IndexedJointValues>("wam_seactrl_curtl", 10);
  n.advertiseService("WamRequestSeaCtrlTorqLimit",&WamDriver::WamRequestSeaCtrlTorqLimit,&wam);

  pr_msgs::WamSetupSeaCtrl kpcmd;
  n.subscribe("wam_seactrl_setkp", 10, &WamDriver::wam_seactrl_setkp_callback, &wam); 
  n.advertise<pr_msgs::IndexedJointValues>("wam_seactrl_curkp", 10);
  n.advertiseService("WamRequestSeaCtrlKp",&WamDriver::WamRequestSeaCtrlKp,&wam);

  pr_msgs::WamSetupSeaCtrl kdcmd;
  n.subscribe("wam_seactrl_setkd", 10, &WamDriver::wam_seactrl_setkd_callback, &wam); 
  n.advertise<pr_msgs::IndexedJointValues>("wam_seactrl_curkd", 10);
  n.advertiseService("WamRequestSeaCtrlKd",&WamDriver::WamRequestSeaCtrlKd,&wam);

  pr_msgs::WamSetupSeaCtrl kicmd;
  n.subscribe("wam_seactrl_setki", 10, &WamDriver::wam_seactrl_setki_callback, &wam); 
  n.advertise<pr_msgs::IndexedJointValues>("wam_seactrl_curki", 10);
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
    wam.Publish(wamstate_publisher);

    // let the driver update
    wam.Update();
    
    //
    // Sleep for the duration we specified above
    //
    sleep_duration.sleep();
  }
}
