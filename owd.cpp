/***********************************************************************

  Copyright 2008-2010 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

  This file is part of owd.

  owd is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  owd is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***********************************************************************/

#include <ros/ros.h>
#include <ros/time.h>
#include <pr_msgs/WAMState.h>
#include <pr_msgs/IndexedJointValues.h>
#include <pr_msgs/WamSetupSeaCtrl.h>
#include "openwamdriver.h"
#include "bhd280.hh"
#include "ft.hh"
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
  std::string hand_type;
  bool forcetorque;
  bool modified_j1;
  n.param("calibration_file",calibration_filename,std::string("wam_joint_calibrations"));
  n.param("canbus_number",canbus_number,0);
  n.param("hand_type",hand_type,std::string("none"));
  n.param("forcetorque_sensor",forcetorque,false);
  n.param("modified_j1",modified_j1,false);

  ROS_DEBUG("Using CANbus number %d",canbus_number);

  int BH_model(0);
  bool tactile(false);
  if (! hand_type.compare(0,3,"280")) {
    BH_model=280;
    if (! hand_type.compare("280+TACT")) {
      tactile=true;
    }
  } else if (! hand_type.compare(0,3,"260")) {
    BH_model=260;
  }

  WamDriver wam(canbus_number,BH_model,forcetorque,tactile);
  if (modified_j1) {
    wam.SetModifiedJ1(true);
  }
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
  
#ifndef OWDSIM
  BHD_280 *bhd(NULL);
  if (BH_model == 280) {
    bhd= new BHD_280(&(wam.bus));
  }

  FT *ft(NULL);
  if (forcetorque) {
    ft = new FT(&(wam.bus));
  }
#endif // OWDSIM


#ifdef BUILD_FOR_SEA
  usleep(100000);
  wam.publishAllSeaSettings();
#endif // BUILD_FOR_SEA

  ROS_DEBUG("Creating timer and spinner threads");
  ros::Timer wam_timer = n.createTimer(ros::Duration(0.1), &WamDriver::Pump, &wam);
#ifndef OWDSIM
  ros::Timer bhd_timer;
  if (bhd) {
    bhd_timer = n.createTimer(ros::Duration(0.1), &BHD_280::Pump, bhd);
  }
  ros::Timer ft_timer;
  if (ft) {
    ft_timer = n.createTimer(ros::Duration(0.1), &FT::Pump, ft);
  }
#endif // OWDSIM
  ros::MultiThreadedSpinner s(3);
  ROS_DEBUG("Spinning");
  ros::spin(s);
  ROS_DEBUG("Done spinning; exiting");
  wam.bus.~CANbus();
  ROS_FATAL("dumped CANbus logs to candata.log");
  exit(0);
}
