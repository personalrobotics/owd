/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef FTCHECK_H
#define FTCHECK_H

#include "HybridPlugin.h"
#include <openwam/Trajectory.hh>
#include <owd_msgs/Reset.h>
#include <string.h>
#include <stdio.h>

class FTCheck : public OWD::Trajectory {
public:

  FTCheck(int testtime);
  ~FTCheck();

  virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t);

  static bool FTCheckSrv(owd_msgs::Reset::Request &req,
			 owd_msgs::Reset::Response &res);

  /// functions for starting up and shutting down the service
  static bool Register();
  static void Shutdown();

  /// Static members for handling the ROS service calls
  static ros::ServiceServer ss_FTCheck;
  
  int testtime, samples;
  std::vector<double> raw_max, raw_min,
    filtered_max, filtered_min,
    raw_sum, filtered_sum;
  static char summary[];
  static bool done;
};

#endif // FTCHECK_H
