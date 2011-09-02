/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef GFEPLUGIN_HH
#define GFEPLUGIN_HH

#include <openwam/Plugin.hh>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>

// Added for Kyle's Follow trajectory
#include <openwam/Trajectory.hh>
#include <std_msgs/String.h>
#include <math.h>
#include <LinearMath/btQuaternion.h>

// #define SIMULATION
#ifdef SIMULATION
#include <gfe_owd_plugin/ApplyForceDebug.h>
#include <pr_msgs/Joints.h> // for debugging the Jacobian
#endif // SIMULATION


class GfePlugin : public OWD::Plugin {
public:

  GfePlugin();

  ~GfePlugin();

  virtual void Publish();

  static std_msgs::Float64MultiArray net_force;
 
private:
  ros::Publisher pub_net_force;
};

extern GfePlugin *gfeplug;

#endif // GFEPLUGIN_HH
