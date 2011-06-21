/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef GFEPLUGIN_HH
#define GFEPLUGIN_HH

#include <openwam/Plugin.hh>
#include <openwam/Trajectory.hh>
#include <openwam/MacJointTraj.hh>
#include <gfe_owd_plugin/ApplyForce.h>
#include <gfe_owd_plugin/StopForce.h>
#include <gfe_owd_plugin/OpenDoor.h>
#include <pr_msgs/SetStiffness.h> // for debugging the force control
#include <pr_msgs/Reset.h> // for debugging the Jacobian Pseudo-Inverse
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>

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
  ros::Publisher  pub_net_force;
};

extern GfePlugin *gfeplug;

#endif // GFEPLUGIN_HH
