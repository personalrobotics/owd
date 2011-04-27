/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef GFEPLUGIN_HH
#define GFEPLUGIN_HH

#include <openwam/Plugin.hh>
#include <openwam/Trajectory.hh>
#include <gfe_owd_plugin/ApplyForce.h>
#include <gfe_owd_plugin/StopForce.h>
#include <ros/ros.h>


class GfePlugin : public OWD::Plugin {
public:

  GfePlugin();

  ~GfePlugin();

  //  virtual void Publish();

private:
  ros::Publisher  pub_info;
};


class ApplyForceTraj : public OWD::Trajectory {
public:

  /// Holds the endpoint at the current position while applying
  /// force in the specified direction.  Endpoint is free to move
  /// in the direction of the force
  ApplyForceTraj(double x, double y, double z, double f);
  ~ApplyForceTraj();

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);

  static bool ApplyForce(gfe_owd_plugin::ApplyForce::Request &req,
			 gfe_owd_plugin::ApplyForce::Response &res);

  bool StopForce(gfe_owd_plugin::StopForce::Request &req,
		 gfe_owd_plugin::StopForce::Response &res);

  /// Static member for handling the ROS service calls
  static ros::ServiceServer ss_ApplyForce;
  static ros::ServiceServer ss_StopForce;

  /// functions for starting up and shutting down the service
  static bool Register();
  static void Shutdown();

private:
  double x, y, z, f;
  bool stopforce;
};


#endif // GFEPLUGIN_HH
