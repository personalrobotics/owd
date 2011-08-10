/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef WSTRAJ_HH
#define WSTRAJ_HH

#include "GfePlugin.hh"
#include <openwam/Trajectory.hh>
#include <openwam/ParabolicSegment.hh>
#include <gfe_owd_plugin/AddWSTraj.h>


class WSTraj : public OWD::Trajectory {
public:

  /// Holds the endpoint at the current position while applying
  /// force in the specified direction.  Endpoint is free to move
  /// in the direction of the force
  WSTraj(gfe_owd_plugin::AddWSTraj::Request &wst);
  virtual ~WSTraj();

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);

  static bool AddWSTraj(gfe_owd_plugin::AddWSTraj::Request &req,
			gfe_owd_plugin::AddWSTraj::Response &res);

  /// functions for starting up and shutting down the service
  static bool Register();
  static void Shutdown();

private:
  R6 driving_force;
  SE3 start_endpoint;
  R3 endpoint_translation;
  so3 endpoint_rotation;
  OWD::JointPos joint_change;
  double max_linear_vel, max_linear_accel,
    max_angular_vel, max_angular_accel;
  OWD::ParabolicSegment parseg;
  double vel_factor;

  /// Static members for handling the ROS service calls
  static ros::ServiceServer ss_AddWSTraj;
  
};

#endif // WSTRAJ_HH
