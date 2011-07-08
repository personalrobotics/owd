/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef APPLYFORCETRAJ_HH
#define APPLYFORCETRAJ_HH

#include "GfePlugin.hh"
#include <openwam/Trajectory.hh>
#include <gfe_owd_plugin/ApplyForce.h>
#include <gfe_owd_plugin/StopForce.h>
#include <pr_msgs/SetStiffness.h> // for debugging the force control
#include <queue>

class ApplyForceTraj : public OWD::Trajectory {
public:

  /// Holds the endpoint at the current position while applying
  /// force in the specified direction.  Endpoint is free to move
  /// in the direction of the force
  ApplyForceTraj(R3 direction, double force);
  ~ApplyForceTraj();

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);

  static bool ApplyForce(gfe_owd_plugin::ApplyForce::Request &req,
			 gfe_owd_plugin::ApplyForce::Response &res);

  static bool SetForcePGain(pr_msgs::SetStiffness::Request &req,
		      pr_msgs::SetStiffness::Response &res);

  static bool SetForceDGain(pr_msgs::SetStiffness::Request &req,
		      pr_msgs::SetStiffness::Response &res);

  static bool SetForceFactor(pr_msgs::SetStiffness::Request &req,
			     pr_msgs::SetStiffness::Response &res);

  bool StopForce(gfe_owd_plugin::StopForce::Request &req,
		 gfe_owd_plugin::StopForce::Response &res);

  /// functions for starting up and shutting down the service
  static bool Register();
  static void Shutdown();

private:
  R3 workspace_force();
  double limit_force_correction_movement(double correction_distance);

  SE3 endpoint_target;
  R3 force_direction;
  R6 forcetorque_vector;
  R3 last_endpoint_velocity, last_endpoint;
  static double cartesian_vel_limit; // m/s
  static const double joint_vel_limit=0.03927; // rad/s; .39 = 90deg/sec
  static const double hand_mass=1.405; // kg
  double correction_motion_sum;
  std::queue<double> correction_distances;
  std::queue<OWD::JointPos> jointpositions;
  std::queue<double> times;
  double time_sum;
  static const double time_window = 0.1; // seconds
  double last_force_error;
  bool stopforce;
  static const unsigned int FT_window_size=64;
  ros::ServiceServer ss_StopForce;
  static double force_gain_kp, force_gain_kd, xforce;
  
  /// Static members for handling the ROS service calls
  static ros::ServiceServer ss_ApplyForce;
  static ros::ServiceServer ss_SetForcePGain, ss_SetForceDGain,
    ss_SetForceFactor;
  
};

#endif // APPLYFORCETRAJ_HH
