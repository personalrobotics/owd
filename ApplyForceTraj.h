/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef APPLYFORCETRAJ_HH
#define APPLYFORCETRAJ_HH

#include "HybridPlugin.h"
#include "ForceController.h"
#include <openwam/Trajectory.hh>
#include <owd_msgs/ApplyForce.h>
#include <owd_msgs/StopForce.h>
#include <owd_msgs/SetJointOffsets.h> // for changing force gains
#include <queue>
#include "Vibration.h"

class ApplyForceTraj : public OWD::Trajectory {
public:

  /// Holds the endpoint at the current position while applying
  /// force in the specified direction.  Endpoint is free to move
  /// in the direction of the force
  ApplyForceTraj(R3 direction, double force, double distance_limit = 0.1);
  ~ApplyForceTraj();

  virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t);

  static bool ApplyForce(owd_msgs::ApplyForce::Request &req,
			 owd_msgs::ApplyForce::Response &res);

  bool StopForce(owd_msgs::StopForce::Request &req,
		 owd_msgs::StopForce::Response &res);

  static bool SetForceGains(owd_msgs::SetJointOffsets::Request &req,
                            owd_msgs::SetJointOffsets::Response &res);

  void SetVibration(double hand_x, double hand_y, double hand_z,
		    double amplitude, double frequency);

  /// functions for starting up and shutting down the service
  static bool Register();
  static void Shutdown();

  OWD::JointPos limit_excursion_and_velocity(double travel);
  double limit_force_correction_movement(double correction_distance);
  bool clamp_torques(OWD::JointPos &torques);

  SE3 endpoint_target;
  R3 force_direction;
  R6 forcetorque_vector;
  R3 last_endpoint_velocity, last_endpoint;
  static const double joint_vel_limit=0.03927; // rad/s; .39 = 90deg/sec
  static const double hand_mass=1.405; // kg
  std::queue<OWD::JointPos> jointpositions;
  std::queue<R3> endpositions;
  double last_time;
  double last_force_error;
  bool stopforce;
  double distance_limit;
  double last_travel;  // for velocity estimate
  ros::ServiceServer ss_StopForce;
  static ForceController force_controller;
  Butterworth<double> velocity_filter;
  Vibration *vibration;
  static double velocity_damping_gain;
  static ApplyForceTraj *current_traj;
  double rotational_leeway;
  SO3 current_endpoint_target;

  /// Static members for handling the ROS service calls
  static ros::ServiceServer ss_ApplyForce;
  static ros::ServiceServer ss_SetForceGains;
  
};

#endif // APPLYFORCETRAJ_HH
