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
#ifdef SIMULATION
#include <gfe_owd_plugin/ApplyForceDebug.h>
#include <gfe_owd_plugin/Jacobian.h>
#include <pr_msgs/Joints.h> // for debugging the Jacobian
#endif // SIMULATION
#include <ros/ros.h>


class GfePlugin : public OWD::Plugin {
public:

  GfePlugin();

  ~GfePlugin();

  virtual void Publish();

private:
  ros::Publisher  pub_jacobian;
#ifdef SIMULATION
  gfe_owd_plugin::Jacobian jacobian_msg;
#endif // SIMULATION
};


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

  bool StopForce(gfe_owd_plugin::StopForce::Request &req,
		 gfe_owd_plugin::StopForce::Response &res);


  /// functions for starting up and shutting down the service
  static bool Register();
  static void Shutdown();

private:
  SE3 endpoint_target;
  R3 force_direction;
  R6 forcetorque_vector;
  bool stopforce;
  ros::ServiceServer ss_StopForce;
#ifdef SIMULATION
  gfe_owd_plugin::ApplyForceDebug afdebug_msg;
  ros::Subscriber sub_setjoints;
  std::vector<double> joints;
  void setjoints_callback(const boost::shared_ptr<const pr_msgs::Joints> &newjoints);
  static ros::Publisher pub_AFDebug;
#endif // SIMULATION
  
  /// Static members for handling the ROS service calls
  static ros::ServiceServer ss_ApplyForce;
  
};


#endif // GFEPLUGIN_HH
