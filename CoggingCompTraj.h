/***********************************************************************

  Copyright 2013 Carnegie Mellon University
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef COGGINGCOMPTRAJ_H
#define COGGINGCOMPTRAJ_H

#include <openwam/Plugin.hh>
#include <openwam/Trajectory.hh>
#include <owd_plugins/CoggingComp.h>
#include <owd_plugins/CogSample.h>
#include <ros/ros.h>


/// Here we subclass the OWD::Trajectory class so that we can create
/// our own trajectory type.
class CoggingCompTraj : public OWD::Trajectory {
public:

  CoggingCompTraj(int joint,
                  double max_torque,
                  double max_step,
                  double hold_duration,
                  int num_samples);

  virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t);

  static bool AddTrajectory(owd_plugins::CoggingComp::Request &req,
			    owd_plugins::CoggingComp::Response &res);

  /// Static member for handling the ROS service calls
  static ros::ServiceServer ss_Add;

  static bool Register();
  static void Shutdown();

private:
  int joint;
  double current_torque;
  double max_torque;
  double max_step;
  double hold_duration;
  int num_samples;

  bool first;
  int sample_count;
  int last_position;
  double sample_time;
  double done_time;
  static owd_plugins::CoggingComp::Response response;
  static bool running;
};


#endif // COGGINGCOMPTRAJ_H
