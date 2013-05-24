/***********************************************************************

  Copyright 2013 Carnegie Mellon University
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef TEACHANDPLAY_H
#define TEACHANDPLAY_H

#include <openwam/Plugin.hh>
#include <openwam/Trajectory.hh>
#include <owd_plugins/Teach.h>
#include <owd_plugins/StopTeach.h>
#include <owd_plugins/Play.h>
#include <owd_plugins/TrajPoint.h>
#include <ros/ros.h>


class TeachTraj : public OWD::Trajectory {
public:

  TeachTraj(int hz=100);
  ~TeachTraj();

  virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t);

  static bool Teach(owd_plugins::Teach::Request &req,
		    owd_plugins::Teach::Response &res);
  static bool StopTeach(owd_plugins::StopTeach::Request &req,
			owd_plugins::StopTeach::Response &res);

  /// Static members for handling the ROS service calls
  static ros::ServiceServer ss_Teach, ss_StopTeach;

  static bool Register();
  static void Shutdown();

private:
  static std::vector<owd_plugins::TrajPoint> taughttraj;
  static bool running, stopteach;
  double last_sample_time;
  double sample_interval;
};


class PlayTraj : public OWD::Trajectory {
public:

  PlayTraj(std::vector<owd_plugins::TrajPoint> traj);

  virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t);

  static bool Play(owd_plugins::Play::Request &req,
		   owd_plugins::Play::Response &res);

  /// Static member for handling the ROS service calls
  static ros::ServiceServer ss_Play;

  static bool Register();
  static void Shutdown();

private:
  std::vector<owd_plugins::TrajPoint> playtraj;
  unsigned int i;
};


#endif // TEACHANDPLAY_H
