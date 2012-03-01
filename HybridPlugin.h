/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef HYBRIDPLUGIN_HH
#define HYBRIDPLUGIN_HH

#include <openwam/Plugin.hh>
#include <openwam/DataRecorder.cc>
#include <openwam/Butterworth.h>
#include <pr_msgs/MoveHand.h>
#include <pr_msgs/Reset.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <pthread.h>

// #define SIMULATION
#ifdef SIMULATION
#include <owd_plugins/ApplyForceDebug.h>
#include <pr_msgs/Joints.h> // for debugging the Jacobian
#endif // SIMULATION


class HybridPlugin : public OWD::Plugin {
public:

  HybridPlugin();

  ~HybridPlugin();

  virtual void Publish();

  static std_msgs::Float64MultiArray net_force;
  static std_msgs::Float64MultiArray tactile_debug;
 
  bool write_log_file;
  DataRecorder<double> *recorder;
  bool flush_recorder_data;
  OWD::Trajectory *current_traj;
  void log_data(const std::vector<double> &data);
  bool write_recorder_data();
  bool StopTraj(pr_msgs::Reset::Request &req,
		pr_msgs::Reset::Response &res);
  bool PowerGrasp(pr_msgs::MoveHand::Request &req,
		  pr_msgs::MoveHand::Response &res);
  R6 workspace_forcetorque();
  
private:
  ros::Publisher pub_net_force;
  ros::Publisher pub_tactile_debug;
  ros::ServiceServer ss_StopTraj, ss_PowerGrasp;
  pthread_mutex_t recorder_mutex,
    pub_mutex;

};

extern HybridPlugin *hybridplug;

#endif // HYBRIDPLUGIN_HH
