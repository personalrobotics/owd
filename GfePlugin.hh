/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef GFEPLUGIN_HH
#define GFEPLUGIN_HH

#include <openwam/Plugin.hh>
#include <openwam/DataRecorder.cc>
#include <openwam/Butterworth.h>
#include <pr_msgs/MoveHand.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <pthread.h>

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
  static std_msgs::Float64MultiArray tactile_debug;
 
  bool write_log_file;
  DataRecorder<double> *recorder;
  void log_data(const std::vector<double> &data);
  bool write_recorder_data();
  bool flush_recorder_data;
  bool PowerGrasp(pr_msgs::MoveHand::Request &req,
		  pr_msgs::MoveHand::Response &res);
  R6 workspace_forcetorque();
  Butterworth<R6> ft_filter;
  
private:
  ros::Publisher pub_net_force;
  ros::Publisher pub_tactile_debug;
  ros::ServiceServer ss_PowerGrasp;
  pthread_mutex_t recorder_mutex,
    pub_mutex;

};

extern GfePlugin *gfeplug;

#endif // GFEPLUGIN_HH
