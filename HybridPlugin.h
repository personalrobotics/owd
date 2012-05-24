/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef HYBRIDPLUGIN_HH
#define HYBRIDPLUGIN_HH

#include <openwam/Plugin.hh>
#include <openwam/DataRecorder.cc>
#include <openwam/Butterworth.h>
#include <owd_msgs/MoveHand.h>
#include <owd_msgs/Reset.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <pthread.h>
#include <sys/time.h>

// #define SIMULATION
#ifdef SIMULATION
#include <owd_plugins/ApplyForceDebug.h>
#include <owd_msgs/Joints.h> // for debugging the Jacobian
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
  bool StopTraj(owd_msgs::Reset::Request &req,
		owd_msgs::Reset::Response &res);
  bool PowerGrasp(owd_msgs::MoveHand::Request &req,
		  owd_msgs::MoveHand::Response &res);
  R6 workspace_forcetorque();
  
  inline unsigned long long time_now_usec() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return (tv.tv_sec * 1e6 + tv.tv_usec);
  }
    

private:
  ros::Publisher pub_net_force;
  ros::Publisher pub_tactile_debug;
  ros::ServiceServer ss_StopTraj, ss_PowerGrasp;
  pthread_mutex_t recorder_mutex,
    pub_mutex;

};

extern HybridPlugin *hybridplug;

#endif // HYBRIDPLUGIN_HH
