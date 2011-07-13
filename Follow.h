/***********************************************************************

  Copyright 2011 Carnegie Mellon University
  Author: Kyle Strabala <strabala@cmu.edu>

**********************************************************************/

#include <kyle_owd_trajs/StartFollow.h>

#include "GfePlugin.hh"

#include <joy/Joy.h>
#include <sensor_msgs/JointState.h>
#include <pr_msgs/ArmConfigCheck.h>

#include <pthread.h>

class Follow : public OWD::Trajectory {
public:

  Follow(int mode_in);
  ~Follow();
  void SetupTrajForJoyTeleop();

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);
  void TrajForJoyTeleop(OWD::Trajectory::TrajControl &tc, double dt);
  void JoyCallback(joy::Joy msg);
  static void *StartRosCommThread(void *data);
  void RosComm();
  
  static bool AddTrajectory(kyle_owd_trajs::StartFollow::Request &req,
			                      kyle_owd_trajs::StartFollow::Response &res);

  static ros::ServiceServer ss_Add;

  static bool Register();
  static void Shutdown();

private:
  int mode;
  JointPos zeros;
  
  ros::Publisher joint_state_pub;
  double last_joint_state_pub_time;
  
  ros::Subscriber joy_sub;
  joy::Joy last_joy_msg;
  double last_joy_msg_time;
  
  ros::ServiceClient openrave_service;
  JointPos last_good_pos;
  bool problem;
  
  pthread_t ros_comm_thread;
  
  JointPos prev_pos;
  JointPos prev_vel;
  JointPos prev_acc;
  
  pthread_mutex_t joy_msg_mutex;
};


