/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Kyle Strabala <strabala@cmu.edu>

**********************************************************************/

#include "HybridPlugin.h"

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <phantom_omni/OmniState.h>
#include <owd_msgs/StartTeleop.h>

#include <openwam/Plugin.hh>
#include <openwam/Trajectory.hh>
#include <ros/ros.h>

#include <LinearMath/btQuaternion.h>

#include <pthread.h>

class Teleop : public OWD::Trajectory {
public:

  Teleop(int mode_in, std::string input_topic);
  ~Teleop();
  void PrintModeError(int mode_in);
  void SetupTrajForSpacenavTeleop(std::string input_topic);
  void SetupTrajForOmniTeleop(std::string input_topic);

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);
  virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double dt);
  void TrajForSpacenavTeleop(OWD::Trajectory::TrajControl &tc, double dt);
  void SpacenavCallback(sensor_msgs::Joy msg);
  void TrajForOmniTeleop(OWD::Trajectory::TrajControl &tc, double dt);
  void OmniCallback(phantom_omni::OmniState msg);
  
  static bool StartTeleopViaSpacenavCallback(
        owd_plugins::StartTeleop::Request &req,
        owd_plugins::StartTeleop::Response &res);
  static bool StartTeleopViaOmniCallback(
        owd_plugins::StartTeleop::Request &req,
        owd_plugins::StartTeleop::Response &res);
  static bool AddTrajectory(owd_plugins::StartTeleop::Request &req,
                            owd_plugins::StartTeleop::Response &res,
                            int mode_in);
  
  static ros::ServiceServer ss_spacenav;
  static ros::ServiceServer ss_omni;

  static bool Register();
  static void Shutdown();
  
  static const int MODE_NONE = 0;
  static const int MODE_SPACENAV = 1;
  static const int MODE_OMNI = 2;

private:
  int mode;
  
  OWD::JointPos zeros;
  
  ros::Publisher joint_state_pub;
  double last_joint_state_pub_time;
  
  ros::Subscriber spacenav_sub;
  sensor_msgs::Joy last_spacenav_msg;
  double last_spacenav_msg_time;
  
  ros::Subscriber omni_sub;
  phantom_omni::OmniState last_omni_msg;
  double last_omni_msg_time;
  bool clutch; //false if omni and robot are connected
  btVector3 clutch_omni_hand, clutch_robot_hand;
  btQuaternion clutch_omni_quat, clutch_robot_quat;
  
  OWD::JointPos prev_pos;
  OWD::JointPos prev_vel;
  OWD::JointPos prev_acc;
  
  pthread_mutex_t spacenav_msg_mutex;
  pthread_mutex_t omni_msg_mutex;
};


