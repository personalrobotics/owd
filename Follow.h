/***********************************************************************

  Copyright 2011 Carnegie Mellon University

  Author:        Kyle Strabala <strabala@cmu.edu>

  Modifications: Kevin Knott, SRI

  Description:   This gfe_owd_plugin can be used for teleoperation of the
                 WAM arm using an Xbox360 controller. This operation of the 
                 arm is done in the WAM0 coordinate frame and operates
                 the velocities (vx,vy,vz,wx,wy,wz) of the end-effector.
                 The end-effector velocities are then transformed into 
                 velocities of the joints and eventually positions of the
                 joint angles. 

                 The joystick has two operating modes, VELOCITY or 
                 ANGULAR_VELOCITY. When being used in VELOCITY mode,
                 the x-axis of the left joystick controls vx, the
                 y axis of the left joystick controls vy, and the y-axis of 
                 the right joystick controls vz. When in ANGULAR_VELOCITY 
                 mode, the x-axis of the left joystick controls wx, the
                 y-axis of the left joystick controls wy, and the y-axis of
                 the right joystick controls vz. While splitting into two 
                 modes is not entirely necessary, this gives the best control
                 of the arm using the analog joysticks rather than any of
                 digital button pads, which provides very jerky movement. 

                 Additionally, the joystick can be run with or without a 
                 high frequency noise component added to the end-effector,
                 for tasks such as peg-in-hole or other localizing
                 operations. 
                  
  Instructions:  To run this plugin:
                 1. type rosmake in gfe_owd_plugin directory
                 2. type rosmake in joy directory
                 3. if OWD is already running, shut it down
                 4. type roslaunch owd.launch in rearm/sri_stacks/launch 
                    to restart OWD
                 5. type rosservice call /right/owd/StartFollow 1 to 
                    start the Follow Trajectory and joystick control
                 6. type rosservice call /right/owd/StopFollow 

                 To modify values in the plugin:
                 1. to change the speed of the arm, change the 
                    VELOCITY_LIMIT #define
                 2. to change the amplitude of the noise, change the 
                    JITTER_SCALE value (at your own discretion)

**********************************************************************/

#include "GfePlugin.hh"
#include <openwam/Trajectory.hh>

#include <joy/Joy.h>
#include <sensor_msgs/JointState.h>
#include <pr_msgs/ArmConfigCheck.h>
#include <gfe_owd_plugin/StartFollow.h>
#include <gfe_owd_plugin/StopFollow.h>
#include <pthread.h>
#include <cmath>
#include <time.h>

#define FULL_SCALE_JOYSTICK .60
#define NDOF 7
#define VELOCITY_LIMIT 0.5
#define JITTER_SCALE 0.05
#define counter_limit 10          //***TODO, switch this value to input

enum JoyToggle {VELOCITY, ANGULAR_VELOCITY};

class Follow : public OWD::Trajectory {
 public:

  Follow(int mode_in);
  ~Follow();

  void SetupTrajForJoyTeleop();

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);
  void TrajForJoyTeleop(OWD::Trajectory::TrajControl &tc, double dt);
  void JoyCallback(joy::Joy msg);
  bool CheckExceedJointLimits(OWD::JointPos position);
  static void *StartRosCommThread(void *data);
  void RosComm();
  static bool AddTrajectory(gfe_owd_plugin::StartFollow::Request &req,
			    gfe_owd_plugin::StartFollow::Response &res);
  static bool Register();
  static void Shutdown();

 private:
  int mode;                         //For holding mode_joy (1) or mode_none (0)
  OWD::JointPos zeros;              //For zeroing out pos, vel, acc, t
  double JointLimits[NDOF][2];      //Array for holding values of joint limits
  JoyToggle whatWeAreControlling;   //Control using velocity or angular velocity
  bool jitter;                      //Jitter the end effector or not
  R6 vel_des;                       //Desired velocity of end-effector
  bool stopfollow;                  //Flag for shutting down trajectory
  bool problem;                     //Flag for alerting a problem state
  
  ros::Publisher joint_state_pub;   
  double last_joint_state_pub_time;
  
  ros::Subscriber joy_sub;
  joy::Joy last_joy_msg;
  double last_joy_msg_time;
  
  ros::ServiceClient openrave_service;
  OWD::JointPos last_good_pos;
    
  //Declare a pthread topic
  pthread_t ros_comm_thread;
  
  OWD::JointPos prev_pos;
  OWD::JointPos prev_vel;
  OWD::JointPos prev_acc;

  //Declare mutual exclusion variable (shared data)
  pthread_mutex_t joy_msg_mutex;

  bool StopFollow(gfe_owd_plugin::StopFollow::Request &req,
		  gfe_owd_plugin::StopFollow::Response &res);

  void Jitter(OWD::JointPos &noise_pos, OWD::JointPos &noise_vel, 
              OWD::JointPos &noise_accel);

  static ros::ServiceServer ss_Add;
  ros::ServiceServer ss_StopFollow;

};


