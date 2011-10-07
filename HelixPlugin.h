/***********************************************************************

Author:        Kevin Knott (SRI), 2011

File:          HelixPlugin.cpp

Description:   This OWD Plugin creates a helical trajectory in the
               z-direction. It will stop upon force greater than the
               #defined value of Z_THRESHOLD in the z-direction. 

Note:          You are free to modify and reuse this code without
               restrictions.

Instructions:  To run this plugin, after typing rosmake in the folder 
               intel-pkg/gfe_owd_plugin, call the service StartHelixTraj with 
               the two parameters for the amplitude and pitch of the helix. To
               run the trajectory with a radius of 5 cm and a pitch of 10 cm,

               Ex. rosservice call right/owd/StartHelixTraj .05 .10

               To stop the plugin manually, call the service StopHelixTraj. 
               No parameters are needed when cancelling the trajectory. 

               Ex. rosservice call right/owd/StopHelixTraj
 
**********************************************************************/

#ifndef HELIXPLUGIN_H
#define HELIXPLUGIN_H

#include "GfePlugin.hh"
#include <openwam/Trajectory.hh>
#include <tf/transform_listener.h>
#include "openwam/Kinematics.hh"
#include <gfe_owd_plugin/StartHelixTraj.h>
#include <gfe_owd_plugin/StopHelixTraj.h>
#include <cmath>
#include <time.h>

#define HP_VERBOSE              // Comment out to eliminate extra printing of values
//#define HP_SAFE                 // Comment out to eliminate singularity check
#define NDOF 7                  // Number of degrees of freedom
#define PI 3.1415926535         // Apple PI, to 10 decimal places
#define AMPLITUDE_MAX 0.1      // Maximum amplitude of helix
#define PITCH_MAX 0.1           // Maximum pitch of helix
#define VELOCITY_LIMIT 0.5     // Maximum joint velocity in m/s
#define FREQUENCY 1           // Cycles/second of helix
#define Z_THRESHOLD 20           // Newtons of force for Z-direction threshold
#define DISTANCE_THRESHOLD .75  // Maximum distance in meters to travel in z-direction

// Class for transforming joint coordinate frames to wam0
class JointsToTF
{
 public:
  btTransform wam_tf_base[7];
  JointsToTF();
  ~JointsToTF();
  void GetTF(std::vector<double> &Joints, std::vector<btTransform> &Transforms);
};

// Class for creating helical motion trajectory in z-direction of hand
class HelixTraj : public OWD::Trajectory, public JointsToTF {
 public:

  HelixTraj(double amplitude_in,double pitch_in);
  ~HelixTraj();

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);
  static bool AddTrajectory(gfe_owd_plugin::StartHelixTraj::Request &req,
			    gfe_owd_plugin::StartHelixTraj::Response &res);

  static bool Register();
  static void Shutdown();

 private:
  double amplitude;
  double pitch; 
  R6 vel_des; 
  double JointLimits[NDOF][2];  
  OWD::JointPos zeros; 
  bool stophelixtraj;
  std::vector<tf::Transform> wamTransforms;

  OWD::JointPos prev_pos;
  OWD::JointPos prev_vel;
  OWD::JointPos prev_acc;

  OWD::JointPos joints;

  void HelixTrajImplementation(OWD::Trajectory::TrajControl &tc, double dt);
  bool CheckExceedJointLimits(OWD::JointPos position);
  bool StopHelixTraj(gfe_owd_plugin::StopHelixTraj::Request &req,
		     gfe_owd_plugin::StopHelixTraj::Response &res);

  ros::ServiceServer ss_StopHelixTraj;
  static ros::ServiceServer ss_Add; 

};
 

#endif // HELIXPLUGIN_H
