/***********************************************************************

Author:        Kevin Knott (SRI), 2011

File:          MoveDirection.h

Description:   This OWD Plugin creates a straight forward trajectory 
               in any arbitrary direction (x, y, or z) in the coordinate
               system of the end-effector (workspace). The user will 
               input the distance that they would like to move in the 
               given direction in meters. The trajectory will
               stop upon force greater than the #defined value of the 
               FORCE_THRESHOLD depending upon the direction of motion. 
               The force must be in the direction orthogonal to that of 
               the direction of motion in order to stop the arm. 
               The user will also input a velocity value for the speed
               of the arm in meters per second. The last value to be input
               is a boolean flag to determine if the trajectory should be 
               run with or without compliance. Without compliance, the arm
               will move in the direction that the palm is facing before 
               the trajector is called, and there will be no adjustment. 
               If compliance is being utilized, the coordinate system of 
               the palm will be updated regularly, so if the palm should
               change direction (either intentionally or not), the direction
               of the trajectory will be updated to the new coordinate system
               of the palm (useful for opening a door knob, for instance). 

Note:          You are free to modify and reuse this code without
               restrictions.

Instructions:  To run this plugin, after typing rosmake in the folder 
               intel-pkg/gfe_owd_plugin, call the service StartMoveDirection
               with the 4 parameters for the direction, distance, and
               velocity along with the desired compliance to include in the 
               trajectory. To move the arm in the Z-direction (first
               parameter = 0 0 1), a distance of 5 cm (second parameter is 0.05 m), 
               at a speed of 1 cm per second (third parameter is 0.01) and
               to run without compliance (fourth parameter is 0), enter the 
               following command. 

               Ex: rosservice call /right/owd/StartMoveDirection 0 0 1 0.05 0.01 0

               To stop the plugin manually, call the service StopMoveDirection. 
               No parameters are needed when cancelling the trajectory.   

               Ex: rosservice call /right/owd/StopMoveDirection
  
**********************************************************************/

#ifndef MOVEDIRECTION_H  
#define MOVEDIRECTION_H

#include "GfePlugin.hh"
#include "HelixPlugin.h"
#include <openwam/Trajectory.hh>
#include <tf/transform_listener.h>
#include "openwam/Kinematics.hh"
#include <gfe_owd_plugin/StartMoveDirection.h>
#include <gfe_owd_plugin/StopMoveDirection.h>
#include <cmath>
#include <time.h>

//#define MD_VERBOSE             // Comment out to eliminate printing joint vel & pos values
//#define MD_SAFE                // Uncomment to look for singularities and limit joint movement
#define NDOF 7                   // Number of degrees of freedom
#define MAX_DISTANCE 1           // Maximum distance to move in given direction in meters
#define FORCE_THRESHOLD 100      // Newtons of force for threshold in case of collision
#define VELOCITY_LIMIT 0.5      // Maximum allowable joint velocity in radians/second
#define VELOCITY_INPUT_LIMIT 1 // Maximum allowable input velocity in meters/second

// Class for creating straight trajectory in x, y or z-direction of wam7
class MoveDirection : public OWD::Trajectory, public JointsToTF {
 public:

  MoveDirection(double direction_in_x, double direction_in_y, double direction_in_z,
                double distance_in, double velocity_in, bool compliance_in);
  ~MoveDirection();

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);
  static bool AddTrajectory(gfe_owd_plugin::StartMoveDirection::Request &req,
			    gfe_owd_plugin::StartMoveDirection::Response &res);

  static bool Register();
  static void Shutdown();

 private:
  bool compliance;                // Whether or not to run the program with compliance
  btVector3 direction;            // Holds input direction request
  double distance;                // Holds input distance request
  double velocity;                // Holds input velocity request
  double distance_traveled;       // Holds current value of distance traveled 
  R6 vel_des;                     // Holds desired vel & angular vel of end-effector in wam0
  double JointLimits[NDOF][2];    // Matrix holding joint limit values
  OWD::JointPos zeros;            // Vector for zeroing out pos, vel, accel
  bool stopmovedirection;         // Boolean flag alerting a reason to stop the trajectory
  
  // Initial and current coordinates of the end-effector to be used in distance calculation
  R3 initial_endpoint_origin;
  R3 endpoint_origin;

  // Position, Velocity, and acceleration to be used in tracking past/current values
  OWD::JointPos prev_pos;
  OWD::JointPos prev_vel;
  OWD::JointPos prev_acc;

  // Transforms from wam? to wam0/base frame
  std::vector<tf::Transform> wamTransforms;

  // Private class functions and services
  void MoveDirectionImplementation(OWD::Trajectory::TrajControl &tc, double dt);
  bool CheckExceedJointLimits(OWD::JointPos position);
  bool StopMoveDirection(gfe_owd_plugin::StopMoveDirection::Request &req,
                         gfe_owd_plugin::StopMoveDirection::Response &res);

  ros::ServiceServer ss_StopMoveDirection;
  static ros::ServiceServer ss_Add; 

};
 

#endif // MOVEDIRECTION_H
