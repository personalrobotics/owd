/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef JACOBIANTEST_HH
#define JACOBIANTEST_HH

#include "GfePlugin.hh"
#include <openwam/Trajectory.hh>
#include <owd_msgs/Reset.h> // for debugging the Jacobian Pseudo-Inverse

class JacobianTestTraj : public OWD::Trajectory {
public:

  JacobianTestTraj();
  ~JacobianTestTraj();

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);

  static bool JacobianTest(owd_msgs::Reset::Request &req,
			   owd_msgs::Reset::Response &res);

  bool StopTraj(owd_msgs::Reset::Request &req,
		owd_msgs::Reset::Response &res);

  inline double random_joint_value(double low, double hi) {
    return random() / RAND_MAX * (hi-low) + low;
  }

  /// functions for starting up and shutting down the service
  static bool Register();
  static void Shutdown();

private:
  ros::ServiceServer ss_StopTraj;
  bool stoptraj;

  /// Static members for handling the ROS service calls
  static ros::ServiceServer ss_JacobianTest;
  
};


#endif // JACOBIANTEST_HH
