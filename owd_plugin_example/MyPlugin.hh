/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

  This ROS package serves as a sample for how to make an OWD plugin.
  For more information about OWD, please see the webpage
  http://personalrobotics.intel-research.net/intel-pkg/owd/html/index.html

  You are free to modify and reuse this code without restrictions.

**********************************************************************/

#ifndef MYPLUGIN_HH
#define MYPLUGIN_HH

#include <openwam/Plugin.hh>
#include <openwam/Trajectory.hh>
#include <owd_plugin_example/AddMyTrajectory.h>
#include <ros/ros.h>


/// We will subclass the OWD::Plugin class so that we can have internal
/// access to WAM variables and motion commands.
class MyPlugin : public OWD::Plugin {
public:

  /// our constructor will also create a ROS topic for publishing its
  /// own data
  MyPlugin();

  /// the destructor simply shuts down the ROS topic
  ~MyPlugin();

  /// By overriding the Publish() function in the base class we can
  /// publish messages to our own clients.  OWD will call our Publish
  /// function at 10hz (or whatever the OWD publish_frequency parameter
  /// is set to).
  virtual void Publish();

private:
  ros::Publisher  pub_info;
};


/// Here we subclass the OWD::Trajectory class so that we can create
/// our own trajectory type.
class MyTraj : public OWD::Trajectory {
public:

  /// Constructor will be called when we have a request to make one of
  /// these trajectories.  It will perform any precalculations that
  /// the evaluate() function needs later.  If the constructor cannot
  /// create a trajectory with the supplied arguments, it throws a
  /// const char * that is caught by the AddTrajectory function
  MyTraj(int joint, double torque);

  /// The only base class function we need to override is the
  /// evaluate_abs() function, which is what OWD will call at 500Hz while
  /// the trajectory is active.  This function is the "meat" of what
  /// makes this Trajectory different from the built-in ones.
  virtual void evaluate_abs(OWD::Trajectory::TrajControl &tc, double t);

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);

  /// The AddTrajectory() function is static so that it can be called
  /// by the ROS service callback before a particular trajectory instance
  /// exists.  It's the responsibility of the AddTrajectory function to 
  /// actually create a MyTraj instance and add it to OWD.  The AddTrajectory
  /// function does not necessarily have to be part of the MyTraj class, but
  /// keeping it in the class is a nice way to have all the trajectory-
  /// related functions in a single place.
  static bool AddTrajectory(owd_plugin_example::AddMyTrajectory::Request &req,
			    owd_plugin_example::AddMyTrajectory::Response &res);

  /// Static member for handling the ROS service calls
  static ros::ServiceServer ss_Add;

  /// Definining our own Register and Shutdown functions is a clean way
  /// to handle the startup/shutdown procedure when the plugin is
  /// loaded.  These are called from the register_owd_plugin and
  /// unregister_owd_plugin functions.
  static bool Register();
  static void Shutdown();

private:
  int joint;
  double torque;
};


#endif // MYPLUGIN_HH
