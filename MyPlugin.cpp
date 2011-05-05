/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

  This ROS package serves as a sample for how to make an OWD plugin.
  For more information about OWD, please see the webpage
  http://personalrobotics.intel-research.net/intel-pkg/owd/html/index.html

  You are free to modify and reuse this code without restrictions.

**********************************************************************/

#include "MyPlugin.hh"
#include <ros/ros.h>
#include <std_msgs/String.h>

MyPlugin::MyPlugin() {
  // ROS has already been initialized by OWD, so we can just
  // create our own NodeHandle in the same namespace
  ros::NodeHandle n("~");

  // Advertise our ROS topic for publishing data
  pub_info = n.advertise<std_msgs::String>("MyPluginInfo",10);

  // Let our Trajectory class register itself
  if (!MyTraj::Register()) {
    throw "MyTraj trajectory failed to register";
  }
}

MyPlugin::~MyPlugin() {
  // Take down the ROS topic so that clients don't try to connect to it.
  pub_info.shutdown();

  // Also tell our Trajectory class to clean up.
  MyTraj::Shutdown();
}

/// Our Publish function will be automatically called by OWD at its own
/// publishing frequency (defined by the OWD publish_frequency parameter).
void MyPlugin::Publish() {
  
  // Just publish a simple string with same data from the WAM, as an
  // example.  Here the data is coming from the accessor function in the
  // OWD::Plugin base class.
  std_msgs::String myinfo;
  std::stringstream ss;
  ss << "MyPlugin: joint 1 is " << arm_position[0];
  myinfo.data = ss.str();
  pub_info.publish(myinfo);
}

// Allocation for the pointer that will hold the single instantiation
// of our plugin class.  We initialize it to NULL so that the register
// function can tell whether or not one has already been allocated.
MyPlugin *myplug = NULL;

// Our trajectory constructor simply does some bound checking on the
// arguments, and sets the start and end positions to the current
// position of the WAM.
// Note that we have to explicitly call the OWD::Trajectory constructor
// so that we can pass in the name of our trajectory for display in the
// trajectory queue.
MyTraj::MyTraj(int j, double t): OWD::Trajectory("MyTraj"), joint(j), torque(t) {
  if ((j<1) || (j>7)) {
    // this throw will be caught by the AddTrajectory function so that
    // we can report the error back to the client
    throw "Joint out of range";
  }
  if (fabs(t)>10) {
    throw "Torque limited to 10nm";
  }
  if (myplug) {
    // If the start position doesn't match the WAM's current position,
    // OWD will reject the trajectory.  Often the start position will be
    // sent in by the client through the service call, but in this case
    // we are just doing a pure torque trajectory at whatever our
    // current configuration is, so the start position doesn't matter.

    // note how we get the current position by using our plugin class.
    // if we didn't have an instantiation of our plugin class, we could
    // have also just called OWD::Plugin::target_arm_position(), since it
    // is a static class member.
    start_position=myplug->target_arm_position;
    end_position = start_position;
  }
}

// This is the handler for the ROS callbacks.  We register our own
// ROS service for getting requests from the client so that we have
// control over all the message parameters.  OWD itself doesn't know
// anything about this service, and won't know anything about our
// trajectory type until we create one and ask OWD to run it using the
// Trajectory::AddTrajectory function in the base class.
bool MyTraj::AddTrajectory(owd_plugin_example::AddMyTrajectory::Request &req,
			   owd_plugin_example::AddMyTrajectory::Response &res) {
  ROS_INFO("MyTraj: received AddTrajectory service call");

  // compute a new trajectory
  try {
    MyTraj *newtraj = new MyTraj(req.joint,req.torque);
    // send it to the arm
    res.id = OWD::Plugin::AddTrajectory(newtraj,res.reason);
    if (res.id > 0) {
      res.ok=true;
    } else {
      delete newtraj;
      res.ok=false;
    }
  } catch (const char *err) {
    res.ok=false;
    res.reason=err;
    res.id=0;
  }

  // always return true for the service call so that the client knows that
  // the call was actually processed, as opposed to there being a
  // network communication error.
  // the client will examine the "ok" field to see if the command was
  // actually successful
  return true;
}

// Here is the key function that makes our trajectory different from
// any of the built-in trajectories.  OWD will call this at 500Hz
// while the trajectory is running.  The Trajectory::TrajControl
// argument serves as both input and output.  On input, it contains
// the current position of the arm joints, and zero for all the 
// velocity, acceleration, and extra torque terms.  If we leave it
// untouched, then OWD will not change anything, and will just keep
// holding the current position.  In this case, we are going to
// tell it to add an additional torque to the specified joint for
// exactly one second, after which we will end the trajectory.
// During this execution time the arm will be "free", except for
// the extra torque, because we are not keeping the position at
// a particular value.  If we had wanted to freeze the other joints,
// we could have been setting each of them to their start_position
// values.
void MyTraj::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {

  time += dt;

  if (time < 1) {
    tc.t[joint-1] = torque; // apply torque for exactly 1 second
    ROS_DEBUG("MyTraj Time=%f secs",time);
  } else {
    ROS_DEBUG("MyTraj DONE Time=%f secs",time);
    runstate=OWD::Trajectory::DONE;
  }
  end_position =tc.q;  // keep tracking the current position

  return;
}

// This helper function is called by our register_owd_plugin function
// when the plugin is first loaded.  It just sets up our ROS service
// for receiving trajectory requests.
bool MyTraj::Register() {
  ros::NodeHandle n("~");
  ss_Add = n.advertiseService("AddMyTrajectory",&MyTraj::AddTrajectory);
  return true;
}

// Our unregister_owd_plugin function will call this when OWD shuts down.
// By shutting down our ROS service we will remove its listing from the
// ROS master.
void MyTraj::Shutdown() {
  ss_Add.shutdown();
}

// Have to allocate storage for our ROS service handler, which is a static
// member of MyTraj.
ros::ServiceServer MyTraj::ss_Add;

// The register_owd_plugin() is the only function that OWD will call when
// the plugin is loaded.  It has to initialize our custom Plugin and 
// Trajectory classes.
bool register_owd_plugin() {
  if (myplug) {
    delete myplug; // free the previous one in case register was called twice
  }
  try {
    // create an instantiation of our custom Plugin class
    myplug = new MyPlugin();
  } catch (...) {
    myplug=NULL;
    return false;
  }
  return true;
}

void unregister_owd_plugin() {
  if (myplug) {
    // remove our Plugin class, which will let it shut down any ROS
    // communications it created
    delete myplug;
    myplug=NULL;
  }
  return;
}
