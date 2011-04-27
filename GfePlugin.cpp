/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "GfePlugin.hh"
#include <ros/ros.h>
#include <std_msgs/String.h>

GfePlugin::GfePlugin() {
  // ROS has already been initialized by OWD, so we can just
  // create our own NodeHandle in the same namespace
  ros::NodeHandle n("~");

  // Let our Trajectory class register itself
  if (!ApplyForceTraj::Register()) {
    throw "ApplyForceTraj trajectory failed to register";
  }
}

GfePlugin::~GfePlugin() {
  // Also tell our Trajectory class to clean up.
  ApplyForceTraj::Shutdown();
}

// Allocation for the pointer that will hold the single instantiation
// of our plugin class.  We initialize it to NULL so that the register
// function can tell whether or not one has already been allocated.
GfePlugin *gfeplug = NULL;

// Process our ApplyForce service calls from ROS
bool ApplyForceTraj::ApplyForce(gfe_owd_plugin::ApplyForce::Request &req,
				gfe_owd_plugin::ApplyForce::Response &res) {
  ROS_INFO("GfePlugin: received ApplyForce service call");

  // compute a new trajectory
  try {
    ApplyForceTraj *newtraj = new ApplyForceTraj(req.x,req.y,req.z,req.f);
    // send it to the arm
    res.id = OWD::Plugin::AddTrajectory(newtraj,res.reason);
    if (res.id > 0) {
      res.ok=true;
      res.reason="";
    } else {
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

ApplyForceTraj::ApplyForceTraj(double _x, double _y, double _z, double _f): OWD::Trajectory("GFE Apply Force"), x(_x), y(_y), z(_z), f(_f), stopforce(false) {
  if (gfeplug) {

    // Set the start position from the current position
    start_position=gfeplug->target_arm_position;
    end_position = start_position;

    // Compute and save the current Cartesian endpoint position

  } else {
    throw "Could not get current WAM values from GfePlugin";
  }

  // create the service that the client can use to stop the force
  ros::NodeHandle n("~");
  ss_StopForce = n.advertiseService("StopForce",&ApplyForceTraj::StopForce, this);

}

void ApplyForceTraj::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {

  time += dt;

  if (stopforce) {
    runstate=OWD::Trajectory::DONE;
    return;
  }

  // calculate the endpoint position

  // calculate the endpoint correction in every direction except the
  // force direction

  // calculate the additional torques for the endpoint force in the
  // specified direction

  end_position =tc.q;  // keep tracking the current position
  return;
}

// Stop the trajectory when asked by a client
bool ApplyForceTraj::StopForce(gfe_owd_plugin::StopForce::Request &req,
			       gfe_owd_plugin::StopForce::Response &res) {
  stopforce=true;
  return true;
}

// Set up our ROS service for receiving trajectory requests.
bool ApplyForceTraj::Register() {
  ros::NodeHandle n("~");
  ss_ApplyForce = n.advertiseService("ApplyForce",&ApplyForceTraj::ApplyForce);
  return true;
}

// Shut down our service so that it's no longer listed by the ROS master
void ApplyForceTraj::Shutdown() {
  ss_ApplyForce.shutdown();
}

ApplyForceTraj::~ApplyForceTraj() {
  ss_StopForce.shutdown();
}

// Have to allocate storage for our ROS service handler
ros::ServiceServer ApplyForceTraj::ss_ApplyForce;

// The register_owd_plugin() is the only function that OWD will call when
// the plugin is loaded.  It has to initialize our custom Plugin and 
// Trajectory classes.
bool register_owd_plugin() {
  if (gfeplug) {
    delete gfeplug; // free the previous one in case register was called twice
  }
  try {
    // create an instantiation of our custom Plugin class
    gfeplug = new GfePlugin();
  } catch (...) {
    gfeplug=NULL;
    return false;
  }
  return true;
}

void unregister_owd_plugin() {
  if (gfeplug) {
    // remove our Plugin class, which will let it shut down any ROS
    // communications it created
    delete gfeplug;
    gfeplug=NULL;
  }
  return;
}
