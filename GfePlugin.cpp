/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "GfePlugin.hh"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include "openwam/Kinematics.hh"
#include <iostream>
#include "ApplyForceTraj.h"
#include "DoorTraj.h"
#include "JacobianTest.h"

GfePlugin::GfePlugin() {
  // ROS has already been initialized by OWD, so we can just
  // create our own NodeHandle in the same namespace
  ros::NodeHandle n("~");

  // Let our Trajectory classes register themselves
  if (!ApplyForceTraj::Register()) {
    throw "ApplyForceTraj trajectory failed to register";
  }
  if (!DoorTraj::Register()) {
    throw "DoorTraj trajectory failed to register";
  }
#ifdef SIMULATION
  if (!JacobianTestTraj::Register()) {
    throw "JacobianTestTraj trajectory failed to register";
  }
#endif // SIMULATION

  pub_net_force = n.advertise<std_msgs::Float64MultiArray>("net_force",1);

}

GfePlugin::~GfePlugin() {
  // Also tell our Trajectory classes to clean up.
  ApplyForceTraj::Shutdown();
  DoorTraj::Shutdown();
  JacobianTestTraj::Shutdown();
}

void GfePlugin::Publish() {
  pub_net_force.publish(net_force);
}

// Static member inside GfePlugin class
std_msgs::Float64MultiArray GfePlugin::net_force;

// Allocation for the pointer that will hold the single instantiation
// of our plugin class.  We initialize it to NULL so that the register
// function can tell whether or not one has already been allocated.
GfePlugin *gfeplug = NULL;


// Have to allocate storage for our ROS service handlers
ros::ServiceServer ApplyForceTraj::ss_ApplyForce,
  ApplyForceTraj::ss_SetForcePGain, 
  ApplyForceTraj::ss_SetForceDGain,
  DoorTraj::ss_OpenDoor,
  JacobianTestTraj::ss_JacobianTest;

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
