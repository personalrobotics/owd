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
#include "WSTraj.h"
#include "Follow.h"
#include "HelixPlugin.h"
#include "MoveDirection.h"

GfePlugin::GfePlugin()
  : write_log_file(false),flush_recorder_data(false)
{
  // ROS has already been initialized by OWD, so we can just
  // create our own NodeHandle in the same namespace
  ros::NodeHandle n("~");

  n.param("log_gfeplugin_data",write_log_file,false);

  // Let our Trajectory classes register themselves
  if (!ApplyForceTraj::Register()) {
    throw "ApplyForceTraj trajectory failed to register";
  }
  if (!DoorTraj::Register()) {
    throw "DoorTraj trajectory failed to register";
  }
 if (!Follow::Register()) {
    throw "Follow trajectory failed to register";
  }
 if (!HelixTraj::Register()) {
   throw "HelixTraj trajectory failed to registr";
 }
 if (!MoveDirection::Register()) {
   throw "MoveDirection trajectory failed to register";
 }
#ifdef SIMULATION
  if (!JacobianTestTraj::Register()) {
    throw "JacobianTestTraj trajectory failed to register";
  }
#endif // SIMULATION
  if (!WSTraj::Register()) {
    throw "WSTraj trajectory failed to register";
  }

  pub_net_force = n.advertise<std_msgs::Float64MultiArray>("net_force",1);

  recorder = new DataRecorder<double>(6000);
  pthread_mutex_init(&recorder_mutex,NULL);
  pthread_mutex_init(&pub_mutex,NULL);
}

GfePlugin::~GfePlugin() {
  // grabbing the lock will make sure that the publisher doesn't try
  // to use ROS at the same time (was causing occassional crashes when
  // reloading the plugin)
  pthread_mutex_lock(&pub_mutex);

  // Tell our Trajectory classes to clean up.
  ApplyForceTraj::Shutdown();
  DoorTraj::Shutdown();
  JacobianTestTraj::Shutdown();
  WSTraj::Shutdown();
  Follow::Shutdown();
  HelixTraj::Shutdown();
  MoveDirection::Shutdown();

  // Shut down our publisher
  pub_net_force.shutdown();
  
  pthread_mutex_unlock(&pub_mutex);
}

void GfePlugin::log_data(const std::vector<double> &data) {
  if (write_log_file && (pthread_mutex_trylock(&recorder_mutex) == 0)) {
    recorder->add(data);
    pthread_mutex_unlock(&recorder_mutex);
  }
}

void GfePlugin::Publish() {
  // only if we are not shutting down
  if (pthread_mutex_trylock(&pub_mutex) == 0) {
    pub_net_force.publish(net_force);
    pthread_mutex_unlock(&pub_mutex);
  }    
  
  if (write_log_file &&
      ((recorder->count > 2500) || flush_recorder_data)) {
    write_recorder_data();
    flush_recorder_data=false;
  }
}


bool GfePlugin::write_recorder_data() {
  char filename[200];
  static int filenum(0);
  ++filenum;
  snprintf(filename,200,"/tmp/gfeplugin-%04d.csv",filenum);
  ROS_INFO("Writing GfePlugin log to %s",filename);
  pthread_mutex_lock(&recorder_mutex);
  ROS_INFO("dumping log...");
  bool result = recorder->dump(filename);
  ROS_INFO("done.");
  recorder->reset();
  pthread_mutex_unlock(&recorder_mutex);
  return result;
}

// Static member inside GfePlugin class
std_msgs::Float64MultiArray GfePlugin::net_force;

// Allocation for the pointer that will hold the single instantiation
// of our plugin class.  We initialize it to NULL so that the register
// function can tell whether or not one has already been allocated.
GfePlugin *gfeplug = NULL;

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
