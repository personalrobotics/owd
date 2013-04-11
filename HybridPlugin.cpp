/***********************************************************************

  Copyright 2011-13 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "HybridPlugin.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include <iostream>
#include "openwam/Kinematics.hh"
#include "ApplyForceTraj.h"
#include "ApplyEEForceTorque.h"
#include "DoorTraj.h"
#include "WSTraj.h"
#include "Servo2Traj.h"
#include "FTCheck.h"
#include "InsertKeyTraj.h"
#include "CoggingCompTraj.h"
#define PEAK_CAN
#include "openwamdriver.h"
#include "openwam/CANdefs.hh"	// for HANDSTATE_* enumeration

HybridPlugin::HybridPlugin()
  : write_log_file(false),flush_recorder_data(false),
    current_traj(NULL)

{
  // ROS has already been initialized by OWD, so we can just
  // create our own NodeHandle in the same namespace
  ros::NodeHandle n("~");
  ss_StopTraj = n.advertiseService("StopTraj",&HybridPlugin::StopTraj, this);
  ss_PowerGrasp = n.advertiseService("PowerGrasp",&HybridPlugin::PowerGrasp, this);

  n.param("log_hybridplugin_data",write_log_file,false);

  // Let our Trajectory classes register themselves
  if (!ApplyEEForceTorque::Register()) {
    throw "ApplyEEForceTorque trajectory failed to register";
  }
  if (!ApplyForceTraj::Register()) {
    throw "ApplyForceTraj trajectory failed to register";
  }
  if (!DoorTraj::Register()) {
    throw "DoorTraj trajectory failed to register";
  }
  if (!FTCheck::Register()) {
    throw "FTCheck failed to register";
  }
  if (!InsertKeyTraj::Register()) {
    throw "InsertKeyTraj failed to register";
  }
#ifdef SIMULATION
  if (!JacobianTestTraj::Register()) {
    throw "JacobianTestTraj trajectory failed to register";
  }
#endif // SIMULATION
  if (!WSTraj::Register()) {
    throw "WSTraj trajectory failed to register";
  }
  if (!Servo2Traj::Register()) {
    throw "Servo2Traj trajectory failed to register";
  }
  if (!CoggingCompTraj::Register()) {
    throw "CoggingCompTraj failed to register";
  }

  pub_net_force = n.advertise<std_msgs::Float64MultiArray>("net_force",1);

  recorder = new DataRecorder<double>(6000);
  pthread_mutex_init(&recorder_mutex,NULL);
  pthread_mutex_init(&pub_mutex,NULL);
}

HybridPlugin::~HybridPlugin() {
  // grabbing the lock will make sure that the publisher doesn't try
  // to use ROS at the same time (was causing occassional crashes when
  // reloading the plugin)
  pthread_mutex_lock(&pub_mutex);

  // Tell our Trajectory classes to clean up.
  ApplyForceTraj::Shutdown();
  ApplyEEForceTorque::Shutdown();
  DoorTraj::Shutdown();
  WSTraj::Shutdown();
  FTCheck::Shutdown();
  InsertKeyTraj::Shutdown();
  Servo2Traj::Shutdown();
  CoggingCompTraj::Shutdown();

  if (write_log_file && (recorder->count > 0)) {
    // do a final write of any lingering log data
    write_recorder_data();
  }

  // Shut down our publishers
  pub_net_force.shutdown();

  // Shut down our services
  ss_StopTraj.shutdown();
  ss_PowerGrasp.shutdown();
  
  delete recorder;

  pthread_mutex_unlock(&pub_mutex);
}

void HybridPlugin::log_data(const std::vector<double> &data) {
  if (write_log_file && (pthread_mutex_trylock(&recorder_mutex) == 0)) {
    try {
      recorder->add(data);
    } catch (const char *err) {
      // must have switched to using the recorder for
      // a different record size, so reset and re-add.
      recorder->reset();
      recorder->add(data);
    }
    pthread_mutex_unlock(&recorder_mutex);
  }
}

void HybridPlugin::Publish() {
  // only if we are not shutting down
  if (pthread_mutex_trylock(&pub_mutex) == 0) {
    if (net_force.data.size() > 0) {
      pub_net_force.publish(net_force);
    }
  
    if (write_log_file &&
	((recorder->count > 2500) || flush_recorder_data)) {
      write_recorder_data();
      flush_recorder_data=false;
    }

    pthread_mutex_unlock(&pub_mutex);
  }    
}


// Stop the trajectory when asked by a client
bool HybridPlugin::StopTraj(owd_msgs::Reset::Request &req,
			 owd_msgs::Reset::Response &res) {
  if (current_traj) {
    current_traj->runstate=OWD::Trajectory::DONE;
    res.reason="";
    res.ok=true;
  } else {
    res.reason="No current owd_plugins trajectory.  Either the trajectory you are stopping has already ended, or it was not created by this plugin.";
    res.ok=false;
  }
  return true;
}

bool HybridPlugin::write_recorder_data() {
  char filename[200];
  static int filenum(0);
  ++filenum;
  snprintf(filename,200,"/tmp/hybridplugin-%04d.csv",filenum);
  ROS_INFO("Writing HybridPlugin log to %s",filename);
  pthread_mutex_lock(&recorder_mutex);
  ROS_INFO("dumping log...");
  bool result = recorder->dump(filename);
  ROS_INFO("done.");
  recorder->reset();
  pthread_mutex_unlock(&recorder_mutex);
  return result;
}

bool HybridPlugin::PowerGrasp(owd_msgs::MoveHand::Request &req,
			   owd_msgs::MoveHand::Response &res) {
  int32_t state[4];
  OWD::WamDriver::bus->hand_get_state(state);
  for (unsigned int i=0; i<4; ++i) {
    if (state[i] == HANDSTATE_UNINIT) {
      ROS_WARN("Rejected PowerGrasp: hand is uninitialized");
      res.ok=false;
      res.reason="Hand is uninitialized";
      return true;
    }
  }
  if (req.positions.size() != 4) {
    std::stringstream s;
    s << "Expected 4 joints for PowerGrasp but received " << req.positions.size();
    ROS_ERROR("%s",s.str().c_str());
    res.ok=false;
    res.reason=s.str().c_str();
    return true;
  }
  if (req.movetype != owd_msgs::MoveHand::Request::movetype_position) {
    ROS_ERROR("Only position moves (type=1) are supported for PowerGrasp");
    res.ok=false;
    res.reason="Only position moves (type=1) are supported for PowerGrasp";
    return true;
  }
  
  // turn off the TSTOP value and set higher torque limit
  for (int32_t node=11; node<=13; ++node) {
    if (OWD::WamDriver::bus->hand_set_property(node,TSTOP,0) != OW_SUCCESS) {
      return OW_FAILURE;
    }
    if (OWD::WamDriver::bus->hand_set_property(node,MT,1800) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  }

  // Now set the endpoints and issue the move command
  for (int32_t i=0; i<3; ++i) {
    // make sure we don't try to move beyond the 0 to 2.4 range
    if (req.positions[i] < 0) {
      req.positions[i]=0;
    }
    if (req.positions[i] > 2.4) {
      req.positions[i]=2.4;
    }
    OWD::WamDriver::bus->hand_goal_positions[i+1]
      = OWD::WamDriver::bus->finger_radians_to_encoder(req.positions[i]);
    if (OWD::WamDriver::bus->hand_set_property(11+i,E,
					       OWD::WamDriver::bus->hand_goal_positions[i+1]) 
	!= OW_SUCCESS) {
      return OW_FAILURE;
    }
  }
  // set the spread position
  OWD::WamDriver::bus->hand_goal_positions[4] 
    = OWD::WamDriver::bus->spread_radians_to_encoder(req.positions[3]);
  if (OWD::WamDriver::bus->hand_set_property(14,E,
					     OWD::WamDriver::bus->hand_goal_positions[4])
      != OW_SUCCESS) {
    return OW_FAILURE;
  }

  // record the fact that a motion request is in progress, so we should
  // ignore any state responses that were made between now and when the
  // move command actually makes it to the pucks
  OWD::WamDriver::bus->hand_motion_state_sequence = 1;

  // send the move command
  if (OWD::WamDriver::bus->hand_set_property(GROUPID(5),MODE,MODE_TRAPEZOID) != OW_SUCCESS) {
    return OW_FAILURE;
  }

  for (unsigned int i=0; i<4; ++i) {
    OWD::WamDriver::bus->apply_squeeze[i]=false;
  }
  for (unsigned int i=0; i<4; ++i) {
    OWD::WamDriver::bus->handstate[i] = HANDSTATE_MOVING;
    OWD::WamDriver::bus->encoder_changed[i] = 6;
  }
  OWD::WamDriver::bus->received_state_flags &= ~(0x7800); // clear the four hand bits

  return true;
}

R6 HybridPlugin::workspace_forcetorque() {
  // get the filtered FT force+torque
  R6 force_torque_avg(filtered_ft_force[0],
		      filtered_ft_force[1],
		      filtered_ft_force[2],
		      filtered_ft_torque[0],
		      filtered_ft_torque[1],
		      filtered_ft_torque[2]);
  
  // rotate the force and torque into workspace coordinates
  // we negate each of the sensor readings because what we want is a
  // measure of what the arm is producing, which is the opposite of
  // what the F/T sensor feels.
  R6 ws_force_torque((SO3)endpoint * (-1 * force_torque_avg.v),
                     (SO3)endpoint * (-1 * force_torque_avg.w));


  return ws_force_torque;
}

// Static member inside HybridPlugin class
std_msgs::Float64MultiArray HybridPlugin::net_force;

// Allocation for the pointer that will hold the single instantiation
// of our plugin class.  We initialize it to NULL so that the register
// function can tell whether or not one has already been allocated.
HybridPlugin *hybridplug = NULL;

// The register_owd_plugin() is the only function that OWD will call when
// the plugin is loaded.  It has to initialize our custom Plugin and 
// Trajectory classes.
bool register_owd_plugin() {
  if (hybridplug) {
    delete hybridplug; // free the previous one in case register was called twice
  }
  try {
    // create an instantiation of our custom Plugin class
    hybridplug = new HybridPlugin();
  } catch (...) {
    hybridplug=NULL;
    return false;
  }
  return true;
}

void unregister_owd_plugin() {
  if (hybridplug) {
    // remove our Plugin class, which will let it shut down any ROS
    // communications it created
    delete hybridplug;
    hybridplug=NULL;
  }
  return;
}
