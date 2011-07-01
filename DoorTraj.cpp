/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "DoorTraj.h"

DoorTraj::DoorTraj(OWD::TrajType &vtraj, R3 _PullDirection)
  : MacJointTraj(vtraj,
		 max_j_vel, 
		 max_j_accel,
		 maxjerk,
		 false, false, false),
    PullDirection(_PullDirection)
{
}

DoorTraj::~DoorTraj() {
}

void DoorTraj::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
  // First, call evaluate on the base class to see where we would want
  // to be if we weren't also dealing with the door
  TrajControl tc2(tc);
  OWD::MacJointTraj::evaluate(tc2,dt);

  // now, compare the desired position against the actual position
  // and relax any of the EE position contraints that are not in
  // the pull direction
  OWD::JointPos joint_error = tc2.q - tc.q;
}

bool DoorTraj::OpenDoor(gfe_owd_plugin::OpenDoor::Request &req,
			gfe_owd_plugin::OpenDoor::Response &res) {
  return true;
}

bool DoorTraj::Register() {
  max_j_vel.resize(7);
  max_j_accel.resize(7);
  for (int i=0; i<4; ++i) {
    max_j_vel[i]=max_j_accel[i]=1.0;
  }
  for (int i=4; i<7; ++i) {
    max_j_vel[i]=max_j_accel[i]=2.0;
  }

  ros::NodeHandle n("~");
  ss_OpenDoor = n.advertiseService("OpenDoor",&DoorTraj::OpenDoor);
  
  return true;
}

void DoorTraj::Shutdown() {
}

std::vector<double> DoorTraj::max_j_vel;
std::vector<double> DoorTraj::max_j_accel;
ros::ServiceServer  DoorTraj::ss_OpenDoor;
