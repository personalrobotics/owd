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
  PullDirection.normalize();
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

  // direction that the trajectory is moving the endpoint
  R6 endpoint_movement = gfeplug->Jacobian_times_vector(joint_error);

  // get the Pull direction in world coordinates
  R3 world_PullDirection = (SO3)gfeplug->endpoint * PullDirection;

  // map the trajectory translation to just the direction we care about
  double dot_prod = endpoint_movement.v * world_PullDirection;
  R3 endpoint_trans = dot_prod * world_PullDirection;

  // don't correct the endpoint rotation at all
  R6 endpoint_correction(endpoint_trans,R3());

  OWD::JointPos joint_correction;
  try {
    joint_correction = 
      gfeplug->JacobianPseudoInverse_times_vector(endpoint_correction);
  } catch (const char *err) {
    // no valid Jacobian, for whatever reason, so we won't get
    // any endpoint correction this timestep
  }
  
  // Finally, project our joint error into the nullspace so that
  // we can correct as much as we can without changing the endpoint
  try {
    OWD::JointPos configuration_correction
      = gfeplug->Nullspace_projection(joint_error);
    joint_correction += configuration_correction;
  } catch (const char *err) {
    // don't worry about it
  }
  tc.q += joint_correction;
  end_position = tc.q;
}

bool DoorTraj::OpenDoor(gfe_owd_plugin::OpenDoor::Request &req,
			gfe_owd_plugin::OpenDoor::Response &res) {

  if (req.traj.positions.size() < 2) {
    ROS_ERROR_NAMED("OpenDoor","Minimum of 2 traj points required for door-opening trajectory");
    res.id = 0;
    res.ok=false;
    res.reason="Minimum of 2 traj points required for door-opening trajectory";
    return true;
  }

  OWD::TrajType traj;
  try {
    traj=OWD::Plugin::ros2owd_traj(req.traj);
  } catch (const char *error) {
    char last_trajectory_error[200];
    snprintf(last_trajectory_error,200,"Could not extract valid trajectory: %s",error);
    ROS_ERROR_NAMED("OpenDoor","%s",last_trajectory_error);
    res.id=0;
    res.ok=false;
    res.reason=last_trajectory_error;
    return true;
  }

  // compute a new trajectory
  try {
    DoorTraj *newtraj = new DoorTraj(traj, R3(req.pull_direction.x,
					      req.pull_direction.y,
					      req.pull_direction.z));

    // send it to the arm
    res.id = OWD::Plugin::AddTrajectory(newtraj,res.reason);
    if (res.id > 0) {
      res.ok=true;
      res.reason="";
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
  // the call was processed, as opposed to there being a
  // network communication error.
  // the client will examine the "ok" field to see if the command was
  // actually successful
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
