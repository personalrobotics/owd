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
    PullDirection(_PullDirection),
    last_traj_pos(vtraj[0]),
    endpoint_goal((R3)gfeplug->endpoint)
{
  type="DoorTraj";
  PullDirection.normalize();
  recorder = new DataRecorder<double>(500000);
}

DoorTraj::~DoorTraj() {
  last_traj_id = id;
  if (pthread_create(&recorder_thread,NULL,&write_recorder_data,(void*)recorder)) {
    ROS_WARN_NAMED("OpenDoor","Could not create thread to write log data");
  }
}

void DoorTraj::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
  // a place to record log values for debugging
  std::vector<double> data;

  // First, call evaluate on the base class to see where we would want
  // to be if we weren't also dealing with the door
  TrajControl tc2(tc);
  OWD::MacJointTraj::evaluate(tc2,dt);

  data.push_back(time); // time, column 1
  data.insert(data.end(),tc.q.begin(), tc.q.end()); // current positions, col 2-8
  data.insert(data.end(),tc2.q.begin(), tc2.q.end()); // traj positions, col 9-15

  // now, compare the desired position against the actual position
  // and relax any of the EE position contraints that are not in
  // the pull direction
  OWD::JointPos joint_movement = tc2.q - last_traj_pos;
  last_traj_pos = tc2.q;
  if (OWD::Plugin::simulation) {
    ROS_DEBUG_NAMED("OpenDoor","Trajectory joint movement of %2.3f",joint_movement.length());
  }

  // direction that the trajectory is moving the endpoint
  R6 endpoint_movement = gfeplug->Jacobian_times_vector(joint_movement);
  data.push_back(endpoint_movement[0]);  // endpoint translation, col 16-18
  data.push_back(endpoint_movement[1]);
  data.push_back(endpoint_movement[2]);
  data.push_back(endpoint_movement[3]);  // endpoint rotation, col 19-21
  data.push_back(endpoint_movement[4]);
  data.push_back(endpoint_movement[5]);
  if (OWD::Plugin::simulation) {
    OWD::JointPos ep;
    ep.push_back(endpoint_movement[0]);
    ep.push_back(endpoint_movement[1]);
    ep.push_back(endpoint_movement[2]);
    ROS_DEBUG_NAMED("OpenDoor","Trajectory ee movement of %2.3f",ep.length());
  }

  // get the Pull direction in world coordinates
  R3 world_PullDirection = (SO3)gfeplug->endpoint * PullDirection;

  // map the trajectory translation to just the direction we care about
  double dot_prod = endpoint_movement.v() * world_PullDirection;
  R3 endpoint_trans = dot_prod * world_PullDirection;

  data.push_back(endpoint_trans[0]); // net endpoint translation, col 22-24
  data.push_back(endpoint_trans[1]);
  data.push_back(endpoint_trans[2]);
  if (OWD::Plugin::simulation) {
    ROS_DEBUG_NAMED("OpenDoor","Net ee movement of %2.3f",endpoint_trans.norm());
  }
  endpoint_goal += endpoint_trans;

  // get the rotation error
  OWD::JointPos joint_error = tc2.q - tc.q;
  R3 endpoint_rot_error = (gfeplug->Jacobian_times_vector(joint_error)).w();
  
  // calculate the endpoint translation error
  R3 endpoint_total_trans_error = endpoint_goal - (R3)gfeplug->endpoint;
  // project the trans error onto the pull direction
  R3 endpoint_longitudinal_error = (endpoint_total_trans_error * world_PullDirection)
    * world_PullDirection;
  R3 endpoint_lateral_error = endpoint_total_trans_error - endpoint_longitudinal_error;
  if (OWD::Plugin::simulation) {
    endpoint_longitudinal_error += (SO3)gfeplug->endpoint * R3(0,0.0001,0);
  }
  
  endpoint_goal -= endpoint_lateral_error;

  // combine the modified translation with the full rotation error
  R6 endpoint_correction(endpoint_longitudinal_error,endpoint_rot_error);

  OWD::JointPos joint_correction;
  try {
    joint_correction = 
      gfeplug->JacobianPseudoInverse_times_vector(endpoint_correction);
    data.insert(data.end(),joint_correction.begin(),
		joint_correction.end());  // joint change to correct ep, col 25-31
    if (OWD::Plugin::simulation) {
      ROS_DEBUG_NAMED("OpenDoor","Endpoint joint correction of %2.3f",
		      joint_correction.length());
    }
    
  } catch (const char *err) {
    // no valid Jacobian, for whatever reason, so we won't get
    // any endpoint correction this timestep
    data.insert(data.end(),tc.q.size(),-1);
    if (OWD::Plugin::simulation) {
      ROS_WARN_NAMED("OpenDoor","No JacobianPseudoInverse available");
    }
  }
  
  // Finally, project our joint error into the nullspace so that
  // we can correct as much as we can without changing the endpoint
  try {
    OWD::JointPos configuration_correction
      = gfeplug->Nullspace_projection(joint_error);
    data.insert(data.end(),configuration_correction.begin(),
		configuration_correction.end()); // joint change to correct config, col 32-38
    if (OWD::Plugin::simulation) {
      ROS_DEBUG_NAMED("OpenDoor","Configuration correction of %2.3f",
		      configuration_correction.length());
    }
    if (joint_correction.size() > 0) {
      joint_correction += configuration_correction;
    } else {
      joint_correction = configuration_correction;
    }
  } catch (const char *err) {
    // don't worry about it
    data.insert(data.end(),tc.q.size(),-1);
    if (OWD::Plugin::simulation) {
      ROS_WARN_NAMED("OpenDoor","Nullspace projection failed");
    }
  }
  if (OWD::Plugin::simulation) {
    ROS_DEBUG_NAMED("OpenDoor","Total joint correction of %2.3f",
		    joint_correction.length());
  }

  if (joint_correction.size() == tc.q.size()) {
    tc.q += joint_correction;
  }
  data.insert(data.end(),tc.q.begin(), tc.q.end()); // target positions, col 39-45

  recorder->add(data);

  if (runstate == DONE) {
    end_position = tc.q;
  }
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

void *DoorTraj::write_recorder_data(void *recorder) {
  char filename[200];
  snprintf(filename,200,"/tmp/doortraj-%02d.csv",last_traj_id);
  ROS_INFO("Writing doortraj log to %s",filename);
  ((DataRecorder<double> *)recorder)->dump(filename);
  delete (DataRecorder<double> *)recorder;
  return NULL;
}


std::vector<double> DoorTraj::max_j_vel;
std::vector<double> DoorTraj::max_j_accel;
ros::ServiceServer  DoorTraj::ss_OpenDoor;
int DoorTraj::last_traj_id = 0;
pthread_t DoorTraj::recorder_thread;
