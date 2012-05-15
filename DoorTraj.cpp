/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "DoorTraj.h"
#include <openwam/Kinematics.hh>
#include <LinearMath/btQuaternion.h>

/* Still need:
   operator for SE3 for subtraction between two SE3s
   operator for SE3 for multiplying a difference by a double
   conversion from geometry_msgs::Pose to SE3
*/

DoorTraj::DoorTraj(OWD::TrajType &vtraj, owd_plugins::OpenDoor::Request::_ee_pose_type &eep, R3 _PullDirection)
  : MacJointTraj(vtraj,
		 max_j_vel, 
		 max_j_accel,
		 maxjerk,
		 false, false, false,false),
    PullDirection(_PullDirection),
    endpoint_position_goal((R3)hybridplug->endpoint),
    last_traj_endpoint((R3)hybridplug->endpoint)
{
  type="DoorTraj";
  PullDirection.normalize();
  if (eep.size() != vtraj.size()) {
    ROS_ERROR_NAMED("OpenDoor","Number of ee_pose points doesn't match number of trajectory points");
    throw "Number of ee_pose points doesn't match number of trajectory points";
  }
  last_pose = pose_to_SE3(eep[0]);
  ROS_INFO_STREAM_NAMED("OpenDoor","First endpoint pose: " << last_pose);
  ROS_INFO_STREAM_NAMED("OpenDoor","Current endpoint pose: " << hybridplug->endpoint);
  std::vector<MacQuinticElement *>::iterator p_traj_element = macpieces.begin();
  for (unsigned int i=1; i<eep.size(); ++i) {
    if ((*p_traj_element)->end_pos == vtraj[i]) {
      // there was no blend at the end of this segment, so we can just
      // use our pose as-is.
      pose_segments.push_back(PoseSegment(last_pose, pose_to_SE3(eep[i]), (*p_traj_element)->start_time, (*p_traj_element)->duration));
      last_pose = pose_to_SE3(eep[i]);
    } else {
      // there's a blend that bypasses the original endpoint, so
      // first back off the endpoint
      OWD::JointPos seg1 = (*p_traj_element)->end_pos - (*p_traj_element)->start_pos;
      OWD::JointPos seg2 = vtraj[i] - (*p_traj_element)->start_pos;
      double straight_fraction = seg1.length() / seg2.length();
      if (straight_fraction > 1) {
	ROS_ERROR_NAMED("OpenDoor","Confused: straight_fraction > 1");
	throw "OpenDoor: straight_fraction > 1";
      }
      SE3 ep = last_pose + straight_fraction * (pose_to_SE3(eep[i]) - last_pose);
      pose_segments.push_back(PoseSegment(last_pose,ep,(*p_traj_element)->start_time, (*p_traj_element)->duration));

      // Now add the blend element
      last_pose = ep;
      if (++p_traj_element == macpieces.end()) {
	ROS_ERROR_NAMED("OpenDoor","Ran out of traj elements before we reached the blend");
	throw "Ran out of traj elements before we reached the blend";
      }
      if (i == vtraj.size() -1) {
	ROS_ERROR_NAMED("OpenDoor","Ran out of traj points before completing a straight segment");
	throw "Ran out of traj points before completing a straight segment";
      }
      seg1 = (*p_traj_element)->end_pos - vtraj[i];
      seg2 = vtraj[i+1] - vtraj[i];
      double blend_fraction = seg1.length() / seg2.length();
      if (blend_fraction > 1) {
	ROS_ERROR_NAMED("OpenDoor","Confused: blend_fraction > 1");
	throw "OpenDoor: blend_fraction > 1";
      }
      ep = pose_to_SE3(eep[i]) + blend_fraction * (pose_to_SE3(eep[i+1]) - pose_to_SE3(eep[i]));
      pose_segments.push_back(PoseSegment(last_pose,ep,(*p_traj_element)->start_time, (*p_traj_element)->duration));
      last_pose = ep;
    }
    // increment our traj element pointer if we still have more to go
    if (i < eep.size()-1) {
      if (++p_traj_element == macpieces.end()) {
	ROS_ERROR_NAMED("OpenDoor","Ran out of traj elements before we reached the final point");
	throw "Ran out of traj elements before we reached the final point";
      }
    }
  }
  current_pose_segment = pose_segments.begin();
      
  recorder = new DataRecorder<double>(500000);
  hybridplug->current_traj=this;
}



DoorTraj::~DoorTraj() {
  last_traj_id = id;
  if (pthread_create(&recorder_thread,NULL,&write_recorder_data,(void*)recorder)) {
    ROS_WARN_NAMED("OpenDoor","Could not create thread to write log data");
  }
  hybridplug->current_traj=NULL;
}

void DoorTraj::evaluate_abs(OWD::Trajectory::TrajControl &tc, double t) {
  if (OWD::Kinematics::max_condition > 25) {
    ROS_ERROR_NAMED("OpenDoor","Jacobian Psuedo-Inverse condition number reached %2f; aborting", OWD::Kinematics::max_condition);
    runstate=ABORT;
    return;
  }

  // a place to record log values for debugging
  std::vector<double> data;

  // First, call evaluate on the base class to see where we would want
  // to be if we weren't also dealing with the door
  TrajControl tc2(tc);
  OWD::MacJointTraj::evaluate_abs(tc2,t);
  SE3 traj_endpoint_pose = interpolate_ee_pose(tc2.q);

  data.push_back(time); // time, column 1
  data.insert(data.end(),tc.q.begin(), tc.q.end()); // current positions, col 2-8
  data.insert(data.end(),tc2.q.begin(), tc2.q.end()); // traj positions, col 9-15
  std::vector<double> pose;
  pose.resize(16);
  memcpy(&pose[0],(double *)traj_endpoint_pose,16*sizeof(double));
  data.insert(data.end(),pose.begin(), pose.end()); // traj pose, col 16-31

  // calculate how much the endpoint position changed
  R3 traj_endpoint_movement = (R3)traj_endpoint_pose - last_traj_endpoint;
  last_traj_endpoint = (R3) traj_endpoint_pose;

  // update the endpoint position goal
  endpoint_position_goal += traj_endpoint_movement;
  //  endpoint_position_goal = (R3) traj_endpoint_pose;

  ROS_DEBUG_STREAM_NAMED("OpenDoor","Current traj endpoint:   " << (R3)traj_endpoint_pose);
  ROS_DEBUG_STREAM_NAMED("OpenDoor","Current actual endpoint: " << (R3)hybridplug->endpoint);
  ROS_DEBUG_STREAM_NAMED("OpenDoor","New endpoint goal:       " << endpoint_position_goal);
  
  // calculate the endpoint position error
  R3 endpoint_error = endpoint_position_goal - (R3)hybridplug->endpoint;
  memcpy(&pose[0],(const double *)hybridplug->endpoint,16*sizeof(double));
  data.insert(data.end(),pose.begin(), pose.end()); // actual pose, col 32-47
  data.push_back(endpoint_error[0]); // endpoint error, col 48-50
  data.push_back(endpoint_error[1]);
  data.push_back(endpoint_error[2]);

  // project the position error onto the pull direction
  R3 world_PullDirection = (SO3)hybridplug->endpoint * PullDirection;
  R3 endpoint_longitudinal_error = (endpoint_error * world_PullDirection) * world_PullDirection;
  R3 endpoint_lateral_error = endpoint_error - endpoint_longitudinal_error;
  if (OWD::Plugin::simulation) {
    ROS_DEBUG_NAMED("OpenDoor","Net ee movement of %2.3f",endpoint_longitudinal_error.norm());
  }
  data.push_back(endpoint_lateral_error[0]);  // lateral error cols 51-53
  data.push_back(endpoint_lateral_error[1]);
  data.push_back(endpoint_lateral_error[2]);
  data.push_back(endpoint_longitudinal_error[0]);  // longitudinal error cols 54-56
  data.push_back(endpoint_longitudinal_error[1]);
  data.push_back(endpoint_longitudinal_error[2]);

  // subtract the lateral error from the goal
  endpoint_position_goal -= endpoint_lateral_error;

  // calculate the endpoint rotation error
  so3 endpoint_rotation_error_so3 =(so3)((SO3)traj_endpoint_pose * (! (SO3)hybridplug->endpoint));
  // rotation error in world frame
  R3 endpoint_rotation_error = endpoint_rotation_error_so3.t() * endpoint_rotation_error_so3.w();
  
  data.push_back(endpoint_rotation_error[0]); // rotational error cols 57-59
  data.push_back(endpoint_rotation_error[1]);
  data.push_back(endpoint_rotation_error[2]);

  // combine the longitudinal translation error with the rotation error
  R6 endpoint_correction(endpoint_longitudinal_error,endpoint_rotation_error);

  OWD::JointPos joint_correction;
  try {
    if (OWD::Plugin::simulation) {
      ROS_DEBUG_STREAM_NAMED("OpenDoor","Endpoint pos/rot correction of" << std::endl << endpoint_correction);
    }
    joint_correction = 
      hybridplug->JacobianPseudoInverse_times_vector(endpoint_correction);
    //    data.insert(data.end(),joint_correction.begin(),
    //		joint_correction.end());  // joint change to correct ep, col 60-66
    if (OWD::Plugin::simulation) {
      ROS_DEBUG_NAMED("OpenDoor","JacobianPsueoInverse condition number %3.3f",OWD::Kinematics::max_condition);
      ROS_DEBUG_NAMED("OpenDoor","JacobianPsuedoInverse Matrix:");
      for (int i=0; i<7; ++i) {
	ROS_DEBUG_NAMED("OpenDoor","  [%3.3f %3.3f %3.3f %3.3f %3.3f %3.3f]",
			OWD::Kinematics::Jacobian0PseudoInverse[0][i], OWD::Kinematics::Jacobian0PseudoInverse[1][i], 
			OWD::Kinematics::Jacobian0PseudoInverse[2][i], OWD::Kinematics::Jacobian0PseudoInverse[3][i], 
			OWD::Kinematics::Jacobian0PseudoInverse[4][i], OWD::Kinematics::Jacobian0PseudoInverse[5][i]);
      }
      ROS_DEBUG_NAMED("OpenDoor","Joint correction [%1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f]",
		      joint_correction[0], joint_correction[1], joint_correction[2], joint_correction[3], joint_correction[4], joint_correction[5], joint_correction[6]);
      ROS_DEBUG_NAMED("OpenDoor","  (length of of %2.3f)",
		      joint_correction.length());

    }
    
  } catch (const char *err) {
    // no valid Jacobian, for whatever reason, so we won't get
    // any endpoint correction this timestep
    data.insert(data.end(),tc.q.size(),-1); // invalid joint change (col 60-66)
    if (OWD::Plugin::simulation) {
      ROS_WARN_NAMED("OpenDoor","No JacobianPseudoInverse available");
    }
  }
  
  // Finally, project our joint error into the nullspace so that
  // we can correct as much as we can without changing the endpoint
  OWD::JointPos joint_error(tc2.q - tc.q);
  try {
    OWD::JointPos configuration_correction
      = hybridplug->Nullspace_projection(joint_error);
    data.insert(data.end(),configuration_correction.begin(),
		configuration_correction.end()); // joint change to correct config, col 67-73 (was 58-64)
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
    data.insert(data.end(),tc.q.size(),-1);  // invalid joint change (col 67-73)
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
  data.insert(data.end(),tc.q.begin(), tc.q.end()); // target positions, col 74-80 (was 65-71)

  recorder->add(data);

  if (runstate == DONE) {
    end_position = tc.q;
  }
}

SE3 DoorTraj::interpolate_ee_pose(const OWD::JointPos &current_pos) {
  // use the trajectory position to interpolate between adjacent end-effector poses
  while ((current_pose_segment->start_time + current_pose_segment->duration) < time) {
    if (++current_pose_segment == pose_segments.end()) {
      --current_pose_segment; // back up to the last segment
      if (time > current_pose_segment->start_time + current_pose_segment->duration + 0.25) {
	ROS_ERROR_NAMED("OpenDoor","Could not find a pose segment for time %2.3f",time);
	throw "Could not find a pose segment for current time";
      } else {
	time = current_pose_segment->start_time + current_pose_segment->duration;
	break;
      }
    }
  }
  double fraction;
  MacQuinticSegment *mqs = dynamic_cast<MacQuinticSegment *>(*current_piece);
  if (mqs) {
    // if we're in a straight segment, interpolate based on distance
    OWD::JointPos progress = current_pos - mqs->start_pos;
    fraction = progress.length() / mqs->distance;
  } else {
    // if we're in a blend, interpolate based on time (since velocity through the blend is
    //   relatively constant, this is close enough to matching the corresponding position)
    fraction = (time - (*current_piece)->start_time) / (*current_piece)->duration;
  }
  if (fraction < 0) {
    ROS_ERROR_NAMED("OpenDoor","Could not find our relative pose position for time %2.3f: fraction=%2.2f",time, fraction);
    throw "Could not find our relative pose position: fraction<0";
  } else if (fraction > 1) {
    ROS_ERROR_NAMED("OpenDoor","Could not find our relative pose position for time %2.3f: fraction=%2.2f",time, fraction);
    throw "Could not find our relative pose position: fraction>1";
  }
  //  ROS_DEBUG_STREAM_NAMED("OpenDoor","Current pose segment starts with" << std::endl << current_pose_segment->starting_pose);
  //  ROS_DEBUG_STREAM_NAMED("OpenDoor","and shifts by " << fraction << " of" << std::endl << current_pose_segment->pose_shift);
  SE3 current_pose = current_pose_segment->starting_pose 
    + fraction * current_pose_segment->pose_shift;

  return current_pose;
}

bool DoorTraj::OpenDoor(owd_plugins::OpenDoor::Request &req,
			owd_plugins::OpenDoor::Response &res) {

  if (req.traj.positions.size() < 2) {
    ROS_ERROR_NAMED("OpenDoor","Minimum of 2 traj points required for door-opening trajectory");
    res.id = std::string("");
    res.ok=false;
    res.reason="Minimum of 2 traj points required for door-opening trajectory";
    return true;
  }
  if (req.ee_pose.size() != req.traj.positions.size()) {
    ROS_ERROR_NAMED("OpenDoor","Length of ee_pose array (%zd) does not match number of trajectory positions (%zd)",req.ee_pose.size(),req.traj.positions.size());
    res.id = std::string("");
    res.ok=false;
    res.reason="Length of ee_pose array does not match number of trajectory positions";
    return true;
  }

  OWD::TrajType traj;
  try {
    traj=OWD::Plugin::ros2owd_traj(req.traj);
  } catch (const char *error) {
    char last_trajectory_error[200];
    snprintf(last_trajectory_error,200,"Could not extract valid trajectory: %s",error);
    ROS_ERROR_NAMED("OpenDoor","%s",last_trajectory_error);
    res.id=std::string("");
    res.ok=false;
    res.reason=last_trajectory_error;
    return true;
  }

  // compute a new trajectory
  try {
    DoorTraj *newtraj = new DoorTraj(traj, req.ee_pose, R3(req.pull_direction.x,
							   req.pull_direction.y,
							   req.pull_direction.z));

    // send it to the arm
    if (OWD::Plugin::AddTrajectory(newtraj,res.reason)) {
      res.ok=true;
      res.id = newtraj->id;
      res.reason="";
    } else {
      delete newtraj;
      res.id = std::string("");
      res.ok=false;
    }
  } catch (const char *err) {
    res.ok=false;
    res.reason=err;
    res.id=std::string("");
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
  snprintf(filename,200,"/tmp/doortraj-%s.csv",last_traj_id);
  ROS_INFO("Writing doortraj log to %s",filename);
  ((DataRecorder<double> *)recorder)->dump(filename);
  delete (DataRecorder<double> *)recorder;
  return NULL;
}

SE3 pose_to_SE3(geometry_msgs::Pose &p) {
  btQuaternion ep_qrot(p.orientation.x,
		       p.orientation.y,
		       p.orientation.z,
		       p.orientation.w);
  so3 R(R3(ep_qrot.getAxis().x(),
	   ep_qrot.getAxis().y(),
	   ep_qrot.getAxis().z()),
	ep_qrot.getAngle());
  R3 t(p.position.x,
       p.position.y,
       p.position.z);
  return SE3(R,t);
}


std::vector<double> DoorTraj::max_j_vel;
std::vector<double> DoorTraj::max_j_accel;
ros::ServiceServer  DoorTraj::ss_OpenDoor;
std::string DoorTraj::last_traj_id;
pthread_t DoorTraj::recorder_thread;
