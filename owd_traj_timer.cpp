/***********************************************************************

  Copyright 2012 Carnegie Mellon University
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

  This file is part of owd.

  owd is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  owd is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***********************************************************************/

#include <ros/ros.h>
#include <ros/time.h>
#include <owd_msgs/AddTrajectory.h>
#include <owd_msgs/AddTimedTrajectory.h>
#include <owd_msgs/GetSpeed.h>
#include "openwam/MacJointTraj.hh"
#include "openwam/Plugin.hh"
// #include "TrajType.hh"

ros::ServiceClient
    left_AddTimedTrajectory,
    right_AddTimedTrajectory,
    left_GetSpeed,
    right_GetSpeed;


bool AddTrajectory(owd_msgs::AddTrajectory::Request &req,
		   owd_msgs::AddTrajectory::Response &res) {

  res.ok=true;
  res.reason="";

  // check number of points
  if (req.traj.positions.size() < 2) {
    ROS_ERROR_NAMED("AddTrajectory","Minimum of 2 traj points required");
    res.id = req.traj.id;
    res.ok=false;
    res.reason="Minimum of 2 traj points required";
    return true;
  }

  // retrieve the current joint speed limits from the right arm
  owd_msgs::GetSpeed speed;
  ROS_DEBUG("Getting right arm speed");
  if (!right_GetSpeed.isValid()) {
    ROS_WARN("Lost connection to /right/owd/GetSpeed; trying to resubscribe...");
    ros::NodeHandle n("~");
    right_GetSpeed = n.serviceClient<owd_msgs::GetSpeed>("/right/owd/GetSpeed",true);
    // try again
    if (!right_GetSpeed.isValid()) {
      ROS_ERROR("Could not reconnect to /right/owd/GetSpeed service: is /right/owd running?");
      res.ok=false;
      res.reason=std::string("Could not reconnect to /right/owd/GetSpeed: is /right/owd running?");
      return true;
    }
    ROS_WARN("Reconnected to /right/owd/GetSpeed service");
  }
  if (!right_GetSpeed.call(speed)) {
    ROS_ERROR("Call to /right/owd/GetSpeed failed");
    res.ok=false;
    res.reason=std::string("Call to /right/owd/GetSpeed failed: is /right/owd running?");
    return true;
  }
  OWD::JointPos joint_vel = speed.response.max_velocity;
  OWD::JointPos joint_accel = speed.response.max_acceleration;
  double max_jerk = speed.response.max_jerk;

  // retrieve the current joint speed limits from the left arm
  ROS_DEBUG("Getting left arm speed");
  if (!left_GetSpeed.isValid()) {
    ROS_WARN("Lost connection to /left/owd/GetSpeed; trying to resubscribe...");
    ros::NodeHandle n("~");
    left_GetSpeed = n.serviceClient<owd_msgs::GetSpeed>("/left/owd/GetSpeed",true);
    // try again
    if (!left_GetSpeed.isValid()) {
      ROS_ERROR("Could not reconnect to /left/owd/GetSpeed service: is /left/owd running?");
      res.ok=false;
      res.reason=std::string("Could not reconnect to /left/owd/GetSpeed: is /left/owd running?");
      return true;
    }
    ROS_WARN("Reconnected to /left/owd/GetSpeed service");
  }
  if (!left_GetSpeed.call(speed)) {
    ROS_ERROR("Call to /left/owd/GetSpeed failed");
    res.ok=false;
    res.reason=std::string("Call to /left/owd/GetSpeed failed: is /left/owd running?");
    return true;
  }

  // combine the right and left speed limits
  joint_vel.insert(joint_vel.end(),
		   speed.response.max_velocity.begin(),
		   speed.response.max_velocity.end());
  joint_accel.insert(joint_accel.end(),
		   speed.response.max_acceleration.begin(),
		   speed.response.max_acceleration.end());
  // take the lowest from the two arms
  if (speed.response.max_jerk < max_jerk) {
    max_jerk = speed.response.max_jerk;
  }

  // if there were not enough blend radii specified, then use
  // zero for any missing ones
  if (req.traj.positions.size() != req.traj.blend_radius.size()) {
    ROS_WARN("Using zero for missing blend_radius values");
    req.traj.blend_radius.resize(req.traj.positions.size(),0);
  }

  OWD::TrajType traj;
  for (unsigned int i=0; i<req.traj.positions.size(); ++i) {
    OWD::JointPos jp = req.traj.positions[i].j;
    OWD::TrajPoint tp(jp,req.traj.blend_radius[i]);
    traj.push_back(tp);
  }

  bool bWaitForStart=(req.traj.options & req.traj.opt_WaitForStart);
  bool bCancelOnStall=(req.traj.options & req.traj.opt_CancelOnStall);
  bool bCancelOnForceInput=(req.traj.options & req.traj.opt_CancelOnForceInput);
  bool bCancelOnTactileInput(req.traj.options & req.traj.opt_CancelOnTactileInput);
  ROS_INFO("Building MacJointTraj with start point %s",traj[0].sdump());
  ROS_INFO("end point %s",traj.back().sdump());
  ROS_INFO("max joint vel %s",joint_vel.sdump());
  ROS_INFO("max joint accel %s",joint_accel.sdump());
  ROS_INFO("max jerk %2.2f, options %d",max_jerk,req.traj.options);

  OWD::MacJointTraj *mjt;
  try {
    mjt = new OWD::MacJointTraj(traj,
				joint_vel, joint_accel, max_jerk,
				bWaitForStart,
				bCancelOnStall,
				bCancelOnForceInput,
				bCancelOnTactileInput);
  } catch (const char *error) {
    char trajectory_error[200];
    snprintf(trajectory_error,200,"Error building blended traj: %s",error);
    ROS_ERROR_NAMED("BuildTrajectory","%s",trajectory_error);
    for (unsigned int tt=0; tt<traj.size(); ++tt) {
      ROS_ERROR_NAMED("BuildTrajectory","  Traj point %d: %s",tt,
		      traj[tt].sdump());
    }
    res.ok=false;
    res.id=std::string("");
    res.reason=std::string(trajectory_error);
    return true;
  } 
  if (req.traj.id == "") {
    mjt->id = OWD::Trajectory::random_id();
  } else {
    mjt->id = req.traj.id;
  }

  owd_msgs::AddTimedTrajectory::Request at_req;
  owd_msgs::AddTimedTrajectory::Response at_res;
  BinaryData bd;
  bd.PutInt(OWD::Trajectory::TRAJTYPE_MACJOINTTRAJ);
  bd.PutString(mjt->serialize(0,6));
  at_req.SerializedTrajectory = bd;
  ROS_WARN("Created timed trajectory for both arms");
  ROS_WARN("Traj will start at %s",mjt->start_position.sdump());
  ROS_WARN("Traj will end at %s",mjt->end_position.sdump());
  ROS_WARN("Serialized synchronized right trajectory into a string of len %ld",
	   at_req.SerializedTrajectory.size());
  at_req.options = owd_msgs::JointTraj::opt_Synchronize;
  at_req.id=mjt->id;
  if (!right_AddTimedTrajectory.isValid()) {
    ROS_WARN("Lost connection to /right/owd/AddTimedTrajectory; trying to resubscribe...");
    ros::NodeHandle n("~");
    right_AddTimedTrajectory = n.serviceClient<owd_msgs::AddTimedTrajectory>("/right/owd/AddTimedTrajectory",true);
    // try again
    if (!right_AddTimedTrajectory.isValid()) {
      ROS_ERROR("Could not reconnect to /right/owd/AddTimedTrajectory service: is /right/owd running?");
      res.ok=false;
      res.reason=std::string("Could not reconnect to /right/owd/AddTimedTrajectory: is /right/owd running?");
      return true;
    }
    ROS_WARN("Reconnected to /right/owd/AddTimedTrajectory service");
  }
  if (!right_AddTimedTrajectory.call(at_req,at_res)) {
    ROS_ERROR("Call to /right/owd/AddTimedTrajectory failed");
    res.ok=false;
    res.reason=std::string("Call to /right/owd/AddTimedTrajectory failed");
    return true;
  }
  if (! at_res.ok) {
    res.ok=false;
    res.reason = at_res.reason;
    ROS_WARN("/right/owd/AddTimedTrajectory failed: %s",res.reason.c_str());
    return true;
  } else {
    ROS_WARN("Synchronized trajectory %s sent to right arm", at_req.id.c_str());
  }

  BinaryData bd2;
  bd2.PutInt(OWD::Trajectory::TRAJTYPE_MACJOINTTRAJ);
  bd2.PutString(mjt->serialize(7,13));
  at_req.SerializedTrajectory = bd2;
  ROS_WARN("Serialized synchronized left trajectory into a string of len %ld",
	   at_req.SerializedTrajectory.size());
  if (!left_AddTimedTrajectory.isValid()) {
    ROS_WARN("Lost connection to /left/owd/AddTimedTrajectory; trying to resubscribe...");
    ros::NodeHandle n("~");
    left_AddTimedTrajectory = n.serviceClient<owd_msgs::AddTimedTrajectory>("/left/owd/AddTimedTrajectory",true);
    // try again
    if (!left_AddTimedTrajectory.isValid()) {
      ROS_ERROR("Could not reconnect to /left/owd/AddTimedTrajectory service: is /left/owd running?");
      res.ok=false;
      res.reason=std::string("Could not reconnect to /left/owd/AddTimedTrajectory: is /left/owd running?");
      return true;
    }
    ROS_WARN("Reconnected to /left/owd/AddTimedTrajectory service");
  }
  if (!left_AddTimedTrajectory.call(at_req,at_res)) {
    ROS_ERROR("Call to /left/owd/AddTimedTrajectory failed");
    res.ok=false;
    res.reason=std::string("Call to /left/owd/AddTimedTrajectory failed");
    return true;
  }
  res.ok=at_res.ok;
  res.id=at_req.id;
  res.reason=at_res.reason;
  if (!res.ok) {
    ROS_WARN("/left/owd/AddTimedTrajectory failed: %s",res.reason.c_str());
  } else {
    ROS_WARN("Synchronized trajectory %s sent to left arm", at_req.id.c_str());
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, std::string("owd_sync"));
  ros::NodeHandle n("~");
  left_AddTimedTrajectory = n.serviceClient<owd_msgs::AddTimedTrajectory>("/left/owd/AddTimedTrajectory",true);
  if (! left_AddTimedTrajectory.exists()) {
    ROS_WARN("Waiting for service /left/owd/AddTimedTrajectory.");
    if (!left_AddTimedTrajectory.waitForExistence()) {
      ROS_WARN("Failed while waiting for /left/owd/AddTimedTrajectory service; exiting");
      return 1;
    }
    ROS_WARN("Connected to service /left/owd/AddTimedTrajectory.");
  }
  right_AddTimedTrajectory = n.serviceClient<owd_msgs::AddTimedTrajectory>("/right/owd/AddTimedTrajectory",true);
  if (! right_AddTimedTrajectory.exists()) {
    ROS_WARN("Waiting for service /right/owd/AddTimedTrajectory.");
    if (!right_AddTimedTrajectory.waitForExistence()) {
      ROS_WARN("Failed while waiting for /right/owd/AddTimedTrajectory service; exiting");
      return 1;
    }
    ROS_WARN("Connected to service /right/owd/AddTimedTrajectory.");
  }
  left_GetSpeed = n.serviceClient<owd_msgs::GetSpeed>("/left/owd/GetSpeed",true);
  if (! left_GetSpeed.exists()) {
    ROS_WARN("Could not connect to service /left/owd/GetSpeed; perhaps the message types have changed?  Waiting for connection.");
    if (!left_GetSpeed.waitForExistence()) {
      ROS_WARN("Failed while waiting for /left/owd/GetSpeed service; exiting");
      return 1;
    }
    ROS_WARN("Connected to service /left/owd/GetSpeed.");
  }
  right_GetSpeed = n.serviceClient<owd_msgs::GetSpeed>("/right/owd/GetSpeed",true);
  if (! right_GetSpeed.exists()) {
    ROS_WARN("Could not connect to service /right/owd/GetSpeed; perhaps the message types have changed?  Waiting for connection.");
    if (!right_GetSpeed.waitForExistence()) {
      ROS_WARN("Failed while waiting for /right/owd/GetSpeed service; exiting");
      return 1;
    }
    ROS_WARN("Connected to service /right/owd/GetSpeed.");
  }
  ros::ServiceServer ss_AddTrajectory = 
    n.advertiseService("AddTrajectory",&AddTrajectory);
  ROS_INFO("owd_traj_timer ready");
  ros::spin();
  return 0;
}
