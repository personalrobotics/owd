/***********************************************************************

  Copyright 2013 Carnegie Mellon University
  Author: Mike Vande Weghe <vandeweg@cmu.edu>


**********************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#define PEAK_CAN
#include "openwam/CANdefs.hh"	// for HANDSTATE_* enumeration
#include "openwamdriver.h"
#include "TeachandPlay.h"


TeachTraj::TeachTraj(int hz) :
  OWD::Trajectory("Teach",""),
  last_sample_time(0),
  sample_interval(1.0/hz)
{
  taughttraj.clear();
  start_position=OWD::Plugin::target_arm_position;
  end_position = start_position;
  running=true;
  stopteach=false;
}

void TeachTraj::evaluate_abs(OWD::Trajectory::TrajControl &tc, double t) {
  end_position=tc.q;
  if (stopteach) {
    runstate=DONE;
    return;
  }
  if ((t-last_sample_time) < sample_interval) {
    return;
  }
  owd_plugins::TrajPoint tp;
  tp.time=last_sample_time=t;
  tp.angles = tc.q;
  taughttraj.push_back(tp);
  unsigned int n=taughttraj.size();
  if (n > 1) {
    // the velocity is set for the previous sample
    taughttraj[n-2].velocities = OWD::Plugin::arm_velocity;
  }
}

TeachTraj::~TeachTraj() {
  running=false;
}

bool TeachTraj::Teach(owd_plugins::Teach::Request &req,
		      owd_plugins::Teach::Response &res) {
  // compute a new trajectory
  try {
    TeachTraj *newtraj = new TeachTraj();

    // send it to the arm
    std::string reason;
    int id = OWD::Plugin::AddTrajectory(newtraj,reason);
    if (id > 0) {
      ROS_INFO("Beginning teach mode; call StopTeach service to end");
    } else {
      running=false;
      delete newtraj;
      ROS_INFO("Teaching failed: %s",reason.c_str());
    }
  } catch (const char *err) {
    ROS_ERROR("Could not start Teach trajectory: %s",err);
    running=false;
  }
  return true;
}

bool TeachTraj::StopTeach(owd_plugins::StopTeach::Request &req,
			  owd_plugins::StopTeach::Response &res) {
  stopteach=true;
  ROS_INFO("Teaching complete");
  // wait for the traj to finish
  while (running) {
    usleep(100000);
  }
  unsigned int dof=taughttraj.front().angles.size();
  taughttraj.back().velocities.resize(dof,0);
  // set all the accelerations from the velocity diffs
  for (unsigned int i=0; i<taughttraj.size()-1; ++i) {
    taughttraj[i].accelerations.resize(dof);
    double delta_t = taughttraj[i+1].time - taughttraj[i].time;
    for (unsigned int j=0; j<dof; ++j) {
      taughttraj[i].accelerations[j] = 
	(taughttraj[i+1].velocities[j] -
	 taughttraj[i].velocities[j])
      / delta_t;
    }
  }
  taughttraj.back().accelerations.resize(dof,0);
  res.traj=taughttraj;
  return true;
}

bool TeachTraj::Register() {
  ros::NodeHandle n("~");
  ss_Teach = n.advertiseService("Teach",&TeachTraj::Teach);
  ss_StopTeach = n.advertiseService("StopTeach",&TeachTraj::StopTeach);
  return true;
}

void TeachTraj::Shutdown() {
  ss_Teach.shutdown();
}

PlayTraj::PlayTraj(std::vector<owd_plugins::TrajPoint> traj):
  OWD::Trajectory("Play",""),
  playtraj(traj),
  i(0)
{
  // check dimensions of all fields
}

void PlayTraj::evaluate_abs(OWD::Trajectory::TrajControl &tc, double t) {
  // move forward until t is less than time at i+1
  while ((i+2 < playtraj.size()) && (t>=playtraj[i+1].time)) {
    ++i;
  }
  if (t>playtraj[i+1].time) {
    tc.q = end_position;
    runstate=DONE;
    return;
  }
  // move backwards until t is greater than or equal to time at i
  while ((i>0) && (t<playtraj[i].time)) {
    --i;
  }
  if (t<playtraj[i].time) {
    tc.q=playtraj[i].angles;
    tc.qd = playtraj[i].velocities;
    tc.qdd = playtraj[i].accelerations;
    return;
  }
  // linearly interpolate between i and i+1
  double fraction=(t-playtraj[i].time) / (playtraj[i+1].time - playtraj[i].time);
  for (unsigned int j=0; j<tc.q.size(); ++j) {
    tc.q[j]=fraction*playtraj[i].angles[j] + (1-fraction)*playtraj[i+1].angles[j];
    tc.qd[j]=fraction*playtraj[i].velocities[j] + (1-fraction)*playtraj[i+1].velocities[j];
    tc.qdd[j]=fraction*playtraj[i].accelerations[j] + (1-fraction)*playtraj[i+1].accelerations[j];
  }
  return;
}

bool PlayTraj::Play(owd_plugins::Play::Request &req,
		    owd_plugins::Play::Response &res) {
  try {
    PlayTraj *newtraj = new PlayTraj(req.traj);

    // send it to the arm
    std::string reason;
    int id = OWD::Plugin::AddTrajectory(newtraj,reason);
    if (id > 0) {
      ROS_INFO("Beginning playback");
    } else {
      delete newtraj;
      ROS_INFO("Playback failed: %s",reason.c_str());
    }
  } catch (const char *err) {
    ROS_ERROR("Could not start Play trajectory: %s",err);
  }
  return true;
}

bool PlayTraj::Register() {
  ros::NodeHandle n("~");
  ss_Play = n.advertiseService("Play",&PlayTraj::Play);
  return true;
}

void PlayTraj::Shutdown() {
  ss_Play.shutdown();
}

ros::ServiceServer TeachTraj::ss_Teach, TeachTraj::ss_StopTeach;
ros::ServiceServer PlayTraj::ss_Play;
std::vector<owd_plugins::TrajPoint> TeachTraj::taughttraj;
bool TeachTraj::running;
bool TeachTraj::stopteach;

