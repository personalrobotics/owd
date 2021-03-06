/***********************************************************************

  Copyright 2013 Carnegie Mellon University
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#define PEAK_CAN
#include "openwam/CANdefs.hh"	// for HANDSTATE_* enumeration
#include "openwamdriver.h"
#include "CoggingCompTraj.h"

CoggingCompTraj::CoggingCompTraj(int _joint,
                                 double _max_torque,
                                 double _max_step,
                                 double _hold_duration,
                                 int _num_samples):
  OWD::Trajectory("CoggingCompTraj",""),
  joint(_joint), 
  max_torque(_max_torque),
  max_step(_max_step),
  hold_duration(_hold_duration),
  num_samples(_num_samples),
  first(true)
{
  if ((joint<1) || (joint>7)) {
    throw "Joint out of range";
  }
  if (fabs(max_torque)>1 ) {
    throw "Torque limited to 1 nm";
  }
  start_position=OWD::Plugin::target_arm_position;
  end_position = start_position;
  response.data.clear();
  running=true;
}

bool CoggingCompTraj::AddTrajectory(owd_plugins::CoggingComp::Request &req,
                                  owd_plugins::CoggingComp::Response &res) {

  // compute a new trajectory
  try {
    CoggingCompTraj *newtraj = new CoggingCompTraj(req.joint,
                                                   req.max_torque,
                                                   req.max_step,
                                                   req.hold_time,
                                                   req.num_samples);
    // send it to the arm
    std::string reason;
    int id = OWD::Plugin::AddTrajectory(newtraj,reason);
    if (id > 0) {
      ROS_INFO("Beginning measurement of motor cogging");
      // wait for the traj to finish
      while (running) {
        usleep(100000);
      }
      res=response;
      ROS_INFO("Cogging data collection complete");
    } else {
      running=false;
      delete newtraj;
      ROS_INFO("Cogging data collection failed: %s",reason.c_str());
    }
  } catch (const char *err) {
    ROS_ERROR("Could not start CoggingComp trajectory: %s",err);
    running=false;
  }

  return true;
}

void CoggingCompTraj::evaluate_abs(OWD::Trajectory::TrajControl &tc, double t) {
  // if position has jumped
  //   done, write out data
  // if t<settle
  //    same torque as before
  // else if t<sample
  //     save data
  // else
  //     increase torque, reset time
  if (first) {
    first=false;
    current_torque=0;
    sample_time=t+hold_duration;
    sample_count=0;
    fixed_pos = tc.q;
    last_position = tc.q[joint-1];
  }
  // hold other joints at their starting position, but let the joint
  // we are testing drift due to the test torque
  fixed_pos[joint-1]=tc.q[joint-1];
  tc.q = fixed_pos;
  // check for sudden movement
  if (fabs(tc.q[joint-1] - last_position) > max_step) {
    ROS_INFO("Stopping due to max step condition (last=%f, current=%f",
	     last_position,tc.q[joint-1]);
    runstate=OWD::Trajectory::DONE;
    running=false;
    return;
  }
  // check for start of sampling
  if (t>sample_time) {
    owd_plugins::CogSample sample;
    // motor position
    sample.position=OWD::WamDriver::bus->rawpos[joint] - OWD::WamDriver::owam->motors[joint-1].offset;
    while (sample.position < 0) {
      sample.position += 4096;
    }
    while (sample.position > 4095) {
      sample.position -= 4096;
    }
    // old (joint position): sample.position=tc.q[joint-1];  
    sample.torque=current_torque;
    response.data.push_back(sample);
    if (++sample_count == num_samples) {
      ROS_INFO("At t=%2.4f menc=%d, jq=%2.4f",
	       current_torque,sample.position,tc.q[joint-1]);
      // increase the torque
      if (max_torque > 0) {
        current_torque += 0.005;
        if (current_torque > max_torque) {
          runstate=OWD::Trajectory::DONE;
          running=false;
        }
      } else {
        current_torque -= 0.005;
        if (current_torque < max_torque) {
          runstate=OWD::Trajectory::DONE;
          running=false;
        }
      }
      sample_time=t+hold_duration;
      sample_count=0;
    }
  }
  // hold the current torque
  tc.t[joint-1] = current_torque;
  // keep tracking the current position
  end_position =tc.q;
  last_position = tc.q[joint-1];

  return;
}

bool CoggingCompTraj::Register() {
  ros::NodeHandle n("~");
  ss_Add = n.advertiseService("CoggingComp",&CoggingCompTraj::AddTrajectory);
  return true;
}

void CoggingCompTraj::Shutdown() {
  ss_Add.shutdown();
}

ros::ServiceServer CoggingCompTraj::ss_Add;
owd_plugins::CoggingComp::Response CoggingCompTraj::response;
bool CoggingCompTraj::running;

