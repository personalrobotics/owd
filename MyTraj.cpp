#include "MyTraj.hh"
#include <ros/ros.h>


MyTraj::MyTraj(int joint, double distance) {
}

bool MyTraj::AddTrajectory(owd_traj_example::AddMyTrajectory::Request &req,
			   owd_traj_example::AddMyTrajectory::Response &res) {
  ROS_INFO("MyTraj: received AddTrajectory service call");

  // compute a new trajectory
  MyTraj *newtraj = new MyTraj(req.joint,req.distance);

  // send it to the arm
  res.id = Trajectory::AddTrajectory(newtraj,res.reason);
  if (res.id > 0) {
    res.ok=true;
  } else {
    res.ok=false;
  }

  // always return true for the service call so that the client knows that
  // the call was actually processed, as opposed to there being a
  // network communication error.
  // the client will examine the "ok" field to see if the command was
  // actually successful
  return true;
}

void MyTraj::evaluate(double y[], double yd[], double ydd[], double dt) {

  time += dt;

  // this trajectory will last for one second, doing nothing
  if (time > 1) {
    Trajectory::stop();
  }

  return;
}

bool register_trajectory_plugin() {
  ros::NodeHandle n("~");
  static ros::ServiceServer ss_Add = n.advertiseService("AddMyTrajectory",&MyTraj::AddTrajectory);
}

