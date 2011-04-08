#ifndef MYTRAJ_HH
#define MYTRAJ_HH

#include <openwam/Trajectory.hh>
#include "owd_traj_example/AddMyTrajectory.h"
#include <ros/ros.h>

bool register_trajectory_plugin();

class MyTraj : public Trajectory {
public:
  MyTraj(int joint, double distance);
  ~MyTraj();
  virtual void evaluate(double y[], double yd[], double ydd[], double dt);
  static bool AddTrajectory(owd_traj_example::AddMyTrajectory::Request &req,
			    owd_traj_example::AddMyTrajectory::Response &res);

  static ros::ServiceServer ss_Add;

};


#endif // MYTRAJ_HH
