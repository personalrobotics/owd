#ifndef MYTRAJ_HH
#define MYTRAJ_HH

#include <openwam/Trajectory.hh>
#include <openwam/Plugin.hh>
#include <owd_traj_example/AddMyTrajectory.h>
#include <ros/ros.h>

class MyTraj : public OWD::Trajectory {
public:
  MyTraj(int joint, double torque);
  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);
  static bool AddTrajectory(owd_traj_example::AddMyTrajectory::Request &req,
			    owd_traj_example::AddMyTrajectory::Response &res);
  static ros::ServiceServer ss_Add;
  static bool Register();
  static void Shutdown();

private:
  int joint;
  double torque;
};

class MyPlugin : public OWD::Plugin {
public:
  MyPlugin();
  ~MyPlugin();
  virtual void Publish();

  ros::Publisher  pub_info;
};

#endif // MYTRAJ_HH
