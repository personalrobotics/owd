/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#ifndef DOORTRAJ_HH
#define DOORTRAJ_HH

#include "GfePlugin.hh"
#include <openwam/MacJointTraj.hh>
#include <gfe_owd_plugin/OpenDoor.h>
#include <pthread.h>
#include <openwam/DataRecorder.cc>
#include <geometry_msgs/Pose.h>

class DoorTraj : public OWD::MacJointTraj {
public:
  DoorTraj(OWD::TrajType &vtraj, gfe_owd_plugin::OpenDoorRequest::_ee_pose_type &eep, R3 PullDirection);
  ~DoorTraj();

  virtual void evaluate(OWD::Trajectory::TrajControl &tc, double dt);

  static bool OpenDoor(gfe_owd_plugin::OpenDoor::Request &req,
		       gfe_owd_plugin::OpenDoor::Response &res);

  /// functions for starting up and shutting down the service
  static bool Register();
  static void Shutdown();

private:
  static std::vector<double> max_j_vel;
  static std::vector<double> max_j_accel;
  static const double maxjerk=10.0*3.141592654;
  static ros::ServiceServer ss_OpenDoor;
  R3 PullDirection;
  R3 endpoint_position_goal;
  R3 last_traj_endpoint;
  std::vector<SE3> endpoint_poses;
  DataRecorder<double> *recorder;
  static void *write_recorder_data(void *);
  static int last_traj_id;
  static pthread_t recorder_thread;
  SE3 last_pose;

  SE3 interpolate_ee_pose(const OWD::JointPos &current_pos);

  class PoseSegment {
  public:
    SE3 starting_pose;
    SE3 pose_shift;
    double start_time;
    double duration;

  PoseSegment(SE3 p1, SE3 p2, double t, double d)
    : starting_pose(p1), pose_shift(p2-p1), start_time(t), duration(d) {
    }

  };

  std::vector<PoseSegment> pose_segments;
  std::vector<PoseSegment>::iterator current_pose_segment;
};

SE3 pose_to_SE3(geometry_msgs::Pose &p);

#endif // DOORTRAJ_HH
