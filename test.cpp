/***********************************************************************

  Copyright 2009-2010 Carnegie Mellon University and Intel Corporation
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
#include <pr_msgs/WAMState.h>
#include <pr_msgs/SetStiffness.h>
#include <pr_msgs/AddTrajectory.h>
// #include <pr_msgs/CancelTrajectory.h>
#include <pr_msgs/DeleteTrajectory.h>
#include <pr_msgs/JointTraj.h>
#include <pr_msgs/Joints.h>
#include <pr_msgs/TrajInfo.h>
#include <iostream>
#include <boost/thread/mutex.hpp>
#include <pthread.h>

class Test {
public:
  pr_msgs::WAMState wamstate, lastwamstate;
  ros::NodeHandle &node;
  ros::Subscriber sub_wamstate;
  pr_msgs::Joints p1, p2;
  bool running;
  int point;
  unsigned int last_traj_id;

  boost::mutex cb_mutex;

  void wamstate_callback(const boost::shared_ptr<const pr_msgs::WAMState> &ws) {
    wamstate = *ws;
    if (running) {
      if ((wamstate.trajectory_queue.size() > 0) &&
          (wamstate.trajectory_queue[0].state == pr_msgs::TrajInfo::state_paused)) {
        // reverse direction shortly after it stalls
        DeleteTrajectory(wamstate.trajectory_queue[0].id);
        return;
      }
      if ((last_traj_id == 0) // first time
          || (wamstate.prev_trajectory.id == last_traj_id)) {
        if (++point > 2) {
          point = 1;
        }
        if (point == 1) {
          last_traj_id = MoveTo(p1,false);
        } else {
          last_traj_id = MoveTo(p2,true);
        }
      }
      ROS_DEBUG("Moving to point %d",point);
    }
    return;
  }

  Test(ros::NodeHandle &n) : node(n), running(false),
                       point(0), last_traj_id(0)
  {
    p1.j.resize(7);
    p2.j.resize(7);
    static const double PI=3.141592654;
    /*
    p1.j[0]=PI/2.0;      p2.j[0]=-PI/2;
    p1.j[1]=0.62;        p2.j[1]=-1.6;
    p1.j[2]=0;           p2.j[2]=0;
    p1.j[3]=2.0;         p2.j[3]=2.4;
    p1.j[4]=0.0;         p2.j[4]=0;
    p1.j[5]=0.4;         p2.j[5]=-1;
    p1.j[6]=0.0;         p2.j[6]=PI/2;
*/
    p1.j[0]=4.76;      p2.j[0]=4.76;
    p1.j[1]=-1.93;        p2.j[1]=-1.16;
    p1.j[2]=0;           p2.j[2]=0;
    p1.j[3]=2.16;         p2.j[3]=1.76;
    p1.j[4]=1.25;         p2.j[4]=1.25;
    p1.j[5]=0.0;         p2.j[5]=0.0;
    p1.j[6]=0;         p2.j[6]=0;
    
  }

  void Subscribe() {
    sub_wamstate =
      node.subscribe("owd/wamstate", 5, &Test::wamstate_callback,this);
  }


  void SetStiffness(float s) {
    pr_msgs::SetStiffness::Request req;
    pr_msgs::SetStiffness::Response res;
    
    req.stiffness = s;

    if(ros::service::call("owd/SetStiffness", req, res)) {
      ROS_DEBUG("SetStiffness %1.1f",req.stiffness);
    } else {
      ROS_WARN("Could not call SetStiffness");
    }
  }

  void DeleteTrajectory(int id) {
    pr_msgs::DeleteTrajectory::Request delete_req;
    pr_msgs::DeleteTrajectory::Response delete_res;
    delete_req.ids.push_back(id);
    ros::service::call("owd/DeleteTrajectory",delete_req,delete_res);
    return;
  }

  bool move_to_start() {
    if (wamstate.positions.size() < 7) {
      ROS_ERROR("Not receiving WAMState messages");
      return false;
    }
    double rms_distance;
    do {
      int first_far_joint = -1;
      double far_joint_distance=0;
      first_far_joint = -1;
      rms_distance = 0.0;
      for (unsigned int i=0; i<7; ++i) {
        double joint_dist = pow(p1.j[i]
          - wamstate.positions[i],2);
        rms_distance += joint_dist;
        if ((first_far_joint == -1) &&
            (joint_dist > 0.1)) {
          first_far_joint = i;
          far_joint_distance = sqrt(joint_dist);
        }
      }
      rms_distance = sqrt(rms_distance);
      if (rms_distance > 0.84) {
        ROS_WARN("Too far from preferred position: joint %d error is %2.2f",
                 first_far_joint+1,far_joint_distance);
        sleep(1);
      }
    } while (rms_distance > 0.84);
    ROS_WARN("Moving to p1");
    SetStiffness(1.0);
    usleep(300000);  // get a joint update

    last_traj_id = MoveTo(p1,false);
    return true;
  }

  int MoveTo(pr_msgs::Joints p, bool StopOnForce) {
    // build trajectory to move from current pos to preferred pos
    pr_msgs::AddTrajectory::Request traj_req;
    pr_msgs::AddTrajectory::Response traj_res;
    pr_msgs::Joints pos;
    pos.j = wamstate.positions;
    traj_req.traj.positions.push_back(pos);
    traj_req.traj.blend_radius.push_back(0.0);

    traj_req.traj.positions.push_back(p);
    traj_req.traj.blend_radius.push_back(0.0);
    
    if (StopOnForce) {
      traj_req.traj.options = traj_req.traj.opt_CancelOnForceInput;
    }
    if (ros::service::call("owd/AddTrajectory",traj_req,traj_res)) {
      ROS_DEBUG("Added Trajectory %d",traj_res.id);
      if (traj_res.id == 0) {
        ROS_WARN("Adding trajectory failed");
        return -2;
      } else {
        return traj_res.id;
      }
    } else {
      ROS_WARN("Could not Add Trajectory");
      return -2;
    }
  }

  void go() {
    while (!move_to_start()) {
      sleep(1);
    }
    running=true;
  }

  void stop() {
    running=false;
    SetStiffness(0.0);
  }
};

void *mainloop(void *p) {
  Test *test=(Test*)p;
  while (1)
  {
    std::string s;
    ROS_WARN("Waiting to start trajectory test");
    std::cin >> s;

    ROS_WARN("Running...");
    test->go();
      
    std::cin >> s;
    ROS_WARN("Stopping...");

    test->stop();

  }
  return NULL;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "owd_test", 0);

  ros::NodeHandle n;
  Test test(n);
  test.Subscribe();

  test.SetStiffness(0.0);
  //  wagon.CancelTrajectory();
  // test.DeleteTrajectory(1);

  pthread_t mainthread;
  if (pthread_create(&mainthread,NULL,&mainloop,&test)) {
    ROS_FATAL("Could not create main thread");
  }

  ros::spin();
}

