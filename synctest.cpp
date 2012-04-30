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
#include <pr_msgs/WAMState.h>
#include <pr_msgs/SetStiffness.h>
#include <pr_msgs/AddTrajectory.h>
#include <pr_msgs/Reset.h>
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
  ros::Subscriber sub_wamstate_l, sub_wamstate_r;
  pr_msgs::Joints p1, p2, p3;
  std::vector<double> h1, h2, h3, h4;
  bool running;
  int point;
  std::string last_traj_id, left_prev_trajectory_id;
  bool left_prev_trajectory_aborted;

  boost::mutex cb_mutex;

  void wamstate_callback_right(const boost::shared_ptr<const pr_msgs::WAMState> &ws) {
    char pos_str[500];
    strcpy(pos_str,"");
    wamstate.positions.resize(14);
    for (unsigned int i=0; i<7; ++i) {
      wamstate.positions[i] = ws->positions[i];
      snprintf(pos_str+strlen(pos_str),499-strlen(pos_str),"%2.2f ",ws->positions[i]);
    }
    //    ROS_INFO("Last right position was %s",pos_str);
    wamstate.prev_trajectory.id = ws->prev_trajectory.id;
    if (running) {
      if ((last_traj_id == "") // first time
          || ((wamstate.prev_trajectory.id == last_traj_id)
	      && (left_prev_trajectory_id == last_traj_id))) {
	if ((wamstate.prev_trajectory.state == wamstate.prev_trajectory.state_aborted) ||
	    left_prev_trajectory_aborted) {
	  ROS_ERROR("Previous trajectory aborted; stopping test");
	  stop();
	  return;
	}

	ROS_INFO("Trajectory %s has ended", last_traj_id.c_str());
	++point;
        if (point == 1) {
          last_traj_id = MoveTo(p1,false);
	  ROS_INFO("moving to point 1; traj id is %s", last_traj_id.c_str());
	} else if (point == 2) {
          last_traj_id = MoveTo(p2,false);
	  ROS_INFO("moving to point 2; traj id is %s", last_traj_id.c_str());
	} else if (point == 3) {
          last_traj_id = MoveTo(p3,false);
	  ROS_INFO("moving to point 3; traj id is %s", last_traj_id.c_str());
	} else {
          last_traj_id = MoveTo(p2,false);
	  ROS_INFO("moving to point 2; traj id is %s", last_traj_id.c_str());
	  point=0;
	}
      }
      //      ROS_DEBUG("Moving to point %d",point);
    }
    return;
  }

  void wamstate_callback_left(const boost::shared_ptr<const pr_msgs::WAMState> &ws) {
    char pos_str[500];
    strcpy(pos_str,"");
    wamstate.positions.resize(14);
    for (unsigned int i=0; i<7; ++i) {
      wamstate.positions[i+7] = ws->positions[i];
      snprintf(pos_str+strlen(pos_str),499-strlen(pos_str),"%2.2f ",ws->positions[i]);
    }
    //    ROS_INFO("Last left position was %s",pos_str);
    left_prev_trajectory_id = ws->prev_trajectory.id;
    left_prev_trajectory_aborted = (ws->prev_trajectory.state == ws->prev_trajectory.state_aborted);
    return;
  }

  Test(ros::NodeHandle &n) : node(n), running(false),
			     point(1), last_traj_id(""),
			     left_prev_trajectory_aborted(false)
  {
    p1.j.resize(14);
    p2.j.resize(14);
    p3.j.resize(14);
    
    p1.j[0]=0.60; 
    p1.j[1]=1.57; 
    p1.j[2]=-1.57;
    p1.j[3]=0.60; 
    p1.j[4]=0;    
    p1.j[5]=0;    
    p1.j[6]=0;    
    p1.j[7]=0.80;
    p1.j[8]=-1.57;
    p1.j[9]=-1.57;
    p1.j[10]=-0.80;
    p1.j[11]=0;
    p1.j[12]=0;
    p1.j[13]=0;

    p2.j[0]=2.14; 
    p2.j[1]=1.57; 
    p2.j[2]=-1.57;
    p2.j[3]=2.70; 
    p2.j[4]=0;    
    p2.j[5]=0;    
    p2.j[6]=0;    
    p2.j[7]=1.0;
    p2.j[8]=-1.57;
    p2.j[9]=-1.57;
    p2.j[10]=2.70;
    p2.j[11]=0;
    p2.j[12]=0;
    p2.j[13]=0;

    p3.j[0]=2.34; 
    p3.j[1]=1.57; 
    p3.j[2]=-1.57;
    p3.j[3]=-0.80; 
    p3.j[4]=0;    
    p3.j[5]=0;    
    p3.j[6]=0;    
    p3.j[7]=2.54;
    p3.j[8]=-1.57;
    p3.j[9]=-1.57;
    p3.j[10]=0.6;
    p3.j[11]=0;
    p3.j[12]=0;
    p3.j[13]=0;

  }

  void Subscribe() {
    sub_wamstate_l =
      node.subscribe("/left/owd/wamstate", 5, &Test::wamstate_callback_left,this);
    sub_wamstate_r =
      node.subscribe("/right/owd/wamstate", 5, &Test::wamstate_callback_right,this);
  }


  void SetStiffness(float s) {
    pr_msgs::SetStiffness::Request req;
    pr_msgs::SetStiffness::Response res;
    
    req.stiffness = s;

    if (! ros::service::call("/right/owd/SetStiffness", req, res) ||
	! ros::service::call("/left/owd/SetStiffness", req, res)) {
      ROS_WARN("Could not call SetStiffness");
    } else {
      ROS_DEBUG("SetStiffness %1.1f",req.stiffness);
    }
  }

  bool move_to_start() {
    if (wamstate.positions.size() < 14) {
      ROS_ERROR("Not receiving WAMState messages");
      return false;
    }
    double rms_distance;
#define SIMULATION
#ifndef SIMULATION
    do {
      int first_far_joint = -1;
      double far_joint_distance=0;
      first_far_joint = -1;
      rms_distance = 0.0;
      for (unsigned int i=0; i<14; ++i) {
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
    
    ROS_WARN("Reached start: moving to p1");
#endif // SIMULATION

    SetStiffness(1.0);
    usleep(500000);  // get a joint update

    last_traj_id = MoveTo(p1,false);
    return true;
  }


  std::string MoveTo(pr_msgs::Joints p, bool StopOnForce) {
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
      // tare the force sensor
      pr_msgs::Reset::Request tare_req;
      pr_msgs::Reset::Response tare_res;
      if (ros::service::call("owd/ft_tare",tare_req, tare_res)) {
	if (tare_res.ok) {
	  ROS_DEBUG("Tared force sensor");
	} else {
	  ROS_WARN("Unable to tare force sensor");
	}
      } else {
	ROS_ERROR("Unable to call ft_tare service");
      }
      traj_req.traj.options = traj_req.traj.opt_CancelOnForceInput;
    }
    if (ros::service::call("owd_sync/AddTrajectory",traj_req,traj_res)) {
      if (!traj_res.ok) {
        ROS_WARN("Adding trajectory failed");
        return std::string("");
      } else {
        return traj_res.id;
      }
    } else {
      ROS_WARN("Could not Add Trajectory");
      return std::string("");
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

  pthread_t mainthread;
  if (pthread_create(&mainthread,NULL,&mainloop,&test)) {
    ROS_FATAL("Could not create main thread");
  }

  ros::spin();
}

