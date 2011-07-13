/***********************************************************************

  Copyright 2011 Carnegie Mellon University
  Author: Kyle Strabala <strabala@cmu.edu>

**********************************************************************/

#include "Follow.h"
#include <time.h>

Follow::Follow(int mode_in): OWD::Trajectory("Follow"), mode(mode_in) {
  zeros = JointPos(std::vector<double>(7,0.0));

  start_position = OWD::Plugin::target_arm_position;
  end_position = start_position;
  
  prev_pos = start_position;
  prev_vel = zeros;
  prev_acc = zeros;

  switch(mode) {
    case gfe_owd_plugin::StartFollow::Request::mode_joy:
      Follow::SetupTrajForJoyTeleop();
      break;
    default:
      mode = 0;
      ROS_ERROR("Follow::Follow: unknown mode integer %d (None=0, Joy=%d). Stopping trajectory.",
                    mode,
                    gfe_owd_plugin::StartFollow::Request::mode_joy);
      runstate=OWD::Trajectory::DONE;
  }
  
  // Setup ROS communication
  ros::NodeHandle n("~");
  joint_state_pub = n.advertise<sensor_msgs::JointState>("follow_joint_state", 1);
  last_joint_state_pub_time = 0.0;
  openrave_service = n.serviceClient<pr_msgs::ArmConfigCheck>("/manipulationapplet/check_for_collision", true);
  last_good_pos = start_position;
  problem = false;
  
  int rc = pthread_create(&ros_comm_thread, NULL, &Follow::StartRosCommThread, this);
  if (rc){
     printf("Error creating ROS comm thread. Return code from pthread_create() is %d\n", rc);
     exit(-1);
  }
  
  pthread_mutex_init(&joy_msg_mutex,NULL);
}

Follow::~Follow() {
  mode = 0;
  joint_state_pub.shutdown();
  if (joy_sub) { //ToDo: Is this right? I want to shutdown joy_sub only if its listening.
    joy_sub.shutdown();
  }
  ROS_INFO("Follow::~Follow: trajectory finished");
}

bool Follow::AddTrajectory(gfe_owd_plugin::StartFollow::Request &req,
			                     gfe_owd_plugin::StartFollow::Response &res) {
  ROS_INFO("Follow::AddTrajectory: received StartFollow service call");

  try {
    Follow *newtraj = new Follow(req.mode);
    res.id = OWD::Plugin::AddTrajectory(newtraj,res.reason);
    if (res.id > 0) {
      res.ok=true;
    } else {
      res.ok=false;
    }
  } catch (const char *err) {
    res.ok=false;
    res.reason=err;
    res.id=0;
  }

  return true;
}

void Follow::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {

  time += dt;

  switch(mode) {
    case gfe_owd_plugin::StartFollow::Request::mode_joy:
      Follow::TrajForJoyTeleop(tc, dt);
      break;
    default:
      tc.q   = tc.q;
      tc.qd  = zeros;
      tc.qdd = zeros;
      tc.t   = zeros;
      ROS_ERROR("Follow::evaluate: unknown mode integer %d (None=0, Joy=%d). Stopping trajectory.",
                    mode,
                    gfe_owd_plugin::StartFollow::Request::mode_joy);
      runstate=OWD::Trajectory::DONE;
      return;
  }

  end_position = tc.q;

  return;
}

bool Follow::Register() {
  ros::NodeHandle n("~");
  ss_Add = n.advertiseService("StartFollow",&Follow::AddTrajectory);
  return true;
}

void Follow::Shutdown() {
  ss_Add.shutdown();
}




void Follow::SetupTrajForJoyTeleop() {
  last_joy_msg_time = -1.0;
  ros::NodeHandle n("~");
  joy_sub = n.subscribe("/spacenav/joy", 1, &Follow::JoyCallback, this);
}

void Follow::JoyCallback(joy::Joy msg) {
  pthread_mutex_lock(&joy_msg_mutex);
  last_joy_msg_time = time;
  last_joy_msg = msg;
  pthread_mutex_unlock(&joy_msg_mutex);
}

void Follow::TrajForJoyTeleop(OWD::Trajectory::TrajControl &tc, double dt) {
  if (time - last_joy_msg_time > 0.5) { //ToDo: What about during stalls? dt=0, so this may never execute.
    tc.q   = tc.q;
    tc.qd  = zeros;
    tc.qdd = zeros;
    tc.t   = zeros;
    prev_pos = tc.q;
    prev_vel = zeros;
    prev_acc = zeros;
    return;
  }
  // Copy latest message
  pthread_mutex_lock(&joy_msg_mutex);
  joy::Joy joy_msg = last_joy_msg;
  pthread_mutex_unlock(&joy_msg_mutex);
  
  JointPos pos(std::vector<double>(7,0.0));
  JointPos vel(std::vector<double>(7,0.0));
  JointPos acc(std::vector<double>(7,0.0));

  // Calculate desired hand velocity
  // joy_msg.buttons = [btn1, btn2]
  // joy_msg.axes = [vx, vy, vz, wx, wy, wz]
  R6 vel_des(joy_msg.axes[2], joy_msg.axes[1], -joy_msg.axes[0], joy_msg.axes[5], joy_msg.axes[4], -joy_msg.axes[3]);
  double joy_scale = 1.0;
  for(int i = 0; i < 6; i++) {
    double vel_sign = (vel_des[i] >= 0.0)?1.0:-1.0;
    vel_des[i] = joy_scale*vel_sign*(vel_des[i]/0.68)*(vel_des[i]/0.68);
  }

  // Calculate dq and vel
  JointPos dq_pose = OWD::Plugin::JacobianPseudoInverse_times_vector(vel_des);
  JointPos q_offset = start_position - tc.q;
  // ToDo: q_offset: hardcode preferred joint config instead of using start_position config
  // ToDo: q_offset: maybe use weighted nullspace to make base joints move less for rotation changes
  JointPos dq_offset = OWD::Plugin::Nullspace_projection(q_offset);
  vel = dq_pose * 1.0 + dq_offset * 1.0;

  // ToDo: Maybe smooth velocity commands

  // Coerce to within angular velocity limits
  double vel_max = 0.0;
  for (uint i = 0; i < vel.size(); i++) {
    if (abs(vel[i]) > vel_max) {
      vel_max = abs(vel[i]);
    }
  }
  double vel_limit = 1.0;
  if (vel_max > vel_limit) {
    vel = vel*(vel_limit/vel_max);
  }
  
  if (problem == true) {
    if (vel*(last_good_pos - tc.q) < 0) {
      vel = zeros;
    }
  }
  
  // Check for joint limits. Project arm 0.5sec into the future
  JointPos q_future = prev_pos + vel*0.5;
  // ToDo: stop arm if q_future exceeds joint limits

  // Set variables
  pos = prev_pos + vel*dt;
  acc = zeros;
  
  // Save values for next iteration
  prev_pos = pos;
  prev_vel = vel;
  prev_acc = acc;

  // Set command values
  tc.q   = pos;
  tc.qd  = zeros;
  tc.qdd = zeros;
  tc.t   = zeros;
}


void *Follow::StartRosCommThread(void *data) {
  Follow *f = (Follow*) data;
  f->RosComm();
  return NULL;
}

void Follow::RosComm() {
  while (mode != 0) {
    sensor_msgs::JointState joint_state_msg;
    
    joint_state_msg.header.frame_id = ros::this_node::getName();
    joint_state_msg.header.stamp    = ros::Time::now();
    joint_state_msg.position = prev_pos;
    joint_state_msg.velocity = prev_vel;
    joint_state_msg.effort = prev_acc;
    
    joint_state_pub.publish(joint_state_msg);
    last_joint_state_pub_time = time;
    
    if(openrave_service) {
      pr_msgs::ArmConfigCheck srv;
      srv.request.joint_state = joint_state_msg;
      if (openrave_service.call(srv)) {
        bool self_collision = srv.response.current_self_collision || srv.response.future_self_collision;
        bool env_collision = srv.response.current_env_collision || srv.response.future_env_collision;
        bool joint_limits = srv.response.current_joint_limits_exceeded || srv.response.future_joint_limits_exceeded;
        problem = self_collision || env_collision || joint_limits;
        if (problem == false) {
          last_good_pos = joint_state_msg.position;
        }
        ROS_DEBUG("Problem: %s", problem?"True":"False");
      } else {
        ROS_DEBUG("openrave_service did not execute properly.");
        problem = false;
      }
    } else {
      ROS_DEBUG("openrave_service is not running.");
      problem = false;
    }
    
    if (mode == 0) break;
    
    sleep(0.1);
  }
}

