/***********************************************************************

  Copyright 2011 Carnegie Mellon University
  Author: Kyle Strabala <strabala@cmu.edu>

**********************************************************************/

#include "Teleop.h"
#include <time.h>
#include <iostream>

Teleop::Teleop(int mode_in, std::string input_topic): OWD::Trajectory("Teleop",""), mode(mode_in) {
  zeros = OWD::JointPos(std::vector<double>(7,0.0));

  start_position = OWD::Plugin::target_arm_position;
  end_position = start_position;
  
  prev_pos = start_position;
  prev_vel = zeros;
  prev_acc = zeros;

  switch(mode) {
    case MODE_SPACENAV:
      Teleop::SetupTrajForSpacenavTeleop(input_topic);
      break;
    case MODE_OMNI:
      Teleop::SetupTrajForOmniTeleop(input_topic);
      break;
    default:
      mode = MODE_NONE;
      PrintModeError(mode);
      runstate=OWD::Trajectory::DONE;
  }
  
  // Setup mutex locks
  pthread_mutex_init(&spacenav_msg_mutex,NULL);
  pthread_mutex_init(&omni_msg_mutex,NULL);
  
}

Teleop::~Teleop() {
  mode = 0;
  if (spacenav_sub) { //ToDo: Is this right? I want to shutdown spacenav_sub only if its listening.
    spacenav_sub.shutdown();
  }
  if (omni_sub) { //ToDo: Is this right? I want to shutdown omni_sub only if its listening.
    omni_sub.shutdown();
  }
  ROS_INFO("Teleop::~Teleop: trajectory finished");
}

void Teleop::PrintModeError(int mode_in) {
      ROS_ERROR("Teleop::Teleop: unknown mode integer %d (None=0, Spacenav=%d, Omni=%d). Stopping trajectory.",
                    mode_in,
                    MODE_SPACENAV,
                    MODE_OMNI);
}

bool Teleop::StartTeleopViaSpacenavCallback(
        owd_msgs::StartTeleop::Request &req,
        owd_msgs::StartTeleop::Response &res) {
    return Teleop::AddTrajectory(req, res, MODE_SPACENAV);
}

bool Teleop::StartTeleopViaOmniCallback(
        owd_msgs::StartTeleop::Request &req,
        owd_msgs::StartTeleop::Response &res) {
    return Teleop::AddTrajectory(req, res, MODE_OMNI);
}

bool Teleop::AddTrajectory(owd_msgs::StartTeleop::Request &req,
                           owd_msgs::StartTeleop::Response &res,
                           int mode_in) {
  ROS_INFO("Teleop::AddTrajectory: received StartTeleop service call with mode=%d and topic=\"%s\"", mode_in, req.input_topic.c_str());

  try {
    Teleop *newtraj = new Teleop(mode_in, req.input_topic);
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

void Teleop::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {

  time += dt;

  switch(mode) {
    case MODE_SPACENAV:
      Teleop::TrajForSpacenavTeleop(tc, dt);
      break;
    case MODE_OMNI:
      Teleop::TrajForOmniTeleop(tc, dt);
      break;
    default:
      tc.q   = tc.q;
      tc.qd  = zeros;
      tc.qdd = zeros;
      tc.t   = zeros;
      PrintModeError(mode);
      runstate=OWD::Trajectory::DONE;
      return;
  }
  prev_pos = tc.q;
  prev_vel = tc.qd;
  prev_acc = tc.qdd;

  end_position = tc.q;

  return;
}


void Teleop::evaluate_abs(OWD::Trajectory::TrajControl &tc, double dt) {

  time += dt;

  switch(mode) {
    case MODE_SPACENAV:
      Teleop::TrajForSpacenavTeleop(tc, dt);
      break;
    case MODE_OMNI:
      Teleop::TrajForOmniTeleop(tc, dt);
      break;
    default:
      tc.q   = tc.q;
      tc.qd  = zeros;
      tc.qdd = zeros;
      tc.t   = zeros;
      PrintModeError(mode);
      runstate=OWD::Trajectory::DONE;
      return;
  }
  prev_pos = tc.q;
  prev_vel = tc.qd;
  prev_acc = tc.qdd;

  end_position = tc.q;

  return;
}

ros::ServiceServer Teleop::ss_spacenav,
                   Teleop::ss_omni;
  
bool Teleop::Register() {
  ros::NodeHandle n("~");
  ss_spacenav = n.advertiseService("StartTeleopViaSpacenav", &Teleop::StartTeleopViaSpacenavCallback);
  ss_omni     = n.advertiseService("StartTeleopViaOmni",     &Teleop::StartTeleopViaOmniCallback);
  return true;
}

void Teleop::Shutdown() {
  ss_spacenav.shutdown();
  ss_omni.shutdown();
}




void Teleop::SetupTrajForSpacenavTeleop(std::string input_topic) {
  last_spacenav_msg_time = -1.0;
  ros::NodeHandle n("~");
  std::string default_input_topic("/spacenav/joy");
  if (input_topic.empty()) { 
      ROS_INFO("Teleop::SetupTrajForSpacenavTeleop: input_topic string is empty, using default_input_topic");
      input_topic = default_input_topic;
  }
  ROS_INFO("Teleop::SetupTrajForSpacenavTeleop: subscribing to \"%s\"", input_topic.c_str());
  spacenav_sub = n.subscribe(input_topic.c_str(), 1, &Teleop::SpacenavCallback, this);
}

void Teleop::SpacenavCallback(sensor_msgs::Joy msg) {
  pthread_mutex_lock(&spacenav_msg_mutex);
  last_spacenav_msg_time = time;
  last_spacenav_msg = msg;
  pthread_mutex_unlock(&spacenav_msg_mutex);
}

void Teleop::TrajForSpacenavTeleop(OWD::Trajectory::TrajControl &tc, double dt) {
  if (time - last_spacenav_msg_time > 0.5) { //ToDo: What about during stalls? dt=0, so this may never execute.
    tc.q   = prev_pos;
    tc.qd  = zeros;
    tc.qdd = zeros;
    tc.t   = zeros;
    return;
  }
  // Copy latest message
  pthread_mutex_lock(&spacenav_msg_mutex);
  sensor_msgs::Joy spacenav_msg = last_spacenav_msg;
  pthread_mutex_unlock(&spacenav_msg_mutex);
  
  OWD::JointPos pos(std::vector<double>(7,0.0));
  OWD::JointPos vel(std::vector<double>(7,0.0));
  OWD::JointPos acc(std::vector<double>(7,0.0));

  // Calculate desired hand velocity
  // spacenav_msg.buttons = [btn1, btn2]
  // spacenav_msg.axes = [vx, vy, vz, wx, wy, wz]
  R6 vel_des(spacenav_msg.axes[2], -spacenav_msg.axes[1], spacenav_msg.axes[0], spacenav_msg.axes[5], -spacenav_msg.axes[4], spacenav_msg.axes[3]);
  double spacenav_scale = 0.5;
  double vel_sign = 0.0;
  for(int i = 0; i < 3; i++) {
    vel_sign = (vel_des.v.x[i] >= 0.0)?1.0:-1.0;
    vel_des.v.x[i] = spacenav_scale*vel_sign*(vel_des.v.x[i]/0.68)*(vel_des.v.x[i]/0.68);
    
    vel_sign = (vel_des.w.x[i] >= 0.0)?1.0:-1.0;
    vel_des.w.x[i] = spacenav_scale*vel_sign*(vel_des.w.x[i]/0.68)*(vel_des.w.x[i]/0.68);
  }

  // Calculate dq and vel
  OWD::JointPos dq_pose = OWD::Plugin::JacobianPseudoInverse_times_vector(vel_des);
  OWD::JointPos q_offset = start_position - tc.q;
  // ToDo: q_offset: hardcode preferred joint config instead of using start_position config
  // ToDo: q_offset: maybe use weighted nullspace to make base joints move less for rotation changes
  OWD::JointPos dq_offset = OWD::Plugin::Nullspace_projection(q_offset);
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
  
  // Check for joint limits. Project arm 0.5sec into the future
  OWD::JointPos q_future = prev_pos + vel*0.5;
  // ToDo: stop arm if q_future exceeds joint limits

  // Set variables
  pos = prev_pos + vel*dt;
  acc = zeros;

  // Set command values
  tc.q   = pos;
  tc.qd  = zeros;
  tc.qdd = zeros;
  tc.t   = zeros;
}




void Teleop::SetupTrajForOmniTeleop(std::string input_topic) {
  last_omni_msg_time = -1.0;
  clutch = true;
  ros::NodeHandle n("~");
  std::string default_input_topic("/phantom_omni/state");
  if (input_topic.empty()) { 
      ROS_INFO("Teleop::SetupTrajForOmniTeleop: input_topic string is empty, using default_input_topic");
      input_topic = default_input_topic;
  }
  ROS_INFO("Teleop::SetupTrajForOmniTeleop: subscribing to \"%s\"", input_topic.c_str());
  omni_sub = n.subscribe(input_topic.c_str(), 1, &Teleop::OmniCallback, this);
}

void Teleop::OmniCallback(phantom_omni::OmniState msg) {
  pthread_mutex_lock(&omni_msg_mutex);
  last_omni_msg_time = time;
  last_omni_msg = msg;
  pthread_mutex_unlock(&omni_msg_mutex);
}

void Teleop::TrajForOmniTeleop(OWD::Trajectory::TrajControl &tc, double dt) {
  if (time - last_omni_msg_time > 0.5) { //ToDo: What about during stalls? dt=0, so this may never execute.
    clutch = true;
    tc.q   = prev_pos;
    tc.qd  = zeros;
    tc.qdd = zeros;
    tc.t   = zeros;
    return;
  }
  // Copy latest messages
  pthread_mutex_lock(&omni_msg_mutex);
  phantom_omni::OmniState omni_msg           = last_omni_msg;
  pthread_mutex_unlock(&omni_msg_mutex);
  
  OWD::JointPos pos(std::vector<double>(7,0.0));
  OWD::JointPos vel(std::vector<double>(7,0.0));
  OWD::JointPos acc(std::vector<double>(7,0.0));

  // Get the hand pose
  //btVector3    omni_hand (omni_msg.pose.position.z, -omni_msg.pose.position.y, omni_msg.pose.position.x);
  //btQuaternion omni_quat (omni_msg.pose.orientation.z, -omni_msg.pose.orientation.y, omni_msg.pose.orientation.x, omni_msg.pose.orientation.w);
  btVector3    omni_hand (omni_msg.pose.position.y, omni_msg.pose.position.x, -omni_msg.pose.position.z);
  btQuaternion omni_quat (omni_msg.pose.orientation.y, omni_msg.pose.orientation.x, -omni_msg.pose.orientation.z, omni_msg.pose.orientation.w);

  // Get current robot hand pose
  R3 robot_hand_R3 = (R3) OWD::Plugin::endpoint;
  so3 endpoint_axis_angle = (so3) OWD::Plugin::endpoint;
  btVector3    robot_hand (robot_hand_R3[0], robot_hand_R3[1], robot_hand_R3[2]);
  btQuaternion robot_quat (btVector3(endpoint_axis_angle.w()[0], endpoint_axis_angle.w()[1], endpoint_axis_angle.w()[2]), endpoint_axis_angle.t());
  
  if (omni_msg.buttons[0] > 0 || omni_msg.buttons[1] > 0) {
    if (clutch == true) {
      clutch = false;
      clutch_omni_hand  = omni_hand;
      clutch_robot_hand = robot_hand;
      clutch_omni_quat  = omni_quat;
      clutch_robot_quat = robot_quat;
    }
  }
  else {
    clutch = true;
    tc.q   = prev_pos;
    tc.qd  = zeros;
    tc.qdd = zeros;
    tc.t   = zeros;
    return;
  }
  
  // Calculate desired hand velocity
  btVector3 omni_hand_diff  = omni_hand - clutch_omni_hand;
  btVector3 robot_hand_diff = robot_hand - clutch_robot_hand;
  btVector3 error_hand(0.0, 0.0, 0.0);
  error_hand = 5.0*omni_hand_diff - robot_hand_diff;
  btQuaternion error_quat;
  btVector3 error_axis;
  btScalar error_angle;
  btQuaternion omni_quat_diff  = clutch_omni_quat.inverse() * omni_quat;
  btQuaternion robot_quat_diff = clutch_robot_quat.inverse() * robot_quat;
  error_quat  = clutch_omni_quat.inverse() * clutch_robot_quat * robot_quat.inverse() * omni_quat;
  error_axis  = error_quat.getAxis();
  error_angle = error_quat.getAngle();

  error_hand  = 0.5*error_hand;
  error_angle = 0.8*error_angle;
  R6 vel_des(error_hand[0], error_hand[1], error_hand[2], error_angle*error_axis[0], error_angle*error_axis[1], error_angle*error_axis[2]);
  
  // Calculate dq and vel
  OWD::JointPos dq_pose = OWD::Plugin::JacobianPseudoInverse_times_vector(vel_des);
  OWD::JointPos q_offset = start_position - tc.q; //Should probably weight this so base joints move less
  OWD::JointPos dq_offset = OWD::Plugin::Nullspace_projection(q_offset);
  vel = dq_pose;// * 1.0 + dq_offset * 1.0;
  
  ROS_DEBUG_NAMED("omni_pose_error", "Omni pose error: (x,y,z,  qx,qy,qz,qw): (% .02f,% .02f,% .02f,  % .02f,% .02f,% .02f,% .02f)", 
                                          omni_hand_diff[0], omni_hand_diff[1], omni_hand_diff[2], 
                                          omni_quat_diff.x(), omni_quat_diff.y(), omni_quat_diff.z(), omni_quat_diff.w());
  
  ROS_DEBUG_NAMED("robot_pose_error", "Robot pose error: (x,y,z,  qx,qy,qz,qw,  wx,wy,wz,angle): (% .02f,% .02f,% .02f,  % .02f,% .02f,% .02f,% .02f,  % .02f,% .02f,% .02f,% .02f)", 
                                          error_hand[0], error_hand[1], error_hand[2], 
                                          error_quat.x(), error_quat.y(), error_quat.z(), error_quat.w(),
                                          error_axis.x(), error_axis.y(), error_axis.z(), error_angle);
  
  
  ROS_DEBUG_NAMED("dq_pose", "  dq_pose: [% .02f,% .02f,% .02f,% .02f,% .02f,% .02f,% .02f]", 
             dq_pose[0], dq_pose[1], dq_pose[2], dq_pose[3], dq_pose[4], dq_pose[5], dq_pose[6]);
  
  /*
  OWD::JointPos dir = tc.q;
  dir[0] = 0.0; dir[1] = 0.0; dir[2] = 0.1; dir[3] = 0.0; dir[4] = 0.0, dir[5] = 0.0; dir[6] = 0.0;
  R6 jac = OWD::Plugin::Jacobian_times_vector(dir);
  ROS_DEBUG_NAMED("jac_test", "  jac: [% .02f,% .02f,% .02f,% .02f,% .02f,% .02f]", 
                                    jac.v.x[0], jac.v.x[1], jac.v.x[2], jac.w.x[0], jac.w.x[1], jac.w.x[2]);
  */
  
  /*
  tc.q   = prev_pos;
  tc.qd  = zeros;
  tc.qdd = zeros;
  tc.t   = zeros;
  return;
  */
  
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
  
  // Check for joint limits. Project arm 0.5sec into the future
  OWD::JointPos q_future = prev_pos + vel*0.5;
  // ToDo: stop arm if q_future exceeds joint limits

  // Set variables
  pos = prev_pos + vel*dt; // Will this drift? prev_pos vs. tc.q?
  acc = zeros;

  // Set command values
  tc.q   = pos;
  tc.qd  = zeros;
  tc.qdd = zeros;
  tc.t   = zeros;
}




