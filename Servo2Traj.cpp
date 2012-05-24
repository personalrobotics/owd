#include "HybridPlugin.h"
#include "Servo2Traj.h"

Servo2Traj *Servo2Traj::current_traj = NULL;
std::vector<double> Servo2Traj::Kp(7,0), Servo2Traj::Kd(7,0);
ros::Subscriber Servo2Traj::wamservo_sub;
ros::ServiceServer Servo2Traj::ss_SetServoGains;
double Servo2Traj::lower_jlimit[7], 
  Servo2Traj::upper_jlimit[7], 
  Servo2Traj::jlimit_buffer;

bool Servo2Traj::Register() {
  Kp.resize(7,64);
  Kd.resize(7,0);

  ros::NodeHandle n("~");
  wamservo_sub = 
    n.subscribe("wamservo2", 1, &Servo2Traj::wamservo_callback);
  ss_SetServoGains = n.advertiseService("SetServoGains", &Servo2Traj::SetServoGains);
  lower_jlimit[0]=  0.52;
  lower_jlimit[1]= -1.96;
  lower_jlimit[2]= -2.73;
  lower_jlimit[3]= -0.86;
  lower_jlimit[4]= -4.79;
  lower_jlimit[5]= -1.56;
  lower_jlimit[6]= -2.99;
  upper_jlimit[0]=  5.76;
  upper_jlimit[1]=  1.96;
  upper_jlimit[2]=  2.73;
  upper_jlimit[3]=  3.13;
  upper_jlimit[4]=  1.30;
  upper_jlimit[5]=  1.56;
  upper_jlimit[6]=  2.99;
  jlimit_buffer = 0.2;

  return true;
}

bool Servo2Traj::Shutdown() {
  wamservo_sub.shutdown();
  return true;
}

void Servo2Traj::wamservo_callback(const boost::shared_ptr<const owd_msgs::Servo> &servo) {
  if (servo->joint.size() != servo->velocity.size()) {
    ROS_ERROR("Servo joint array is not the same size as servo velocity array");
    return;
  }
  if (!current_traj) {
    try {
      current_traj = new Servo2Traj(OWD::Plugin::target_arm_position);
    } catch (const char *errstr) {
      ROS_ERROR("Failed to start servo motion: %s",errstr);
    }
    std::string reason;
    if (!OWD::Plugin::AddTrajectory(current_traj, reason)) {
      ROS_WARN("Could not start Servo2 trajectory: %s",
	       reason.c_str());
      return;
    }
  }
  for (unsigned int i=0; i<servo->joint.size(); ++i) {
    if ((servo->joint[i]-1) >= current_traj->target_velocity.size()) {
      ROS_WARN("Joint index %d exceeds DOF of %ld; servo velocity ignored for this joint",
	       servo->joint[i], current_traj->target_velocity.size());
      continue;
    }
    current_traj->target_velocity[servo->joint[i]-1] = servo->velocity[i];
    if (!current_traj->active[servo->joint[i]-1]) {
      current_traj->last_jpos[servo->joint[i]-1] = OWD::Plugin::target_arm_position[i];
      current_traj->last_vel_error[i]=0;
      current_traj->vel_filter[i]->reset();
      current_traj->active[servo->joint[i]-1] = true;
    }
    current_traj->stoptime[servo->joint[i]-1] = current_traj->time + 0.5;
  }
}

Servo2Traj::Servo2Traj(const std::vector<double> &start) :
  OWD::Trajectory("Servo2 Trajectory", OWD::Trajectory::random_id()),
  last_time(0),
  start_time(0)
{
  start_position=start;
  end_position=start;
  target_velocity.resize(start.size());
  last_jpos = OWD::Plugin::target_arm_position;
  last_vel_error.resize(start.size(),0);
  stoptime.resize(start.size());
  active.resize(start.size(), false);
  static_q = start;
  vel_filter.resize(start.size());
  for (unsigned int i=0; i<start.size(); ++i) {
    vel_filter[i] = new Butterworth<double>(2,20);
  }

  if (start.size() > 7) {
    ROS_ERROR("Cannot create a servo traj for more than 7 joints");
    throw "Cannot create a servo traj for more than 7 joints";
  }

  hybridplug->net_force.data.resize(start.size()*4);
}

Servo2Traj::~Servo2Traj() {
  current_traj = NULL;
  for (unsigned int i=0; i<vel_filter.size(); ++i) {
    delete vel_filter[i];
  }
}

void Servo2Traj::evaluate_abs(OWD::Trajectory::TrajControl &tc, double t) {
  double clock_time = hybridplug->time_now_usec() / 1.0e6;
  if (start_time == 0) {
    start_time = clock_time;
  }
  time = clock_time - start_time;
  double dt = time - last_time;
  last_time = time;
  for (unsigned int i = 0; i<(unsigned int)tc.q.size(); ++i) {
    if (active[i] && (time > stoptime[i])) {
      // remember position and switch to inactive
      static_q[i] = tc.q[i];
      active[i]=false;
      hybridplug->net_force.data[i*4+0] = hybridplug->net_force.data[i*4+1] = 
	hybridplug->net_force.data[i*4+2] = hybridplug->net_force.data[i*4+3] = 0;
      // if all joints have gone inactive, then end the trajectory
      bool anyactive = false;
      for (unsigned int j=0; j<active.size(); ++j) {
	if (active[j]) {
	  anyactive = true;
	  break;
	}
      }
      if (! anyactive) {
	end_position = tc.q;
	runstate=DONE;
	return;
      }
    }
    if (active[i]) {
      // check for approaching joint limits: if our current velocity
      // will cause us to hit the limit in the next 0.1 seconds, then
      // stop now.
      if ((tc.q[i] + target_velocity[i] * 0.1 > upper_jlimit[i]) ||
	  (tc.q[i] + target_velocity[i] * 0.1 < lower_jlimit[i])) {
	target_velocity[i] = 0.0; // come to a stop before hitting limit
      }

      // compute our velocity correction
      double actual_velocity(0); // initialize to zero for case dt=0
      if (dt > 0) {
	actual_velocity = vel_filter[i]->eval((tc.q[i] - last_jpos[i])/dt);
      }
      double vel_error = target_velocity[i] - actual_velocity;
      double vel_error_delta = vel_error - last_vel_error[i];
      double correction = Kp[i] * vel_error + Kd[i] * vel_error_delta;
      tc.qd[i]=target_velocity[i];
      tc.qdd[i]=correction;
      last_vel_error[i] = vel_error;
      last_jpos[i] = tc.q[i];
      hybridplug->net_force.data[i*4+0] = target_velocity[i];
      hybridplug->net_force.data[i*4+1] = actual_velocity;
      hybridplug->net_force.data[i*4+2] = vel_error;
      hybridplug->net_force.data[i*4+3] = correction;
    } else {
      // hold our previous position
      tc.q[i]=static_q[i];
    }
  }
  end_position = tc.q; // keep tracking our current position
}

bool Servo2Traj::SetServoGains(owd_msgs::SetGains::Request &req,
				      owd_msgs::SetGains::Response &res) {
  if ((req.joint < 1) || (req.joint > OWD::Plugin::arm_position.size())) {
    ROS_ERROR("SetServoGains ignored for joint %d (out of range)",
	      req.joint);
    return true;
  }
  Kp[req.joint-1]=req.gains.kp;
  Kd[req.joint-1]=req.gains.kd;
  return true;
}

