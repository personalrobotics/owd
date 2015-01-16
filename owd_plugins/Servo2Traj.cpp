#include "HybridPlugin.h"
#include "Servo2Traj.h"

Servo2Traj *Servo2Traj::current_traj = NULL;
std::vector<double> Servo2Traj::Kp(7,32), Servo2Traj::Kd(7,0), Servo2Traj::Ki(7,0);
ros::Subscriber Servo2Traj::wamservo_sub;
ros::ServiceServer Servo2Traj::ss_SetServoGains;

bool Servo2Traj::Register() {
  ros::NodeHandle n("~");
  wamservo_sub = 
    n.subscribe("wamservo2", 1, &Servo2Traj::wamservo_callback);
  ss_SetServoGains = n.advertiseService("SetServoGains", &Servo2Traj::SetServoGains);

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
      current_traj = new Servo2Traj(hybridplug->target_arm_position);
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
    if (servo->velocity[i] == 0) {
      // better to keep holding position than to try to servo to zero vel.
      // setting the stop time to now will cause it to go back to
      // inactive and hold position.
      current_traj->stoptime[servo->joint[i]-1] = current_traj->time;
      continue;
    }
    if (servo->velocity[i] > OWD::Plugin::joint_vel[servo->joint[i]-1]) {
      // clamp to upper vel limit
      current_traj->target_velocity[servo->joint[i]-1] 
	= OWD::Plugin::joint_vel[servo->joint[i]-1];
    } else if (servo->velocity[i] < -OWD::Plugin::joint_vel[servo->joint[i]-1]) {
      // clamp to lower vel limit
      current_traj->target_velocity[servo->joint[i]-1] 
	= -OWD::Plugin::joint_vel[servo->joint[i]-1];
    } else {
      current_traj->target_velocity[servo->joint[i]-1] = servo->velocity[i];
    }
    if (!current_traj->active[servo->joint[i]-1]) {
      current_traj->last_vel_error[i]=0;
      current_traj->total_vel_error[i]=0;
      current_traj->active[servo->joint[i]-1] = true;
    }
    current_traj->stoptime[servo->joint[i]-1] = current_traj->time + 0.5;
  }
}

Servo2Traj::Servo2Traj(const std::vector<double> &start) :
  OWD::Trajectory("Servo2 Trajectory", OWD::Trajectory::random_id()),
  start_time(0)
{
  start_position=start;
  end_position=start;
  target_velocity.resize(start.size());
  last_vel_error.resize(start.size(),0);
  total_vel_error.resize(start.size(),0);
  stoptime.resize(start.size());
  active.resize(start.size(), false);
  static_q = start;

  if (start.size() > 7) {
    ROS_ERROR("Cannot create a servo traj for more than 7 joints");
    throw "Cannot create a servo traj for more than 7 joints";
  }

  hybridplug->net_force.data.resize(start.size()*5);
}

Servo2Traj::~Servo2Traj() {
  current_traj = NULL;

  // stop publishing
  hybridplug->net_force.data.resize(0);
}

void Servo2Traj::evaluate_abs(OWD::Trajectory::TrajControl &tc, double t) {

  double clock_time = hybridplug->time_now_usec() / 1.0e6;
  if (start_time == 0) {
    start_time = clock_time;
  }
  time = clock_time - start_time;
  for (unsigned int i = 0; i<(unsigned int)tc.q.size(); ++i) {
    if (active[i] && (time > stoptime[i])) {
      // remember position and switch to inactive
      static_q[i] = tc.q[i];
      active[i]=false;
      hybridplug->net_force.data[i*5+0]
	= hybridplug->net_force.data[i*5+1]
	= hybridplug->net_force.data[i*5+2]
	= hybridplug->net_force.data[i*5+3]
	= hybridplug->net_force.data[i*5+4]
	= 0;
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
      if ((tc.q[i] + target_velocity[i] * 0.1 > hybridplug->upper_jlimit[i]) ||
	  (tc.q[i] + target_velocity[i] * 0.1 < hybridplug->lower_jlimit[i])) {
	target_velocity[i] = 0.0; // come to a stop before hitting limit
      }

      double vel_error = target_velocity[i] - hybridplug->arm_velocity[i];
      double vel_error_delta = vel_error - last_vel_error[i];
      total_vel_error[i] += vel_error;
      double correction = 
	Kp[i] * vel_error +
	Kd[i] * vel_error_delta +
	Ki[i] * total_vel_error[i];
      tc.qd[i]=target_velocity[i];
      tc.qdd[i]=correction;
      last_vel_error[i] = vel_error;
      hybridplug->net_force.data[i*5+0] = target_velocity[i];
      hybridplug->net_force.data[i*5+1] = OWD::Plugin::arm_velocity[i];
      hybridplug->net_force.data[i*5+2] = vel_error;
      hybridplug->net_force.data[i*5+3] = correction;
      hybridplug->net_force.data[i*5+4] = tc.q[i];
    } else {
      // hold our previous position
      tc.q[i]=static_q[i];
    }
  }
  end_position = tc.q; // keep tracking our current position
}

bool Servo2Traj::SetServoGains(owd_msgs::SetGains::Request &req,
				      owd_msgs::SetGains::Response &res) {
  if ((req.joint < 1) || (req.joint > hybridplug->arm_position.size())) {
    ROS_ERROR("SetServoGains ignored for joint %d (out of range)",
	      req.joint);
    return true;
  }
  Kp[req.joint-1]=req.gains.kp;
  Kd[req.joint-1]=req.gains.kd;
  Ki[req.joint-1]=req.gains.ki;
  return true;
}

