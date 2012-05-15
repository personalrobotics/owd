#include "Servo2Traj.h"

Servo2Traj *Servo2Traj::current_traj = NULL;
std::vector<double> Servo2Traj::Kp(7,0), Kd(7,0);

bool Servo2Traj::Register() {
  Kp.resize(7,0);
  Kd.resize(7,0);

  ros::NodeHandle n("~");
  wamservo_sub = 
    n.subscribe("wamservo2", 1, &Servo2Traj::wamservo_callback);
  
  return true;
}

bool Servo2Traj::Shutdown() {
  wamservo_sub.shutdown();
  return true;
}

void Servo2Traj::wamservo_callback(const boost::shared_ptr<const pr_msgs::Servo> &servo) {
  if (servo->joint.size() != servo->velocity.size()) {
    ROS_ERROR("Servo joint array is not the same size as servo velocity array");
    return;
  }
  if (!current_traj) {
    current_traj = new Servo2Traj(OWD::Plugin::target_arm_position);
    std::string reason;
    if (!OWD::Plugin::AddTrajectory(current_traj, reason)) {
      ROS_WARN("Could not start Servo2 trajectory: %s",
	       reason.c_str());
      return;
    }
  }
  for (unsigned int i=0; i<servo->joint.size(); ++i) {
    current_traj->target_velocity[servo->joint[i]-1] = servo->velocity[i];
    if (!current_traj->active[servo->joint[i]-1]) {
      current_traj->last_jpos[servo->joint[i]-1] = OWD::Plugin::target_arm_position[i];
      current_traj->last_vel_error[i]=0;
      current_traj->active[servo->joint[i]-1] = true;
    }
    current_traj->stoptime[servo->joint[i]-1] = current_traj->time + 0.5;
  }
}

Servo2Traj::Servo2Traj(const std::vector<double> &start) :
  OWD::Trajectory("Servo2 Trajectory", OWD::Trajectory::random_id())
{
  start_position=start;
  end_position=start;
  active.resize(start.size(), false);
  last_jpos = OWD::Plugin::target_arm_position;
  last_vel_error.resize(start.size(),0);
  vel_filter.resize(start.size());
  for (unsigned int i=0; i<start.size(); ++i) {
    vel_filter[i] = new Butterworth<double>(2,50);
  }

  static_q = start;
  lower_jlimit[0]= -2.60;
  lower_jlimit[1]= -1.96;
  lower_jlimit[2]= -2.73;
  lower_jlimit[3]= -0.86;
  lower_jlimit[4]= -4.79;
  lower_jlimit[5]= -1.56;
  lower_jlimit[6]= -2.99;
  upper_jlimit[0]=  2.60;
  upper_jlimit[1]=  1.96;
  upper_jlimit[2]=  2.73;
  upper_jlimit[3]=  3.13;
  upper_jlimit[4]=  1.30;
  upper_jlimit[5]=  1.56;
  upper_jlimit[6]=  2.99;
  jlimit_buffer = 0.2;
}

Servo2Traj::~Servo2Traj() {
  current_traj = NULL;
  for (unsigned int i=0; i<vel_filter.size(); ++i) {
    delete vel_filter[i];
  }
}

void Servo2Traj::evaluate_abs(OWD::Trajectory::TrajControl &tc, double t) {
  time = t;
  double dt = time - last_time;
  last_time = time;
  for (unsigned int i = 0; i<(unsigned int)tc.q.size(); ++i) {
    if (active[i] && (time > stoptime[i])) {
      // remember position and switch to inactive
      static_q[i] = tc.q[i];
      active[i]=false;
    }
    if (active[i]) {
      // check for approaching joint limits
      if ((target_velocity[i] > 0) &&
	  (tc.q[i] + jlimit_buffer > upper_jlimit[i])) {
	target_velocity[i] = 0.0; // come to a stop before hitting limit
      } else if ((target_velocity[i] < 0) &&
		 (tc.q[i] - jlimit_buffer < lower_jlimit[i])) {
	target_velocity[i] = 0.0; // come to a stop before hitting limit
      }

      // compute our velocity correction
      double actual_velocity = vel_filter[i]->eval((tc.q[i] - last_jpos[i])/dt);
      double vel_error = target_velocity[i] - actual_velocity;
      double vel_error_delta = vel_error - last_vel_error[i];
      double correction = Kp[i] * vel_error + Kd[i] * vel_error_delta;
      tc.qd[i]=target_velocity[i];
      tc.qdd[i]=correction;
      last_vel_error[i] = vel_error;
      last_jpos[i] = tc.q[i];
    } else {
      // hold our previous position
      tc.q[i]=static_q[i];
    }
  }
}
