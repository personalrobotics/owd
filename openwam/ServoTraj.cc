#include "ServoTraj.hh"


ServoTraj::ServoTraj(int dof, int id_num, double *start_pos) 
  : nDOF(dof),
    vlimit(0.5),
    accel(2.5)
{
  id = id_num;
  stoptime.resize(nDOF,0.0);
  target_velocity.resize(nDOF,0.0);
  current_velocity.resize(nDOF,0.0);
  start_position.SetFromArray(nDOF,start_pos);
  current_position = start_position;
  end_position.resize(nDOF);
 // space to leave near joint limits to allow for deceleration 
  jlimit_buffer = 0.5*vlimit*vlimit/accel * 1.3;
}

ServoTraj::~ServoTraj() {
}
    
bool ServoTraj::SetVelocity(int j, float v, float duration) {
  if ((j > nDOF) || (j<1)) {
    return false;
  }
  if (v > vlimit) {
    v = vlimit;
  } else if (v < -vlimit) {
    v = -vlimit;
  }
  target_velocity[j-1] = v;
  stoptime[j-1] = time + duration;
  return true;
}
  
void ServoTraj::stop() {
  target_velocity.resize(nDOF,0.0);
  Trajectory::stop();
}

void ServoTraj::evaluate(double y[], double yd[], double ydd[], double dt) {
  const static double lower_jlimit[7]
    ={-2.60, -1.96, -2.73, -0.86, -4.79, -1.56, -2.99};
  const static double upper_jlimit[7]
    ={ 2.60,  1.96,  2.73,  3.13,  1.30,  1.56,  2.99};

  time += dt;
  bool active = false;
  for (unsigned int i = 0; i<nDOF; ++i) {
    if (time < stoptime[i]) {
      // check for approaching joint limits
      if ((target_velocity[i] > 0) &&
	  (current_position[i] + jlimit_buffer > upper_jlimit[i])) {
	target_velocity[i] = 0.0; // come to a stop before hitting limit
      } else if ((target_velocity[i] < 0) &&
	  (current_position[i] - jlimit_buffer < lower_jlimit[i])) {
	target_velocity[i] = 0.0; // come to a stop before hitting limit
      }

      // check for accel/decel
      if (target_velocity[i] > current_velocity[i]) {
	// still accelerating to target
	current_velocity[i] += accel * dt;
	if (current_velocity[i] > target_velocity[i]) {
	  current_velocity[i] = target_velocity[i];
	}
      } else if (target_velocity[i] < current_velocity[i]) {
	// still decelerating
	current_velocity[i] -= accel * dt;
	if (current_velocity[i] < target_velocity[i]) {
	  current_velocity[i] = target_velocity[i];
	}
      }

      // update position and velocity
      current_position[i] += current_velocity[i] * dt;
      yd[i+1] = current_velocity[i];
      active = true;
    } else {
      yd[i+1]=0.0;
    }
    y[i+1] = current_position[i];
    ydd[i+1] = 0.0;
  }
  if (!active) {
    end_position = current_position;
    runstate = DONE;
  }

}
