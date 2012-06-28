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

#include "ServoTraj.hh"
#include <string.h>

namespace OWD {

  ServoTraj::ServoTraj(int dof, std::string id, double *start_pos,
		     double *lower_joint_limits,
		     double *upper_joint_limits)
  : Trajectory("ServoTraj", id),
    nDOF(dof),
    vlimit(2.5),
    accel(2.5),
    lasttime(time)
{
  stoptime.resize(nDOF,0.0);
  target_velocity.resize(nDOF,0.0);
  current_velocity.resize(nDOF,0.0);
  start_position.SetFromArray(nDOF,start_pos);
  current_position = start_position;
  end_position.resize(nDOF);
  duration=99999999999; // we might want to run a long time

  if (lower_joint_limits) {
    memcpy(lower_jlimit,lower_joint_limits,7*sizeof(double));
  } else {
    // WAM default
    lower_jlimit[0]= -2.60;
    lower_jlimit[1]= -1.96;
    lower_jlimit[2]= -2.73;
    lower_jlimit[3]= -0.86;
    lower_jlimit[4]= -4.79;
    lower_jlimit[5]= -1.56;
    lower_jlimit[6]= -2.99;
  }
  if (upper_joint_limits) {
    memcpy(upper_jlimit,upper_joint_limits,7*sizeof(double));
  } else {
    // WAM default
    upper_jlimit[0]=  2.60;
    upper_jlimit[1]=  1.96;
    upper_jlimit[2]=  2.73;
    upper_jlimit[3]=  3.13;
    upper_jlimit[4]=  1.30;
    upper_jlimit[5]=  1.56;
    upper_jlimit[6]=  2.99;
  }
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

void ServoTraj::evaluate_abs(Trajectory::TrajControl &tc, double t) {
  if (tc.q.size() < (unsigned int)nDOF) {
    runstate=DONE;
    return;
  }
  time = t;
  double dt = time - lasttime;  // need delta time for accel
  lasttime=time;
  bool active = false;
  for (unsigned int i = 0; i<(unsigned int)nDOF; ++i) {
    if (time < stoptime[i]) {
        // June 28, 2012
        // Moslem removed the joint limit check since it was problematic when we
        // are close to joint limits.
      // check for approaching joint limits
//      if ((target_velocity[i] > 0) &&
//	  (current_position[i] + jlimit_buffer > upper_jlimit[i])) {
//	target_velocity[i] = 0.0; // come to a stop before hitting limit
//      } else if ((target_velocity[i] < 0) &&
//	  (current_position[i] - jlimit_buffer < lower_jlimit[i])) {
//	target_velocity[i] = 0.0; // come to a stop before hitting limit
//      }

      // check for accel/decel
      if (target_velocity[i] > current_velocity[i]) {
	// still accelerating to target
	current_velocity[i] += accel * dt;
	tc.qdd[i]=accel;
	if (current_velocity[i] > target_velocity[i]) {
	  current_velocity[i] = target_velocity[i];
	}
      } else if (target_velocity[i] < current_velocity[i]) {
	// still decelerating
	current_velocity[i] -= accel * dt;
	tc.qdd[i]=-accel;
	if (current_velocity[i] < target_velocity[i]) {
	  current_velocity[i] = target_velocity[i];
	}
      } else {
	tc.qdd[i]=0.0;
      }

      // update position and velocity
      current_position[i] += current_velocity[i] * dt;
      tc.qd[i] = current_velocity[i];
      active = true;
    } else {
      tc.qd[i]=0.0;
      tc.qdd[i]=0.0;
    }
    tc.q[i] = current_position[i];
  }
  if (!active) {
    end_position = current_position;
    runstate = DONE;
  }

}

};// namespace OWD
