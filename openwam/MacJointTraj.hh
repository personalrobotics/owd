/***********************************************************************

  Copyright 2007-2010 Carnegie Mellon University and Intel Corporation
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

#include <iostream>
#include <math.h>
#include "MacQuinticSegment.hh"
#include "TrajType.hh"
#include "Trajectory.hh"
#include <vector>

#ifndef __MACJOINTTRAJ_HH__
#define __MACJOINTTRAJ_HH__

class MacJointTraj : public Trajectory {
private:
  int DOF;
  JointPos max_joint_vel;
  JointPos max_joint_accel;
  std::vector<MacQuinticElement *>::iterator current_piece;
public:
  vector<MacQuinticElement *> macpieces;
  double traj_duration;

  static const int STOP = 0;
  static const int RUN  = 1;
  static const int DONE = 2;
  static const int LOG  = 3;


private:
  int rescale_to_slowest(int slowest_joint,
			 double max_end_time,
			 double accel_time,
			 const vector<double> &max_joint_vel,
			 const vector<double> &max_joint_accel);

  inline int sgn(double x) { return (x>0.0f)?1:(x<0.0f)?-1:0; }

  bool check_for_bend(TrajPoint &segstart, TrajPoint &p1, TrajPoint &p2);

public:
  MacJointTraj(TrajType &vtraj, 
		 const JointPos &max_joint_vel, 
		 const JointPos &max_joint_accel,
		 double max_jerk,
		 bool bWaitForStart,
		 bool bAutoBrakeOnStall,
		 int trajid);

  virtual ~MacJointTraj();
    
  void log(char *prefix);
    
  void run();
  void evaluate(double y[], double yd[], double ydd[], double dt);
  void get_path_values(double *path_vel, double *path_accel) const;
  void get_limits(double *max_path_vel, double *max_path_accel) const;
  void rebuild_from_current();
  void dump();
  void reset(double t);
};

#endif
