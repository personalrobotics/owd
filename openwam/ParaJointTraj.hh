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
#include "ParabolicSegment.hh"
#include "TrajType.hh"
#include "Trajectory.hh"
#include <vector>

#ifndef __PARAJOINTTRAJ_HH__
#define __PARAJOINTTRAJ_HH__

class ParaJointTraj : public Trajectory {
private:
    int DOF;
    vector<double> max_joint_vel;
    vector<double> max_joint_accel;
    int rescale_to_slowest(int slowest_joint,double max_end_time,double accel_time, const vector<double> &max_joint_vel, const vector<double> &max_joint_accel);
    inline int sgn(double x) { return (x>0.0f)?1:(x<0.0f)?-1:0; }
    bool check_for_bend(vector<ParabolicSegment> *ps,
                               TrajPoint &p1, TrajPoint &p2);
    double restart_time;
    vector<ParabolicSegment>::iterator *current_segment;
    
public:
    
    vector<ParabolicSegment> *parsegs;
    vector <ParabolicSegment> restart_parsegs;
    bool restart;

    double traj_duration;
    
    ParaJointTraj(TrajType &vtraj, 
                  const vector<double> &max_joint_vel, 
                  const vector<double> &max_joint_accel,
                  bool bWaitForStart,
                  bool bAutoBrakeOnStall,
                  int trajid);
    virtual ~ParaJointTraj();
    
    void lock(){pthread_mutex_lock(&mutex);}
    void unlock(){pthread_mutex_unlock(&mutex);}
    bool log(const char *prefix);
    
    void run();
    void stop();
    int  state();
    void evaluate(double y[], double yd[], double ydd[], double dt);
    void rebuild_from_current();
  void reset(double t);
};

#endif
