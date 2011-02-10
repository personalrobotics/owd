/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openwam is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openwam is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Modified 2007-2010 by:
      Mike Vande Weghe <vandeweg@cmu.edu>
      Robotics Institute
      Carnegie Mellon University
*/

#include <pthread.h>

// #include "Profile.hh"
#include "TrajType.hh"

#ifndef __TRAJECTORY_HH__
#define __TRAJECTORY_HH__

using namespace std;

class Trajectory{
protected:
  pthread_mutex_t mutex;
  JointPos start_position, end_position;
  int runstate;
  double time;
public:
  
  static const int STOP = 0;
  static const int RUN = 1;
  static const int DONE=2;
  static const int LOG  = 3;
  int id;
  bool HoldOnStall, WaitForStart, HoldOnForceInput;
  double forcetorque[6];
  bool valid_ft;
  
  Trajectory():runstate(STOP),time(0.0),id(0),
	       HoldOnStall(false),WaitForStart(false),
	       HoldOnForceInput(false),valid_ft(false) {
    pthread_mutex_init(&mutex, NULL);
  }

  virtual ~Trajectory(){}
    
  virtual void lock(){pthread_mutex_lock(&mutex);}
  virtual void unlock(){pthread_mutex_unlock(&mutex);}
    
  virtual void run() {
    runstate=RUN;
  }

  virtual void stop() {
    runstate=STOP;
  }

  virtual int  state() {
    return runstate;
  }

  virtual void evaluate(double y[], double yd[], double ydd[], double dt)=0;

  inline virtual const JointPos &endPosition() const {return end_position;}
  
  virtual bool log(const char* fname);
  
  virtual void reset(double t) {time=t;}

  inline virtual double curtime() const {return time;}

  virtual void update_torques(double t[]) {}

  virtual void ForceFeedback(double ft[]);

  friend class WamDriver;
};

#endif

