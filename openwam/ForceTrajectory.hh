/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openman is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openman is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <pthread.h>

#ifndef __TRAJECTORY_HH__
#define __TRAJECTORY_HH__

using namespace std;

class ForceTrajectory{
protected:
  pthread_mutex_t mutex;
  int runstate;
public:
  
  static const int STOP = 0;
  static const int RUN = 1;
  static const int DONE=2;
  static const int LOG  = 3;
  int id;
  
  ForceTrajectory():runstate(STOP),id(0) {
    pthread_mutex_init(&mutex, NULL);
  }

  virtual ~ForceTrajectory(){}
    
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

  virtual void evaluate(double y[], double t[], double dt)=0;

  virtual void reset() {}

  friend class WamDriver;
};

#endif

