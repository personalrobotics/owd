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

#include <iostream>
#include <iomanip>
#include <pthread.h>
#include <vector>
#include <map>
#include "globals.h"

#ifndef __MOTOR_HH__
#define __MOTOR_HH__

using namespace std;

class Motor{
public:
  pthread_mutex_t mutex;
  
  int ID;
  double q;
  double t;
  double puckI_per_Nm;       // Newton Meters -> puck torque units
  int offset;
  int rawpos;
  
private:
  void lock(){pthread_mutex_lock(&mutex);};
  void unlock(){pthread_mutex_unlock(&mutex);};

 public:
  
  static const int M1 = 1;
  static const int M2 = 2;
  static const int M3 = 3;
  static const int M4 = 4;
  static const int M5 = 5;
  static const int M6 = 6;
  static const int M7 = 7;
#if defined WRIST
  static const int Mn = 7;
#elif defined HEAD
  static const int Mn = 2;
#else
  static const int Mn = 4;
#endif
  
  static const float MIN_TRQ[];
  static const float MAX_TRQ[];
 
  Motor() : 
    ID(-1),
    q(0),
    t(0),
    puckI_per_Nm(1),
    offset(0)
  {
    pthread_mutex_init(&mutex, NULL);
  }
  
  int id(){return ID;}
  double IPNm(){return puckI_per_Nm;}
  
  double trq(){double T; lock(); T = t; unlock(); return T;}
  double pos(){double Q; lock(); Q = q; unlock(); return Q;}

  void trq(double T){ 
    lock(); 
    t=T;
    unlock();
  }
  void pos(double Q){ lock();  q = Q;  unlock(); }
  std::vector<std::pair<double,double> > cogging_torques;
  double cogging_comp_torque(double angle); // any angle
  double cogging_comp_torque(); // current angle
  static bool cogging_data_lessthan(const std::pair<double,double> &p1, const std::pair<double,double> &p2);
  bool load_cogging_data(const char *filename);
}; 

#endif
