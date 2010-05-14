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

#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <inttypes.h>        // uint64_t
#include <unistd.h>          // usleep
#include "globals.h"

#include <sys/mman.h>

#ifndef __CONTROL_LOOP_H__
#define __CONTROL_LOOP_H__

using namespace std;

enum CONTROLLOOP_STATE{CONTROLLOOP_STOP, CONTROLLOOP_RUN};

#ifdef OWDSIM
typedef unsigned long long RTIME; // usually defined in xenomai types.h
#else
#include <native/types.h>
#endif

class ControlLoop{
private:
  pthread_t ctrlthread;
  pthread_mutex_t mutex;
  CONTROLLOOP_STATE cls;

public:
  //static const double PERIOD = 0.01;
  //static const double PERIOD = 0.002;
  static const double PERIOD = 0.002;

  void* (*ctrl_fnc)(void*);
  void* ctrl_argv;

  void lock(){pthread_mutex_lock(&mutex);}
  void unlock(){pthread_mutex_unlock(&mutex);}

  ControlLoop();
  
  int start(void* ctrl_fnc(void*), void* argv);
  int stop();
  int state();
  void wait();
  static RTIME get_time_ns();

};

#endif

