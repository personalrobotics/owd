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

#include "ControlLoop.hh"
#include <ros/ros.h>
#include <sys/time.h>
#include <time.h>


static void* control_handler(void* argv){
  ControlLoop* ctrl_loop;

  ctrl_loop = (ControlLoop*)argv;

  ctrl_loop->ctrl_fnc( ctrl_loop->ctrl_argv );

  return argv;
}

ControlLoop::ControlLoop(int tasknum) : task_number(tasknum){
  pthread_mutex_init(&mutex, NULL);
}

int ControlLoop::start(void* (*fnc)(void*), void* argv){

  ctrl_fnc = fnc;
  ctrl_argv = argv;

  lock();
  cls = CONTROLLOOP_RUN;
  unlock();

  if(pthread_create(&ctrlthread, NULL, control_handler, this)){
    ROS_FATAL("ControlLoop::start: pthread_create failed.");
    lock();
    cls = CONTROLLOOP_STOP;
    unlock();
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int ControlLoop::stop(){
  lock();
  cls = CONTROLLOOP_STOP;
  unlock();

  if(pthread_join(ctrlthread, NULL) != 0){
    ROS_FATAL("ControlLoop::stop: pthread_join failed.");
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int ControlLoop::state(){
  CONTROLLOOP_STATE s;
  lock();
  s = cls;
  unlock();
  return s;
}

void ControlLoop::wait() {
  usleep(700);
}

RTIME ControlLoop::get_time_ns() {
  timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec * 1e9 + t.tv_usec * 1e3;
}
