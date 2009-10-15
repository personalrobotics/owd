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
#include <native/task.h>
#include <native/timer.h>
#include <ros/ros.h>

static void* control_handler(void* argv){
  RT_TASK task;
  RTIME samper, now;
  ControlLoop* ctrl_loop;

  ctrl_loop = (ControlLoop*)argv;

  //RTAI:  rt_allow_nonroot_hrt();

  int retvalue = rt_task_create(&task, "CANBUS", 0, 99, T_CPU(1));
  if(retvalue){
    ROS_FATAL("control_handler: rt_task_create failed");
    return NULL;
  }
  if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1){
    ROS_FATAL("control_handler: mlockall failed");
    ROS_ERROR_COND(
		   rt_task_delete(&task), "Problem deleting RT task");
    return NULL;
  }

#ifdef NOT_NECESSARY // causes an error
    retvalue = rt_task_set_mode(0, T_PRIMARY, NULL);
  if (retvalue) {
    ROS_FATAL("Unable to run task in Primary mode: return=%d",retvalue);
    ROS_ERROR_COND(
		   rt_task_delete(&task),"Problem deleting RT task");
    return NULL;
  }
#endif   
  // 
  retvalue = rt_timer_set_mode(TM_ONESHOT);
  ROS_ERROR_COND(retvalue,"Unable to set timer in oneshot mode");

  samper = rt_timer_ns2ticks( (RTIME)(ControlLoop::PERIOD*1000000000.0) );

  // start_rt_timer(samper);
  //  now = rt_timer_read();

  if(rt_task_set_periodic(&task, TM_NOW, samper) != 0){
    ROS_FATAL("control_handler: rt_task_make_periodic failed.");
    ROS_ERROR_COND(
		   rt_task_delete(&task),"Problem deleting RT task");
    return NULL;
  }

  ctrl_loop->ctrl_fnc( ctrl_loop->ctrl_argv );

  ROS_ERROR_COND(
		 rt_task_delete(&task),"Problem deleting RT task");

  return argv;
}

ControlLoop::ControlLoop(){
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

void ControlLoop::wait(){rt_task_wait_period(NULL);}
