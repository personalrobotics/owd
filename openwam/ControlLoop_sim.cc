/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Labs Pittsburgh *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

#include "ControlLoop.hh"
#include <ros/ros.h>
#include <sys/time.h>
#include <time.h>


ControlLoop::ControlLoop(int tasknum, void (*fnc)(void*), void* argv) :
  task_number(tasknum), ctrl_fnc(fnc), ctrl_argv(argv) {

  pthread_mutex_init(&mutex, NULL);

  snprintf(taskname,20,"OWDTASK%02d",task_number);
}

void *ControlLoop::start_thread(void *data) {
  ControlLoop *cl = (ControlLoop*)data;
  (*cl->ctrl_fnc)(cl->ctrl_argv);
  return NULL;
}


int ControlLoop::start() {

  if (cls == CONTROLLOOP_RUN) {
    return OW_SUCCESS;
  }

  lock();
  cls = CONTROLLOOP_RUN;
  unlock();

  if(pthread_create(&ctrlthread, NULL, &ControlLoop::start_thread, this)) {
    ROS_FATAL("ControlLoop::start: pthread_create failed.");
    lock();
    cls = CONTROLLOOP_STOP;
    unlock();
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int ControlLoop::stop(){
  if (cls == CONTROLLOOP_STOP) {
    ROS_DEBUG("ControlLoop: thread %s was already stopped", taskname);
    return OW_SUCCESS;
  }

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

ControlLoop::~ControlLoop() {
  stop();
}
