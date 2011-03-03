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

#include <iostream>
#include <fstream>
#include <unistd.h>          // usleep
#include <pthread.h>
#include <math.h>            // M_PI
#include <iomanip>

#ifndef OWDSIM

#ifdef ESD_CAN
#include <ntcan.h>
#else // ! ESD_CAN
#include <stdint.h>
#endif // ! ESD_CAN

#ifdef PEAK_CAN
#include <libpcan.h>
#endif  // PEAK_CAN

#endif // OWDSIM

#include <stdint.h>
#ifdef OWD_RT
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <native/intr.h>
#include <native/pipe.h>
#else // ! OWD_RT
#include <queue>
typedef unsigned long long RTIME; // usually defined in xenomai types.h
#endif // ! OWD_RT

#include <sys/mman.h>

#include "Group.hh"
#include "globals.h"

#ifdef CAN_RECORD
#include "DataRecorder.cc"
#endif // CAN_RECORD

#ifndef __CANBUS_H__
#define __CANBUS_H__

using namespace std;

#define NODE_MIN 1
#define NODE_MAX 15
// it looks like NUM_NODES is 1 too many, since +1 seems to be added elsewhere.
#define NUM_NODES (NODE_MAX - NODE_MIN + 2)

#define PUCK_IDLE 0 

class CANstats{
public:
  double cansend_time;
  double canread_sendtime;
  double canread_readtime;
  int    canread_badpackets;

  CANstats() : cansend_time(0.0f),
	       canread_sendtime(0.0f),
	       canread_readtime(0.0f),
	       canread_badpackets(0)
  {}

  void rosprint();
    
};

class CANmsg {
public:
  int32_t nodeid;
  int32_t property;
  int32_t value;

};


class CANbus{
private:
  pthread_t canthread;
  int bus_run;
  int32_t puck_state;
#ifdef PEAK_CAN
#define MAX_FILTERS (5)
  int32_t can_accept[MAX_FILTERS];
  int32_t mask[MAX_FILTERS];
#endif // PEAK_CAN

public:
  CANstats stats;
  bool BH280_installed;
  int32_t id;
  Group groups[NUM_GROUPS+1];
  int32_t* trq;
  double* pos;
  double* jpos;
  double* forcetorque_data;
  float* tactile_data;
  bool valid_forcetorque_data;
  bool valid_tactile_data;
  bool tactile_top10;
  Puck *pucks;
  int n_arm_pucks;
  bool simulation;
  char last_error[200];
  int32_t received_position_flags;
  int32_t received_state_flags;

#ifndef OWDSIM
#ifdef CAN_RECORD
typedef struct {
  int32_t secs;
  int32_t usecs;
  bool send;
  int32_t msgid;
  int32_t msglen;
  int32_t msgdata[8];
} canio_data;
DataRecorder<canio_data> candata;
#endif // CAN_RECORD

  int unread_packets;
  HANDLE handle;

#ifdef OWD_RT
  RT_INTR rt_can_intr;
#endif // OWD_RT

#endif // ! OWDSIM

#ifdef OWD_RT
  RT_PIPE handpipe;
  int handpipe_fd;
#else // ! OWD_RT
  std::queue<CANmsg> hand_command_queue;
  std::queue<CANmsg> hand_response_queue;
#endif // ! OWD_RT

  int load();
  int open();
  int check();
  
  int allow_message(int32_t id, int32_t mask);   
  int wake_puck(int32_t id);
  
  int status(int32_t* nodes);
  int parse(int32_t msgid, uint8_t* msg, int32_t msglen,
	    int32_t* nodeid, int32_t* property, 
	    int32_t* value, int32_t *value2=NULL);
  int compile(int32_t property, int32_t value, uint8_t *msg, int32_t *msglen);
  
  int set_property_rt(int32_t nid, int32_t property, int32_t value, bool check =false, int32_t usecs=200);

  /// Get a property from a puck (call only from control loop)
  /// \param nid Node number
  /// \param property Property number
  /// \param value Pointer to a place to store the value
  /// \param usecs Time to wait for response (defaults to 2000
  ///              if not specified)
  /// \param retries Number of times to resend the request after a timeout
  int request_property_rt(int32_t id, int32_t property);
  int get_property_rt(int32_t nid, int32_t property, int32_t* value, int32_t usecs=2000, int32_t retries=0);
  
  int read_rt(int32_t* msgid, uint8_t* msg, int32_t* msglen, int32_t usecs);
  int send_rt(int32_t  msgid, uint8_t* msg, int32_t  msglen, int32_t usecs);
  
  int clear();
  int32_t get_puck_state();
  int set_puck_state_rt();
  int set_puck_group_id(int32_t nid);

  int send_torques_rt();
  int read_positions_rt();

  int request_positions_rt(int32_t groupid);
  int request_puck_state_rt(int32_t nodeid);
  int request_hand_state_rt();
  int request_tactile_rt();
  int request_strain_rt();
  int request_forcetorque_rt();

  int process_positions_rt(int32_t msgid, uint8_t* msg, int32_t msglen);
  int process_arm_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen);
  int process_safety_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen);
  int process_hand_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen);
  int process_forcetorque_response_rt(int32_t msgid, uint8_t* msg, int32_t msglen);
  
  int read_forcetorque_rt();
  static int ft_combine(unsigned char msb, unsigned char lsb);

  int read_tactile_rt();
  int configure_tactile_sensors();

  void initPropertyDefs(int32_t firmwareVersion);
  RTIME time_now_ns();

  inline void rosprint_stats() {
    stats.rosprint();
  }


  /// Constructor
  /// \param bus_id numeric identifier of the CANbus device (/dev/can# for
  ///               ESD cards and /dev/pcan# for PEAK cards)
  /// \param number_of_arm_pucks 4 for arm only or 7 for arm + wrist
  ///                            (don't count hand)
  /// \param bh280 set to true if the CANbus hand (model BH280) is installed
  /// \param ft set to true if the CANbus force/torque sensor is installed
  /// \param tactile set to true if the hand has tactile sensors installed
  CANbus(int32_t bus_id,
	 int number_of_arm_pucks,
	 bool bh280=false,
	 bool ft=false,
	 bool tactile=false);

  ~CANbus();

  int init();
  int start();
  int stop();
  int run();
  void dump();
  void printpos();
  
  int send_torques(int32_t* mtrq);
  int read_positions(double* mpos);
  //  int read_torques(int32_t* mtrq);
  int send_positions(double* mpos);
  int send_AP(int32_t* apval);

  // mutex functions that work when compiled with or without Realtime kernel
#ifdef OWD_RT
  inline int mutex_init(RT_MUTEX *mutex) {
    return rt_mutex_create(mutex,NULL);
  }

  inline int mutex_lock(RT_MUTEX *mutex) {
    return rt_mutex_acquire(mutex,TM_INFINITE);
  }

  inline int mutex_trylock(RT_MUTEX *mutex) {
    return rt_mutex_acquire(mutex,TM_NONBLOCK);
  }

  inline int mutex_unlock(RT_MUTEX *mutex) {
    return rt_mutex_release(mutex);
  }
#else // ! OWD_RT
  inline int mutex_init(pthread_mutex_t *mutex) {
    return pthread_mutex_init(mutex,NULL);
  }

  inline int mutex_lock(pthread_mutex_t *mutex) {
    return pthread_mutex_lock(mutex);
  }

  inline int mutex_trylock(pthread_mutex_t *mutex) {
    return pthread_mutex_trylock(mutex);
  }

  inline int mutex_unlock(pthread_mutex_t *mutex) {
    return pthread_mutex_unlock(mutex);
  }
#endif // ! OWD_RT


  enum {HANDSTATE_UNINIT=0,
    HANDSTATE_DONE,
    HANDSTATE_MOVING };

private:
  int32_t hand_positions[4+1];
  int32_t hand_distal_positions[4+1];
  double hand_strain[4+1];
#ifdef OWD_RT
  RT_MUTEX hand_queue_mutex;
  RT_MUTEX hand_cmd_mutex;
#else // ! OWD_RT
  pthread_mutex_t hand_queue_mutex;
  pthread_mutex_t hand_cmd_mutex;
#endif // ! OWD_RT
  int32_t handstate;
  int first_moving_finger;

  int finger_reset(int32_t id);
  double finger_encoder_to_radians(int32_t enc);
  int32_t finger_radians_to_encoder(double radians);
  double spread_encoder_to_radians(int32_t enc);
  int32_t spread_radians_to_encoder(double radians);

public:
  int hand_get_property(int32_t id, int32_t prop, int32_t *val);
  int hand_set_property(int32_t id, int32_t prop, int32_t val);
  int hand_set_state_rt();
  int hand_activate(int32_t *nodes);
  int hand_reset();
  int hand_move(double p1, double p2, double p3, double p4);
  int hand_velocity(double v1, double v2, double v3, double v4);
  int hand_relax();
  int hand_get_positions(double &p1, double &p2, double &p3, double &p4);
  int hand_get_distal_positions(double &p1, double &p2, double &p3);
  int hand_get_strain(double &s1, double &s2, double &s3);
  int hand_get_state(int32_t &state);

  int ft_get_data(double *values);
  int ft_tare();

  int tactile_get_data(float *values);
  int tactile_set_hires(bool hires);

  int limits(double jointVel, double tipVel, double elbowVel);
  friend void* canbus_handler(void* argv);
};

#endif // __CANBUS_H__
