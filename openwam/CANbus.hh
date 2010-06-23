
#include <iostream>
#include <fstream>
#include <unistd.h>          // usleep
#include <pthread.h>
#include <math.h>            // M_PI
#include <iomanip>

#ifndef OWDSIM
#include <ntcan.h>
#else
#include <stdint.h>
#endif

#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>

#include <sys/mman.h>

#include "Group.hh"
#include "globals.h"

#ifndef __CANBUS_H__
#define __CANBUS_H__

using namespace std;

#define NODE_MIN 1
#define NODE_MAX 15
#define NUM_NODES (NODE_MAX - NODE_MIN + 2)

class CANstats{
public:
  double cansend_time;
  double canread_sendtime;
  double canread_readtime;
  double cansetpuckstate_time;

  CANstats() : cansend_time(0.0f),
	       canread_sendtime(0.0f),
	       canread_readtime(0.0f),
	       cansetpuckstate_time(0.0f)
  {}

  void rosprint() const;
    
};

class CANbus{
private:
  pthread_mutex_t busmutex;
  pthread_t canthread;
  pthread_mutex_t runmutex;
  pthread_mutex_t statemutex;
  int bus_run;
  long puck_state;
  CANstats stats;

public:
  int id;
  Group groups[NUM_GROUPS+1];
  long* trq;
  double* pos;
  Puck *pucks;
  int npucks;
  bool simulation;

  pthread_mutex_t trqmutex;
  pthread_mutex_t posmutex;

#ifndef OWDSIM
  NTCAN_HANDLE handle;
#endif

  int load();
  int open();
  int check();
  
  int allow_message(int id, int mask);   
  int wake_puck(int id);
  
  int status(long* nodes);
  int parse(int32_t msgid, uint8_t* msg, int32_t msglen,
	    int32_t* nodeid, int32_t* property, long* value);
  int compile(int32_t property, long value, uint8_t *msg, int *msglen);
  
  int set_property(int32_t nid, int32_t property, long value, bool check);
  int get_property(int32_t nid, int32_t property, long* value);
  
  int read(int32_t* msgid, uint8_t* msg, int32_t* msglen, bool block);
  int send(int32_t  msgid, uint8_t* msg, int32_t  msglen, bool block);
  
  long get_puck_state();
  void set_puck_state();

  int send_torques();
  int read_positions();

  void initPropertyDefs(int firmwareVersion);

  inline void rosprint_stats() {
    stats.rosprint();
  }


  CANbus(int bus_id, int num_pucks);
  ~CANbus(){
    if(pucks!=NULL) delete pucks; 
    if(trq!=NULL) delete trq; 
    if(pos!=NULL) delete pos;
  }

  int init();
  int start();
  int stop();
  int run();
  void dump();
  void printpos();
  
  int send_torques(long* mtrq);
  int read_positions(double* mpos);
  int read_torques(long* mtrq);
  int send_positions(double* mpos);
  int send_AP(long* apval);
#ifdef BH280
  enum {HANDSTATE_UNINIT=0,
    HANDSTATE_DONE,
    HANDSTATE_MOVING };

  int hand_reset();
  int hand_move(double p1, double p2, double p3, double p4);
  int hand_velocity(double v1, double v2, double v3, double v4);
  int hand_relax();
  int hand_get_positions(double &p1, double &p2, double &p3, double &p4);
  int hand_get_state(int32_t &state);
#endif // BH280

  
  int limits(double jointVel, double tipVel, double elbowVel);
  friend void* canbus_handler(void* argv);
};

#endif
