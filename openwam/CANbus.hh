
#include <iostream>
#include <fstream>
#include <unistd.h>          // usleep
#include <pthread.h>
#include <math.h>            // M_PI
#include <iomanip>

#ifndef OWDSIM

#ifdef ESD_CAN
#include <ntcan.h>
#else
#include <stdint.h>
#endif

#ifdef PEAK_CAN
#include <libpcan.h>
#endif  // PEAK_CAN

#endif // OWDSIM

#include <stdint.h>
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>

#include <sys/mman.h>
#include <queue>

#include "Group.hh"
#include "globals.h"

#ifndef __CANBUS_H__
#define __CANBUS_H__

using namespace std;

#define NODE_MIN 1
#define NODE_MAX 15
// it looks like NUM_NODES is 1 too many, since +1 seems to be added elsewhere.
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

#ifdef BH280
class CANmsg {
public:
  int32_t nodeid;
  int32_t property;
  int32_t value;
};
#endif // BH280


class CANbus{
private:
  pthread_mutex_t busmutex;
  pthread_t canthread;
  pthread_mutex_t runmutex;
  pthread_mutex_t statemutex;
  int bus_run;
  int32_t puck_state;
  CANstats stats;
#ifdef PEAK_CAN
#define MAX_FILTERS (3)
  int32_t can_accept[MAX_FILTERS];
  int32_t mask[MAX_FILTERS];
#endif // PEAK_CAN

public:
  int32_t id;
  Group groups[NUM_GROUPS+1];
  int32_t* trq;
  double* pos;
  Puck *pucks;
  int npucks;
  bool simulation;

  pthread_mutex_t trqmutex;
  pthread_mutex_t posmutex;

#ifndef OWDSIM
  HANDLE handle;
#endif

  int load();
  int open();
  int check();
  
  int allow_message(int32_t id, int32_t mask);   
  int wake_puck(int32_t id);
  
  int status(int32_t* nodes);
  int parse(int32_t msgid, uint8_t* msg, int32_t msglen,
	    int32_t* nodeid, int32_t* property, int32_t* value);
  int compile(int32_t property, int32_t value, uint8_t *msg, int32_t *msglen);
  
  int set_property(int32_t nid, int32_t property, int32_t value, bool check);
  int get_property(int32_t nid, int32_t property, int32_t* value);
  
  int read(int32_t* msgid, uint8_t* msg, int32_t* msglen, bool block);
  int send(int32_t  msgid, uint8_t* msg, int32_t  msglen, bool block);
  
  int32_t get_puck_state();
  void set_puck_state();

  int send_torques();
  int read_positions();

  void initPropertyDefs(int firmwareVersion);

  inline void rosprint_stats() {
    stats.rosprint();
  }


  CANbus(int32_t bus_id, int num_pucks);
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
  
  int send_torques(int32_t* mtrq);
  int read_positions(double* mpos);
  int read_torques(int32_t* mtrq);
  int send_positions(double* mpos);
  int send_AP(int32_t* apval);
#ifdef BH280
  enum {HANDSTATE_UNINIT=0,
    HANDSTATE_DONE,
    HANDSTATE_MOVING };

private:
  std::queue<CANmsg> hand_read_queue;
  std::queue<CANmsg> hand_write_queue;
  int32_t hand_positions[4+1];
  pthread_mutex_t handmutex;

  int32_t hand_get_property(int32_t id, int32_t prop);
  void hand_set_property(int32_t id, int32_t prop, int32_t val);
  int finger_reset(int32_t id);
  double finger_encoder_to_radians(int32_t enc);
  int32_t finger_radians_to_encoder(double radians);
  double spread_encoder_to_radians(int32_t enc);
  int32_t spread_radians_to_encoder(double radians);

public:
  int hand_activate();
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
