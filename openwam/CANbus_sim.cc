/***********************************************************************

  Copyright 2010 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

  This file is part of owd.

  owd is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  owd is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***********************************************************************/


#include "CANbus.hh"
#include "CANdefs.hh"
#include <ros/ros.h>
#include <time.h>
#include <sys/time.h>

#define PUCK_IDLE 0 

CANbus::CANbus(int32_t bus_id, int num_pucks) : 
  puck_state(2),id(bus_id),
  trq(NULL),pos(NULL),
  pucks(NULL),npucks(num_pucks),simulation(true)
{
  //  pthread_mutex_init(&trqmutex, NULL);
  //  pthread_mutex_init(&posmutex, NULL);
  //  pthread_mutex_init(&runmutex, NULL);
  //  pthread_mutex_init(&statemutex, NULL);

  pucks = new Puck[npucks+1];
  trq = new int32_t[npucks+1];
  pos = new double[npucks+1];
  for(int p=1; p<=npucks; p++){
    pos[p] = 0.0;
    trq[p] = 0;
    pucks[p].ID = p;
    pucks[p].motor_id = p;
    if (p<5) { // 4-DOF
      pucks[p].group_id = 1;
      pucks[p].group_order = p;
    } else if (p<8) { // WRIST
      pucks[p].group_id = 2;
      pucks[p].group_order = p-4;
    } else { // HAND
      pucks[p].group_id = 3;
      pucks[p].group_order = p-7;
    }
    pucks[p].cpr = 4096;
  }

  for(int p=1; p<=npucks; p++){
    if(groups[ pucks[p].group() ].insert(&pucks[p]) == OW_FAILURE){
      ROS_ERROR("CANbus::load: insert failed.");
      throw OW_FAILURE;
    }
  }
}

int CANbus::init(){
  if(open() == OW_FAILURE){
    ROS_ERROR("CANbus::init: open failed.");
    return OW_FAILURE;
  }

  if(check() == OW_FAILURE){
      //ROS_ERROR("CANbus::init: check failed.");
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::open(){
  return OW_SUCCESS;
}

int CANbus::check(){
  return OW_SUCCESS;
}

void CANbus::dump(){
}
   
int CANbus::allow_message(int32_t id, int32_t mask){
  return OW_SUCCESS;
}
 
int CANbus::wake_puck(int32_t p){
    return OW_SUCCESS;
}

int CANbus::clear() {
  return OW_SUCCESS;
}
  
int CANbus::status(int32_t* nodes){
  return OW_SUCCESS;
}

int CANbus::set_property_rt(int32_t nid, int32_t property, int32_t value,bool check, int32_t usecs){
  return OW_SUCCESS;
}

int CANbus::get_property_rt(int32_t nid, int32_t property, int32_t* value, int32_t usecs){
  return OW_SUCCESS;
}

int CANbus::send_torques(int32_t* torques){
  return OW_SUCCESS;
}

int CANbus::send_torques_rt(){
  return OW_SUCCESS;
}

#ifdef NDEF
/*
int CANbus::read_torques(int32_t* mtrq){
  return OW_SUCCESS;
}
*/
#endif

int CANbus::read_positions(double* positions){
  for(int p=1; p<=npucks; p++) {
    positions[p] = 0;
  }
  return OW_SUCCESS;
}

int CANbus::read_positions_rt(){
  return OW_SUCCESS;
}

int CANbus::send_positions(double* mpos){
  return OW_SUCCESS;
}

int CANbus::send_AP(int32_t* apval){
  return OW_SUCCESS;
}

int CANbus::parse(int32_t msgid, uint8_t* msg, int32_t msglen,
		  int32_t* nodeid, int32_t* property, int32_t* value){
  return OW_SUCCESS;
}

int CANbus::compile(int32_t property, int32_t value, 
		    uint8_t *msg, int32_t *msglen){
  return OW_SUCCESS;
}

int CANbus::read_rt(int32_t* msgid, uint8_t* msg, int32_t* msglen, int32_t usecs){
    //    ROS_ERROR("CANbus:: READ");
  return OW_SUCCESS;
}

int CANbus::send_rt(int32_t msgid, uint8_t* msg, int32_t msglen, int32_t usecs){
  return OW_SUCCESS;
}

// set safety limits (does it really work?)
// it always sets the same property!!
// 4.2 rad/s corresponds to 240deg/sec
int CANbus::limits(double jointVel, double tipVel, double elbowVel){
  return OW_SUCCESS;
}

int CANbus::run(){
  int r;
  r = bus_run;
  return r;
}


void CANbus::printpos(){
  for(int p=1; p<=npucks; p++)
    ROS_DEBUG("%f",pos[p]);
}


int32_t CANbus::get_puck_state() {
  int32_t pstate;
  pstate = puck_state;
  return pstate;
}

int CANbus::set_puck_state_rt() {
  puck_state = 2;
  return OW_SUCCESS;
}

void CANstats::rosprint()  {
  //  ROS_DEBUG_NAMED("times","CANbus::send %2.1fms per group (2 groups)",
  //		  cansend_time);
  //  ROS_DEBUG_NAMED("times","CANbus::read: send=%2.1fms, read=%2.1fms",
  //		  canread_sendtime, canread_readtime);
  ROS_DEBUG_NAMED("times","CANbus::set_puck_state: %2.2fms",
  		  cansetpuckstate_time);
}
  
 RTIME CANbus::time_now_ns() {
   struct timeval tv;
   gettimeofday(&tv,NULL);
   return (tv.tv_sec * 1e6 + tv.tv_usec);
 }


void CANbus::initPropertyDefs(int32_t firmwareVersion){
  if(firmwareVersion < 40){
    VERS = 0;
    ROLE = 1;
    SN   = 2;
    ID   = 3;
    ERROR= 4;
    STAT = 5;
    ADDR = 6;
    VALUE= 7;
    MODE = 8;
    D    = 9;
    T=TORQ=10;
    P    = 11;
    V    = 12;
    E    = 13;
    B=FET0=14;
    MD   = 15;
    MT   = 16;
    MV   = 17;
    MCV  = 18;
    MOV  = 19;
    MOFST= 20;
    IOFST= 21;
    PTEMP= 22;
    UPSECS=23;
    OD   = 24;
    MDS  = 25;
    AP   = 26;
    AP2  = 27;
    MECH = 28;
    MECH2= 29;
    CTS  = 30;
    CTS2 = 31;
    DP   = 32;
    DP2  = 33;
    OT   = 34;
    OT2  = 35;
    CT   = 36;
    CT2  = 37;
    BAUD = 38;
    TEMP = 39;
    OTEMP= 40;
    _LOCK= 41;
    DIG0 = 42;
    DIG1 = 43;
    ANA0 = 44;
    ANA1 = 45;
    THERM= 46;
    VBUS = 47;
    IMOTOR=48;
    VLOGIC=49;
    ILOGIC=50;
    GRPA = 51;
    GRPB = 52;
    GRPC = 53;
    PIDX = 54;
    ZERO = 55;
    SG   = 56;
    HSG  = 57;
    LSG  = 58;
    _DS  = 59;
    IVEL = 60;
    IOFF = 61;
    MPE  = 62;
    EN   = 63;
    TSTOP= 64;
    KP   = 65;
    KD   = 66;
    KI   = 67;
    SAMPLE=68;
    ACCEL= 69;
    TENSION=FET1=70;
    UNITS= 71;
    RATIO= 72;
    LOG  = 73;
    DUMP = 74;
    LOG1 = 75;
    LOG2 = 76;
    LOG3 = 77;
    LOG4 = 78;
    GAIN1= 79;
    GAIN2= 80;
    GAIN3= 81;
    OFFSET1=82;
    OFFSET2=83;
    OFFSET3=84;
    PEN  = 85;
    SAFE = 86;
    SAVE = 87;
    LOAD = 88;
    DEF  = 89;
    VL1  = 90;
    VL2  = 91;
    TL1  = 92;
    TL2  = 93;
    VOLTL1=94;
    VOLTL2=95;
    VOLTH1=96;
    VOLTH2=97;
    MAXPWR=98;
    PWR  = 99;
    IFAULT=100;
    IKP  = 101;
    IKI  = 102;
    IKCOR= 103;
    VNOM = 104;
    TENST= 105;
    TENSO= 106;
    JIDX = 107;
    IPNM = 108;
    
    PROP_END = 109;
    

  } else { // (firmwareVersion >= 40)
    
    /* Common */
    VERS = 0;
    ROLE = 1; /* P=PRODUCT, R=ROLE: XXXX PPPP XXXX RRRR */
    SN   = 2;
    ID   = 3;
    ERROR= 4;
    STAT = 5;
    ADDR = 6;
    VALUE= 7;
    MODE = 8;
    TEMP = 9;
    PTEMP= 10;
    OTEMP= 11;
    BAUD = 12;
    _LOCK= 13;
    DIG0 = 14;
    DIG1 = 15;
    TENSION=FET0= 16;
    BRAKE=  FET1= 17;
    ANA0 = 18;
    ANA1 = 19;
    THERM= 20;
    VBUS = 21;
    IMOTOR=22;
    VLOGIC=23;
    ILOGIC=24;
    SG   = 25;
    GRPA = 26;
    GRPB = 27;
    GRPC = 28;
    CMD  = 29; /* For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T */
    SAVE = 30;
    LOAD = 31;
    DEF  = 32;
    FIND = 33;
    X0   = 34;
    X1   = 35;
    X2   = 36;
    X3   = 37;
    X4   = 38;
    X5   = 39;
    X6   = 40;
    X7   = 41;
    
    /* Motor pucks */
    T=TORQ=42;
    MT   = 43;
    V    = 44;
    MV   = 45;
    MCV  = 46;
    MOV  = 47;
    AP=P = 48; /* 32-Bit Present Position */
    P2   = 49;
    DP   = 50; /* 32-Bit Default Position */
    DP2  = 51;
    E    = 52; /* 32-Bit Endpoint */
    E2   = 53;
    OT   = 54; /* 32-Bit Open Target */
    OT2  = 55;
    CT   = 56; /* 32-Bit Close Target */
    CT2  = 57;
    M    = 58; /* 32-Bit Move command for CAN*/
    M2   = 59;
    _DS  = 60;
    MOFST= 61;
    IOFST= 62;
    UPSECS=63;
    OD   = 64;
    MDS  = 65;
    MECH = 66; /* 32-Bit */
    MECH2= 67;
    CTS  = 68; /* 32-Bit */
    CTS2 = 69;
    PIDX = 70;
    HSG  = 71;
    LSG  = 72;
    IVEL = 73;
    IOFF = 74; /* 32-Bit */
    IOFF2= 75;
    MPE  = 76;
    HOLD = 77;
    TSTOP= 78;
    KP   = 79;
    KD   = 80;
    KI   = 81;
    ACCEL= 82;
    TENST= 83;
    TENSO= 84;
    JIDX = 85;
    IPNM = 86;
    HALLS= 87;
    HALLH= 88; /* 32-Bit */
    HALLH2=89;
    POLES= 90;
    IKP  = 91;
    IKI  = 92;
    IKCOR= 93;
    EN   = 94;
    EN2  = 95;
    JP   = 96;
    JP2  = 97;
    JOFST= 98;
    JOFST2=99;
    TIE  = 100;
    ECMAX= 101;
    ECMIN= 102;
    LFLAGS=103;
    LCTC = 104;
    LCVC = 105;
    TACT = 106;
    TACTID=107;
    
    /* Safety puck */
    ZERO = 43;
    PEN  = 44;
    SAFE = 45;
    VL1  = 46;
    VL2  = 47;
    TL1  = 48;
    TL2  = 49;
    VOLTL1=50;
    VOLTL2=51;
    VOLTH1=52;
    VOLTH2=53;
    PWR  = 54;
    MAXPWR=55;
    IFAULT=56;
    VNOM = 57;
    
    /* Force/Torque sensor */
    FT   = 54;
  }

}
 
 CANbus::~CANbus(){
    if(pucks!=NULL) delete pucks; 
    if(trq!=NULL) delete trq; 
    if(pos!=NULL) delete pos;
 }
