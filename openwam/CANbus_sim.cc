/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Labs Pittsburgh *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
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
   int i = 0;
   if(firmwareVersion < 40){
      VERS = i++;
      ROLE = i++;
      SN = i++;
      ID = i++;
      ERROR = i++;
      STAT = i++;
      ADDR = i++;
      VALUE = i++;
      MODE = i++;
      D = i++;
      TORQ = i++;
      P = i++;
      V = i++;
      E = i++;
      B = i++;
      MD = i++;
      MT = i++;
      MV = i++;
      MCV = i++;
      MOV = i++;
      MOFST = i++;
      IOFST = i++;
      PTEMP = i++;
      UPSECS = i++;
      OD = i++;
      MDS = i++;
      AP = i++;
      AP2 = i++;
      MECH = i++;
      MECH2 = i++;
      CTS = i++;
      CTS2 = i++;
      DP = i++;
      DP2 = i++;
      OT = i++;
      OT2 = i++;
      CT = i++;
      CT2 = i++;
      BAUD = i++;
      TEMP = i++;
      OTEMP = i++;
      _LOCK = i++;
      DIG0 = i++;
      DIG1 = i++;
      ANA0 = i++;
      ANA1 = i++;
      THERM = i++;
      VBUS = i++;
      IMOTOR = i++;
      VLOGIC = i++;
      ILOGIC = i++;
      GRPA = i++;
      GRPB = i++;
      GRPC = i++;
      PIDX = i++;
      ZERO = i++;
      SG = i++;
      HSG = i++;
      LSG = i++;
      _DS = i++;
      IVEL = i++;
      IOFF = i++;
      MPE = i++;
      EN = i++;
      TSTOP = i++;
      KP = i++;
      KD = i++;
      KI = i++;
      SAMPLE = i++;
      ACCEL = i++;
      TENSION = i++;
      UNITS = i++;
       RATIO = i++;
      LOG = i++;
      DUMP = i++;
      LOG1 = i++;
      LOG2 = i++;
      LOG3 = i++;
      LOG4 = i++;
      GAIN1 = i++;
      GAIN2 = i++;
      GAIN3 = i++;
      OFFSET1 = i++;
      OFFSET2 = i++;
      OFFSET3 = i++;
      PEN = i++;
      SAFE = i++;
      SAVE = i++;
      LOAD = i++;
      DEF = i++;
      VL1 = i++;
      VL2 = i++;
      TL1 = i++;
      TL2 = i++;
      VOLTL1 = i++;
      VOLTL2 = i++;
      VOLTH1 = i++;
      VOLTH2 = i++;
      MAXPWR = i++;
      PWR = i++;
      IFAULT = i++;
      IKP = i++;
      IKI = i++;
      IKCOR = i++;
      VNOM = i++;
      TENST = i++;
      TENSO = i++;
      JIDX = i++;
      IPNM = i++;
      
      PROP_END = i++;

      T = TORQ;
      FET0 = B;
      FET1 = TENSION;
      /*
      HALLS = i++;
      HALLH = i++;
      HALLH2 = i++;
      POLES = i++;
      ECMAX = i++;
      ECMIN = i++;
      ISQ = i++;
      TETAE = i++;
      FIND = i++;
      LCV = i++;
      LCVC = i++;
      LFV = i++;
      LFS = i++;
      LFAP = i++;
      LFDP = i++;
      LFT = i++;
      VALUE32 = i++;
      */
   }
   else
   {
   /* Common */
      VERS = i++;
      ROLE = i++; /* P=PRODUCT, R=ROLE: XXXX PPPP XXXX RRRR */
      SN = i++;
      ID = i++;
      ERROR = i++;
      STAT = i++;
      ADDR = i++;
      VALUE = i++;
      MODE = i++;
      TEMP = i++;
      PTEMP = i++;
      OTEMP = i++;
      BAUD = i++;
      _LOCK = i++;
      DIG0 = i++;
      DIG1 = i++;
      FET0 = i++;
      FET1 = i++;
      ANA0 = i++;
      ANA1 = i++;
      THERM = i++;
      VBUS = i++;
      IMOTOR = i++;
      VLOGIC = i++;
      ILOGIC = i++;
      SG = i++;
      GRPA = i++;
      GRPB = i++;
      GRPC = i++;
      CMD = i++; /* For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T */
      SAVE = i++;
      LOAD = i++;
      DEF = i++;
      FIND = i++;
      X0 = i++;
      X1 = i++;
      X2 = i++;
      X3 = i++;
      X4 = i++;
      X5 = i++;
      X6 = i++;
      X7 = i++;
      
      COMMON_END = i++;
   
   /* Safety */
      i = COMMON_END;
      ZERO = i++;
      PEN = i++;
      SAFE = i++;
      VL1 = i++;
      VL2 = i++;
      TL1 = i++;
      TL2 = i++;
      VOLTL1 = i++;
      VOLTL2 = i++;
      VOLTH1 = i++;
      VOLTH2 = i++;
      PWR = i++;
      MAXPWR = i++;
      IFAULT = i++;
      VNOM = i++;
      
      SAFETY_END = i++;
   
   /* Tater */
      i = COMMON_END;
      T = i++;
      MT = i++;
      V = i++;
      MV = i++;
      MCV = i++;
      MOV = i++;
      P = i++; /* 32-Bit Present Position */
      P2 = i++;
      DP = i++; /* 32-Bit Default Position */
      DP2 = i++;
      E = i++; /* 32-Bit Endpoint */
      E2 = i++;
      OT = i++; /* 32-Bit Open Target */
      OT2 = i++;
      CT = i++; /* 32-Bit Close Target */
      CT2 = i++;
      M = i++; /* 32-Bit Move command for CAN*/
      M2 = i++;
      _DS = i++;
      MOFST = i++;
      IOFST = i++;
      UPSECS = i++;
      OD = i++;
      MDS = i++;
      MECH = i++; /* 32-Bit */
      MECH2 = i++;
      CTS = i++; /* 32-Bit */
      CTS2 = i++;
      PIDX = i++;
      HSG = i++;
      LSG = i++;
      IVEL = i++;
      IOFF = i++; /* 32-Bit */
      IOFF2 = i++;
      MPE = i++;
      EN = i++;
      TSTOP = i++;
      KP = i++;
      KD = i++;
      KI = i++;
      ACCEL = i++;
      TENST = i++;
      TENSO = i++;
      JIDX = i++;
      IPNM = i++;
      HALLS = i++;
      HALLH = i++; /* 32-Bit */
      HALLH2 = i++;
      POLES = i++;
      IKP = i++;
      IKI = i++;
      IKCOR = i++;
      HOLD = i++;
      TIE = i++;
      ECMAX = i++;
      ECMIN = i++;
      LFLAGS = i++;
      LCTC = i++;
      LCVC = i++;
      
      PROP_END = i++;
      
      AP = P; // Handle parameter name change
      TENSION = FET1;
   }
}
 
 CANbus::~CANbus(){
    if(pucks!=NULL) delete pucks; 
    if(trq!=NULL) delete trq; 
    if(pos!=NULL) delete pos;
 }
