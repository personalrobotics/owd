// #define IDF_DEBUG

#include "CANbus.hh"
#include "CANdefs.hh"
#include <ros/ros.h>
#include <native/task.h>
#include <stdio.h>
#include <fcntl.h>

#define PUCK_IDLE 0 
#define MODE_IDLE      0
#define MODE_TORQUE    2
#define MODE_PID       3
#define MODE_VELOCITY  4
#define MODE_TRAPEZOID 5


CANbus::CANbus(int32_t bus_id, int num_pucks) :  // DONE MVW
  puck_state(-1),id(bus_id),trq(NULL),
  pos(NULL), pucks(NULL),npucks(num_pucks),
  simulation(false)
{
  pthread_mutex_init(&busmutex, NULL);
  pthread_mutex_init(&trqmutex, NULL);
  pthread_mutex_init(&posmutex, NULL);
  pthread_mutex_init(&runmutex, NULL);
  pthread_mutex_init(&statemutex, NULL);
#ifdef BH280
  pthread_mutex_init(&handmutex, NULL);
  handstate = HANDSTATE_UNINIT;
  first_moving_finger=11;
#endif // BH280

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
#ifdef PEAK_CAN
  can_accept[0] = 0x0000;  mask[0] = 0x03E0;
  can_accept[1] = 0x0403;  mask[1] = 0x03E0;
  can_accept[2] = 0x0406;  mask[2] = 0x03E0;
#endif // PEAK_CAN

  snprintf(last_error,200,"");
}

int CANbus::init(){  // DONE MVW
  if(open() == OW_FAILURE){
    ROS_ERROR("CANbus::init: open failed.");
    return OW_FAILURE;
  }

  if(check() == OW_FAILURE){
    ROS_ERROR("CANbus::init: check failed.");
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::open(){  // DONE MVW
  pthread_mutex_lock(&busmutex);
   
#ifdef PEAK_CAN
  DWORD  err;

  char devicename[20];
  snprintf(devicename,20,"/dev/pcan%d",id);
  handle = LINUX_CAN_Open(devicename,O_RDWR);
  
  if (!handle) {
    ROS_ERROR("CANbus::open(): CAN_Open(): cannot open device");
    pthread_mutex_unlock(&busmutex);
    throw OW_FAILURE;
  }
  
  // Clear Status
  err = CAN_Status(handle);
  ROS_DEBUG("CANbus::open(): bus status = 0x%x",err);
  
  err = CAN_Init(handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
  if (err) {
    ROS_ERROR("CANbus::open(): CAN_Init(): failed with 0x%x",err);
    pthread_mutex_unlock(&busmutex);
    return OW_FAILURE;		
  }
  pthread_mutex_unlock(&busmutex);
  
  if (err = CAN_ResetFilter(handle)) {
    ROS_ERROR("CANbus::open(): Could not Reset Filter: 0x%x",err);
  }
  if (err = CAN_MsgFilter(handle, 0x0000, 0x07FF, MSGTYPE_STANDARD)) {
    ROS_ERROR("CANbus::open(): Could not set Msg Filter: 0x%x",err);
  }
	
#endif // PEAK_CAN
#ifdef ESD_CAN

  if(canOpen(id, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, 
	     TX_TIMEOUT, RX_TIMEOUT, &handle) != NTCAN_SUCCESS){  
    ROS_ERROR("CANbus::open: canOpen failed.");
    pthread_mutex_unlock(&busmutex);
    return OW_FAILURE;
  }   
  
  // 0 = 1Mbps, 2 = 500kbps, 4 = 250kbps
  if(canSetBaudrate(handle, 0) != NTCAN_SUCCESS){
    ROS_ERROR("CANbus::opent: canSetBaudrate failed.");
    pthread_mutex_unlock(&busmutex);
    return OW_FAILURE;
  }
  pthread_mutex_unlock(&busmutex);
    
  // Mask 3E0: 0000 0011 1110 0000
  // Messages sent directly to host
  if(allow_message(0x0000, 0x03E0) == OW_FAILURE){
    ROS_ERROR("CANbus::open: allow_message failed.");
    return OW_FAILURE;
  }
  // Group 3 messages
  if(allow_message(0x0403, 0x03E0) == OW_FAILURE){
    ROS_ERROR("CANbus::open: allow_message failed.");
    return OW_FAILURE;
  }
  // Group 6 messages
  if(allow_message(0x0406, 0x03E0) == OW_FAILURE){
    ROS_ERROR("CANbus::open: allow_message failed.");
    return OW_FAILURE;
  }
#endif // ESD_CAN

  // Set the minimum required property values
  // (we'll set the rest once we know the pucks' firmware version
  VERS = 0;
  STAT = 5;
  PROP_END = 10;
   
  return OW_SUCCESS;
}

int CANbus::check(){  // DONE MVE
  int32_t nodes[NUM_NODES+1];//, value;
  int online_pucks, reset_pucks, running_pucks;

  //Get status of pucks on bus
  usleep(100000);
  if(status(nodes) == OW_FAILURE){
    return OW_FAILURE;
  }
   
  //count number of live pucks
  online_pucks = 0;
  for(int n=NODE_MIN; n<=NODE_MAX; n++){
    
    if(nodes[n] != STATUS_OFFLINE){      // display online nodes
      ROS_DEBUG_NAMED("cancheck","Node (id, status): (%d,%s)",n,(nodes[n] == STATUS_RESET)?"reset":"running");
    }
    
    if(nodes[n]!=STATUS_OFFLINE && n!=SAFETY_MODULE) {
      online_pucks++;
    }
  }
  
  if (online_pucks==0){
    ROS_INFO_NAMED("cancheck","The wam appears to be turned off");
    return OW_FAILURE;
  }
#ifdef BH280
  if(online_pucks < npucks + 4 ){
#else
  if(online_pucks < npucks){
#endif
    ROS_INFO_NAMED("cancheck","Bus has %d pucks. We expected %d",online_pucks,npucks);
    return OW_FAILURE;
  }  
#ifndef BH280_ONLY
  if(nodes[SAFETY_MODULE] == STATUS_OFFLINE){
    ROS_DEBUG_NAMED("cancheck","Safety module is offline");
    return OW_FAILURE;
  }

  reset_pucks = 0;
  running_pucks = 0;
  for(int p=1; p<=npucks; p++){
    if(nodes[ pucks[p].id() ] == STATUS_RESET)
      reset_pucks++;
    else if(nodes[ pucks[p].id() ] != STATUS_OFFLINE)
      running_pucks++;
  }
   
  ROS_DEBUG_NAMED("cancheck","Expected: %d; Online: %d",npucks,online_pucks);
  ROS_DEBUG_NAMED("cancheck","Running: %d; Reset: %d",running_pucks,reset_pucks);
      
  if((running_pucks+reset_pucks)!=npucks){
    ROS_WARN_NAMED("cancheck","Some of the pucks must have reset unexpectedly");
    ROS_WARN_NAMED("cancheck","ALL pucks should be in either running or reset state");
    return OW_FAILURE;
  }
      
  ROS_INFO_NAMED("cancheck","  Initializing pucks 1 to %d...",npucks);
  for(int p=1; p<=npucks; p++){

    ROS_DEBUG_NAMED("cancheck","Checking puck %d",pucks[p].id());
	 
    if(nodes[ pucks[p].id() ] == STATUS_RESET){
      ROS_DEBUG_NAMED("cancheck"," (reset) ->Waking up the puck...");
      if(wake_puck(pucks[p].id()) == OW_FAILURE){
	ROS_WARN_NAMED("cancheck","wake_puck failed.");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");
      usleep(10000);
      
      ROS_DEBUG_NAMED("cancheck","Setting PID values");
      if(set_property(pucks[p].id(),PIDX, pucks[p].order(),true)==OW_FAILURE){
	ROS_WARN_NAMED("cancheck","set_property failed (PID)");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");

      ROS_DEBUG_NAMED("cancheck","Setting max torque...");
      int32_t max_torque;
      if (p<4) { // pucks 1-3 (shoulder)
          max_torque = 4860;
      } else if (p==4) {  // puck 4 (elbow)
          max_torque = 4320;
      } else if (p<7) {  // pucks 5 and 6 (wrist diff)
          max_torque = 3900;
      } else if (p==7) { // puck 7 (wrist final twist)
          max_torque = 1370;
      } else {
	ROS_ERROR("Unknown puck id of %d",p);
	throw -1;  // unknown puck id
      }
          
      if(set_property(pucks[p].id(), MT, max_torque, true) == OW_FAILURE){
	ROS_WARN_NAMED("cancheck","set_property failed (max torque)");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");
    }
    else{
      ROS_DEBUG_NAMED("cancheck"," (running)");
      ROS_DEBUG_NAMED("cancheck","setting idle mode...");
      // this was set to CONTROLLER_IDLE originally
      if(set_property(pucks[p].id(), MODE, PUCK_IDLE, true) == OW_FAILURE){
	ROS_WARN_NAMED("cancheck","set_property failed (MODE)");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");
    }
  }
  ROS_INFO_NAMED("cancheck","done.");
#endif  // not BH280_ONLY
#ifdef BH280
  ROS_INFO_NAMED("can_bh280","  Initializing hand pucks 11 to 14...");
  if (hand_activate() != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280","Hand not initialized");
    return OW_FAILURE;
  }
#endif // BH280
  ROS_INFO_NAMED("can_bh280","done.");

  return OW_SUCCESS;
}

void CANbus::dump(){
  //  ROS_DEBUG("Printing bus information.");
  for(int g=GROUP_ID_MIN; g<=GROUP_ID_MAX; g++){
    //    if(groups[g].id() != GROUP_INVALID)
      // ROS_DEBUG_STREAM(groups[g]);
  }
}
   
int CANbus::allow_message(int32_t id, int32_t mask){  // MVW DONE 
  
  pthread_mutex_lock(&busmutex);

#ifdef PEAK_CAN
  DWORD err;
  if ((err = CAN_ResetFilter(handle)) ||
      (err = CAN_MsgFilter(handle, 0x0000, 0x07FF, MSGTYPE_STANDARD))) {
    pthread_mutex_unlock(&busmutex);
    ROS_WARN("CANbus::allow_message: Could not set Msg Filter: 0x%x",err);
    return OW_FAILURE;
  }
#endif // PEAK_CAN

#ifdef ESD_CAN
  int i;
  for(i=0; i<2048; i++){
    if((i & ~mask) == id){
      if(canIdAdd(handle, i) != NTCAN_SUCCESS){
	pthread_mutex_unlock(&busmutex);
	ROS_WARN("CANbus::allow_message: canIdAdd failed.");
	return OW_FAILURE;
      }
    }
  }
#endif  // ESD_CAN

  pthread_mutex_unlock(&busmutex);
  return OW_SUCCESS;
}
 
int CANbus::wake_puck(int32_t puck_id){  // DONE MVW
    if(set_property(puck_id, STAT, STATUS_READY, false) == OW_FAILURE){
      ROS_WARN("CANbus::wake_puck: set_property failed for puck %d.",puck_id);
        return OW_FAILURE;
    }
    usleep(750000); // Wait 750ms for puck to initialize    
    return OW_SUCCESS;
}

// Got to make sure that nodes is NUM_NODES+1 long !!!
int CANbus::status(int32_t* nodes){
  unsigned char msg[8];
  int32_t msgid, msglen, nodeid, property;
  int firstFound = 0;
  int32_t fw_vers;

  ROS_INFO_NAMED("canstatus","Probing for nodes on the CAN bus");

  pthread_mutex_lock(&busmutex);

  for(int n=NODE_MIN; n<=NODE_MAX; n++){
    msg[0] = (unsigned char) 5;   // STAT = 5 for all firmware versions
    nodes[n] = STATUS_OFFLINE;      // Initialize node as offline

    if(send(NODE2ADDR(n), msg, 1, true) == OW_FAILURE){
      pthread_mutex_unlock(&busmutex);
      ROS_WARN_NAMED("canstatus","send failed: %s",last_error);
      return OW_FAILURE;
    }
    ROS_DEBUG_NAMED("canstatus","Sent probe message to puck %d id %d",n,NODE2ADDR(n));

    usleep(40000);
    if(read(&msgid, msg, &msglen, false) == OW_FAILURE){
      ROS_DEBUG_NAMED("canstatus","node %d didn't answer",NODE2ADDR(n));
    }
    else{
      if(parse(msgid, msg, msglen,&nodeid,&property,&nodes[n])==OW_FAILURE){
	pthread_mutex_unlock(&busmutex);
	ROS_DEBUG_NAMED("canstatus","parse failed: %s",last_error);
	return OW_FAILURE;
      } else {
          // parsed ok
	ROS_DEBUG_NAMED("canstatus","parsed response from node %d",NODE2ADDR(n));
	if (!firstFound) {
	  pthread_mutex_unlock(&busmutex);
	  ROS_DEBUG_NAMED("canstatus","trying to wake node %d",n);
	  if ((wake_puck(NODE2ADDR(n)) == OW_SUCCESS) &&
	      (get_property(NODE2ADDR(n), 0, &fw_vers) == OW_SUCCESS)){
	    ROS_DEBUG_NAMED("canstatus","puck %d firmware version %d",n,fw_vers);
	    initPropertyDefs(fw_vers);
	    firstFound = 1;
	  }
	  else {
	    ROS_DEBUG_NAMED("canstatus","unable to get firmware vers from puck %d",n);
	  }
	  pthread_mutex_lock(&busmutex);
	}
      }
    }
  }
  pthread_mutex_unlock(&busmutex);

  if (!firstFound) {
      // we never got a firmware version, so we never initialized
      // the property definitions.
    ROS_WARN_NAMED("canstatus","Could not determine puck firmware version");
    ROS_WARN_NAMED("canstatus","Unable to continue");
    return OW_FAILURE;
  }
  
  ROS_INFO_NAMED("canstatus","done");
  return OW_SUCCESS;
}

int CANbus::set_property(int32_t nid, int32_t property, int32_t value,bool check){
  uint8_t msg[8];
  int32_t response;
  int32_t msglen;
   
  if(compile(property, value, msg, &msglen) ==OW_FAILURE){
    ROS_WARN("CANbus::set_property: compile failed.");
    return OW_FAILURE;
  }
  msg[0] |= (uint8_t)0x80; // Set the 'Set' bit

  pthread_mutex_lock(&busmutex);
  if(send(NODE2ADDR(nid), msg, msglen, true) == OW_FAILURE){
    pthread_mutex_unlock(&busmutex);
    ROS_WARN("CANbus::set_property: send failed: %s",last_error);
    return OW_FAILURE;
  }
  pthread_mutex_unlock(&busmutex);
   
  if(check){
    // Get the new value of the property
    if(get_property(nid, property, &response) == OW_FAILURE){
      ROS_WARN("CANbus::set_property: get_property failed.");
      return OW_FAILURE;
    }

    // Compare response to value
    if(response != value){
      ROS_WARN("CANbus::set_property: value confirmation failed.");
      ROS_WARN("Puck %d, Property %d Set Value %d Response Value %d",nid,property,value,response);
      return OW_FAILURE;
    }
  }   

  return OW_SUCCESS;
}

int CANbus::get_property(int32_t nid, int32_t property, int32_t* value){
  uint8_t msg[8];
  int32_t msgid, msglen, nodeid, prop;
  *value=0;

  msg[0] = (uint8_t)property;   

  pthread_mutex_lock(&busmutex);
  if(send(NODE2ADDR(nid), msg, 1, true) == OW_FAILURE){
    ROS_WARN("CANbus::get_property: send failed: %s",last_error);
    pthread_mutex_unlock(&busmutex);
    return OW_FAILURE;
  }
  if(read(&msgid, msg, &msglen, true) == OW_FAILURE){
    pthread_mutex_unlock(&busmutex);
    ROS_WARN("CANbus::get_property: read failed: %s",last_error);
    return OW_FAILURE;
  }
  pthread_mutex_unlock(&busmutex);
   
  // Parse the reply
  if(parse(msgid, msg, msglen, &nodeid, &prop, value) == OW_FAILURE){
    ROS_WARN("CANbus::get_property: parse failed: %s",last_error);
    return OW_FAILURE;
  }
      
  // Check that the ids and properties match
  if(nodeid!=nid) {
    ROS_WARN("Asked for property %d from node %d but got answer from node %d",
    	     property, nid, nodeid);
    return OW_FAILURE;
  }
  else if (prop!=property) {
    ROS_WARN("Asked node %d for property %d but got property %d",
    	     nid, property, prop);
    return OW_FAILURE;
  }   
  return OW_SUCCESS;
}

int CANbus::send_torques(int32_t* torques){
  if (! pthread_mutex_trylock(&trqmutex)) {
    for(int p=1; p<=npucks; p++) {
      trq[p] = torques[p];
    }
    pthread_mutex_unlock(&trqmutex);
  }
  return OW_SUCCESS;
}

int CANbus::send_torques(){
  uint8_t msg[8];
  int32_t torques[NUM_ORDERS+1];
  Puck* puck;

  static double sendtime=0.0f;
  static unsigned int sendcount=0;

  // make a quick copy so that we can release the mutex
  static int32_t *mytorqs = (int32_t *) malloc(NUM_NODES * sizeof(int32_t));

  pthread_mutex_lock(&trqmutex);
  memcpy(mytorqs,trq,NUM_NODES*sizeof(int32_t));
  pthread_mutex_unlock(&trqmutex);

  static int DEBUGCOUNT=0;
  if (++DEBUGCOUNT == 1000) {
    DEBUGCOUNT = 0;
  }

#ifdef RT_STATS
  RTIME bt1 = rt_timer_ticks2ns(rt_timer_read());
#endif

  for(int g=GROUP_ID_MIN; g<=GROUP_ID_MAX; g++){
      
      if(groups[g].id() != GROUP_INVALID){
          
          for(int p=PUCK_ORDER_MIN; p<=PUCK_ORDER_MAX; p++){
              
              puck = groups[g].puck(p); 
              if(puck){ 
                  torques[p] = clip(mytorqs[puck->motor()], 
                                    Puck::MIN_TRQ[puck->id()], 
                                    Puck::MAX_TRQ[puck->id()] );
		  if (DEBUGCOUNT==0) {
		    //		    ROS_DEBUG_NAMED("torques","Puck %d Torque %d",puck->id(),torques[p]);
		  }
              }
              else{
                  torques[p] = 0;	    
              }
          }

          msg[0] = TORQ | 0x80; 
          msg[1] = (uint8_t)(( torques[1]>>6)&0x00FF);
          msg[2] = (uint8_t)(((torques[1]<<2)&0x00FC) | ((torques[2]>>12)&0x0003));
          msg[3] = (uint8_t)(( torques[2]>>4)&0x00FF);
          msg[4] = (uint8_t)(((torques[2]<<4)&0x00F0) | ((torques[3]>>10)&0x000F));
          msg[5] = (uint8_t)(( torques[3]>>2)&0x00FF);
          msg[6] = (uint8_t)(((torques[3]<<6)&0x00C0) | ((torques[4]>>8) &0x003F));
          msg[7] = (uint8_t)(  torques[4]    &0x00FF);
          
	  pthread_mutex_lock(&busmutex);
          if(send(GROUPID(groups[g].id()), msg, 8, false) == OW_FAILURE) {
	    pthread_mutex_unlock(&busmutex);
	    ROS_ERROR("CANbus::set_torques: send failed: %s",last_error);
	    return OW_FAILURE;
          }
	  pthread_mutex_unlock(&busmutex);
	  
      }
      
  }
  
#ifdef RT_STATS
  RTIME bt2 = rt_timer_ticks2ns(rt_timer_read());
  sendtime += (bt2-bt1) * 1e-6; // ns to ms
  if (++sendcount == 1000) {
    stats.cansend_time = sendtime/1000.0;
    sendcount=0;
    sendtime=0.0f;
  }
#endif
  
  return OW_SUCCESS;
}

int CANbus::read_torques(int32_t* mtrq){
  uint8_t  msg[8];
  int32_t value;
  int32_t msgid, msglen, nodeid, property;

  // Compile the packet
  msg[0] = (uint8_t)TORQ;

  pthread_mutex_lock(&busmutex);

  if(send(GROUPID(0), msg, 1, true) == OW_FAILURE){
    pthread_mutex_unlock(&busmutex);
    ROS_WARN("CANbus::read_torques: send failed: %s", last_error);
    return OW_FAILURE;
  }
  for(int p=1; p<=npucks; ){
    if(read(&msgid, msg, &msglen, true) == OW_FAILURE){
      pthread_mutex_unlock(&busmutex);
      ROS_WARN("CANbus::read_torques: read failed on puck %d: %s",p,last_error);
      return OW_FAILURE;
    }
    if(parse(msgid, msg, msglen, &nodeid, &property, &value) == OW_FAILURE){
      pthread_mutex_unlock(&busmutex);
      ROS_WARN("CANbus::read_torques: parse failed: %s",last_error);
      return OW_FAILURE;
    }
    if(property == TORQ){
      trq[nodeid] = value;
      p++;
    }
  }
  pthread_mutex_unlock(&busmutex);

  return OW_SUCCESS;
}

int CANbus::read_positions(double* positions){

  pthread_mutex_lock(&posmutex);
  for(int p=1; p<=npucks; p++) {
    positions[p] = pos[p];
  }
  pthread_mutex_unlock(&posmutex);

  return OW_SUCCESS;
}

int CANbus::read_positions(){
  uint8_t  msg[8];

  int32_t data[NUM_NODES+1], value;
  int32_t msgid, msglen, property, nodeid;
  static double sendtime=0.0f;
  static double readtime=0.0f;
  static unsigned int loopcount=0;

  // Compile the packet
  msg[0] = (uint8_t)AP;


#ifdef RT_STATS
  RTIME bt1 = rt_timer_ticks2ns(rt_timer_read());
#endif

  pthread_mutex_lock(&busmutex);

  if(send(GROUPID(0), msg, 1, false) == OW_FAILURE){
    pthread_mutex_unlock(&busmutex);
    ROS_WARN("CANbus::get_positions: send failed: %s",last_error);
    return OW_FAILURE;
  }

#ifdef RT_STATS
  RTIME bt2 = rt_timer_ticks2ns(rt_timer_read());
  sendtime += (bt2-bt1) * 1e-6; // ns to ms
#endif

#ifdef BH280
  for(int p=1; p<=npucks + 4; ){
#else
  for(int p=1; p<=npucks; ){
#endif // BH280
    if(read(&msgid, msg, &msglen, true) == OW_FAILURE){
      pthread_mutex_unlock(&busmutex);
      ROS_WARN("CANbus::get_positions: read failed: %s",last_error);
      return OW_FAILURE;
    }

    if(parse(msgid, msg, msglen, &nodeid, &property, &value) == OW_FAILURE){
      pthread_mutex_unlock(&busmutex);
      ROS_WARN("CANbus::get_positions: parse failed: %s",last_error);
      return OW_FAILURE;
    }
    if (property == AP) {
      data[nodeid] = value;
      p++;
    }
  }
  pthread_mutex_unlock(&busmutex);

#ifdef RT_STATS
  bt1 = rt_timer_ticks2ns(rt_timer_read());
  readtime += (bt1-bt2) * 1e-6; // ns to ms
#endif

  // convert the results
  pthread_mutex_lock(&posmutex);
  for(int p=1; p<=npucks; p++)
    pos[ pucks[p].motor() ] = 2.0*M_PI*( (double) data[ pucks[p].id() ] )/ 
                                       ( (double) pucks[p].CPR() );
  pthread_mutex_unlock(&posmutex);

#ifdef BH280
  // it's ok if we miss a few position updates, so just "try"
  if (!pthread_mutex_trylock(&handmutex)) {
    for (int p=1; p<=4; ++p) {
      hand_positions[p] = data[p+10];
    }
    pthread_mutex_unlock(&handmutex);
  }
#endif // BH280

  if (++loopcount == 1000) {
    stats.canread_sendtime=sendtime/1000.0;
    stats.canread_readtime=readtime/1000.0;
    sendtime=readtime=0.0f;
    loopcount=0;
  }

  return OW_SUCCESS;
}

int CANbus::send_positions(double* mpos){
  int32_t position[npucks+1];

  for(int p=1; p<=npucks; p++){
    position[p] = (int32_t)floor( mpos[ pucks[p].motor() ]*
			    pucks[p].CPR()/(2.0*M_PI) );
  }   
  return send_AP(position);
}

int CANbus::send_AP(int32_t* apval){

  if(set_property(SAFETY_MODULE, IFAULT, 8, true) == OW_FAILURE){
    ROS_WARN("CANbus::send_AP: set_property IFAULT=8 failed.");
    return OW_FAILURE;
  }

  for(int p=1; p<=npucks; p++){
    if(set_property(pucks[p].id(), AP, apval[p], false) == OW_FAILURE){
      ROS_WARN("CANbus::send_AP: set_property AP failed.");
      return OW_FAILURE;
    }
  }

  // let the safety module see the new positions
  read_positions();
   
  // start monitoring tip velocity again
  if(set_property(SAFETY_MODULE, ZERO, 1, true) == OW_FAILURE){
    ROS_WARN("CANbus::send_AP: set_property ZERO=1 failed.");
    return OW_FAILURE;
  }

  return OW_SUCCESS;
}

int CANbus::parse(int32_t msgid, uint8_t* msg, int32_t msglen,
		  int32_t* nodeid, int32_t* property, int32_t* value){

  // THIS FUNCTION IS OFTEN CALLED WHILE HOLDING THE BUS MUTEX.
  // DON'T ADD ANY TERMINAL I/O TO THIS FUNCTION (IT WILL SLOW THINGS
  // DOWN TOO MUCH).

  int32_t i;
  int32_t dataHeader;

  *nodeid = ADDR2NODE(msgid);
  if(*nodeid == -1){
    snprintf(last_error,200,"invalid node id %d",*nodeid);
    return OW_FAILURE;
  }
   
  dataHeader = ((msg[0] >> 6) & 0x0002) | ((msgid & 0x041F) == 0x0403);

  switch (dataHeader){
      
  case 3:  // Data is a packed 22-bit position, SET 
    *value = 0x00000000;
    *value |= ( (int32_t)msg[0] << 16) & 0x003F0000;
    *value |= ( (int32_t)msg[1] << 8 ) & 0x0000FF00;
    *value |= ( (int32_t)msg[2] )      & 0x000000FF;
      
    if(*value & 0x00200000) // If negative 
      *value |= 0xFFC00000; // Sign-extend 
      
    *property = AP;

#ifdef JOINT_ENCODERS
      jointPosition[*nodeid] = 0;
      jointPosition[*nodeid] |= ( (int32_t)messageData[3] << 16) & 0x003F0000;
      jointPosition[*nodeid] |= ( (int32_t)messageData[4] << 8 ) & 0x0000FF00;
      jointPosition[*nodeid] |= ( (int32_t)messageData[5] ) & 0x000000FF;
      
      if (jointPosition[*nodeid] & 0x00200000) /* If negative */
         jointPosition[*nodeid] |= 0xFFC00000; /* Sign-extend */
#endif // JOINT_ENCODERS

    break;

  case 2:  // Data is normal, SET 

     /***************************************
      ***   UPDATED CODE FROM BARRETT     ***
      *** Need to switch to this as       ***
      *** soon as I have time for testing ***
      *** Mike Vande Weghe 9/19/2009      ***
      ***************************************

      *property = messageData[0] & 0x7F;
      *value = messageData[len-1] & 0x80 ? -1L : 0;
      for (i = len-1; i >= 2; i--)
         *value = *value << 8 | messageData[i];
      break;

      ***  End updateded code (replaces entire case) ***/

    *property = msg[0] & 0x7F;

    //
    // Store the value
    // second byte of message is zero (for DSP word alignment) 
    //
    *value = 0;
    for(i=0; i<msglen-2; i++)
      *value |= ((unsigned int32_t)msg[i + 2] << (i * 8))
	& (0x000000FF << (i * 8));
      
    if (*value & (1 << ((i*8) - 1)))
      *value |= 0xFFFFFFFF << (i * 8); // Sign extend the value 
    break;

  case 0:  // Assume firmware request (GET) 
      
    // A negative (or zero) property means GET
    *property = -(msg[0] & 0x7F); 
    *value = 0;
    break;
      
  default:
    snprintf(last_error,200,"Illegal message header.");
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::compile(int32_t property, int32_t value, 
		    uint8_t *msg, int32_t *msglen){
  int i;
   
  // Check the property
  if(PROP_END < property){
    ROS_WARN("CANbus::compile: invalid property (%d > %d).",property,PROP_END);
    return OW_FAILURE;
  }
   
  msg[0] = (uint8_t)property; // Insert the property
  msg[1] = 0;                 // To align the values for the tater's DSP
   
  // Append the value 
  for(i=2; i<6; i++){
    msg[i] = (uint8_t)(value & 0x000000FF);
    value >>= 8;
  }
   
  // Record the proper data length 
  // this was also removed from Barrett code:
  //    *msglen = (dataType[property] & 0x0007) + 2;
  *msglen = 6;
   
  return OW_SUCCESS;
}

int CANbus::read(int32_t* msgid, uint8_t* msgdata, int32_t* msglen, bool block){ // DONE MVW

  int32_t len = 1;
  int i, err;
  
  // THIS FUNCTION MUST BE CALLED WHILE HOLDING THE BUS MUTEX.
  // DON'T ADD ANY TERMINAL I/O TO THIS FUNCTION (IT WILL SLOW THINGS
  // DOWN TOO MUCH).

#ifdef PEAK_CAN
  TPCANRdMsg cmsg;
  int pendread=0;
  int pendwrite=0;

  if (block) {
    err = LINUX_CAN_Read(handle, &cmsg);
  } else {
    err = LINUX_CAN_Extended_Status(handle,&pendread, &pendwrite);
    if ((err != CAN_ERR_OK) && (err != CAN_ERR_QRCVEMPTY)) {
      //      ROS_INFO("CANbus status is 0x%x",err);
    }
    if (pendread) {
      err = LINUX_CAN_Read(handle,&cmsg);
    } else {
      // sleep briefly and try once more
      usleep(100);
      pendread=pendwrite=0;
      err = LINUX_CAN_Extended_Status(handle,&pendread, &pendwrite);
      if ((err != CAN_ERR_OK) && (err != CAN_ERR_QRCVEMPTY)) {
	//	ROS_INFO("CANbus status is 0x%x",err);
      }
      if (!pendread) {
	snprintf(last_error,200,"nothing to read");
	return OW_FAILURE; // just means nothing was read
      } else {
	err = LINUX_CAN_Read(handle,&cmsg);
      }
    }
  }
  if (err != CAN_ERR_OK) {
    snprintf(last_error,200,"LINUX_CAN_Read failed: 0x%x",err);
    return OW_FAILURE;
  }
  *msgid = cmsg.Msg.ID;
  *msglen = cmsg.Msg.LEN;
  for (i=0; i< *msglen; ++i) {
    msgdata[i] = cmsg.Msg.DATA[i];
  }
  return OW_SUCCESS;
#endif // PEAK_CAN

#ifdef ESD_CAN
  CMSG cmsg;

  if(block){
    //    int count=10;
    //    while (((err = canRead(handle, &cmsg, &len, NULL)) == NTCAN_RX_TIMEOUT) &&
    //	   (++count > 0)) {
    //      usleep(400);
    //    }
    err = canRead(handle, &cmsg, &len, NULL);

    if (err != NTCAN_SUCCESS) {
      snprintf(last_error,200,"canRead failed: 0x%x",err);
      return OW_FAILURE;
    }
  }
  else{
    if((err=canTake(handle, &cmsg, &len)) != NTCAN_SUCCESS){
      snprintf(last_error,200,"canTake failed: 0x%x",err);
      return OW_FAILURE;
    }
  }

  if(len == 0){
    snprintf(last_error,200,"read something with length zero");
    return OW_FAILURE;
  } else if(len != 1){
    snprintf(last_error,200,"received a message of length: %d",len);
    return OW_FAILURE;
  }

  *msgid = cmsg.id;
  *msglen = cmsg.len;
  for(i=0; i<*msglen; i++)
    msgdata[i] = cmsg.data[i];
 
  return OW_SUCCESS;
#endif // ESD_CAN
}

int CANbus::send(int32_t msgid, uint8_t* msgdata, int32_t msglen, bool block){ // DONE MVW

  // THIS FUNCTION MUST BE CALLED WHILE HOLDING THE BUS MUTEX.
  // DON'T ADD ANY TERMINAL I/O TO THIS FUNCTION (IT WILL SLOW THINGS
  // DOWN TOO MUCH).

  int32_t len = 1;
  int i;
  int32_t err;
  
#ifdef PEAK_CAN
  TPCANMsg msg;
  int pendread;
  int pendwrite=1;

  msg.ID = msgid;
  msg.MSGTYPE = MSGTYPE_STANDARD;
  msg.LEN = msglen & 0x0F;
  for (i=0; i<msglen; ++i) {
    msg.DATA[i] = msgdata[i];
  }
  
  if (block) {
    err = CAN_Write(handle,&msg);
  } else {
    err = LINUX_CAN_Extended_Status(handle, &pendread, &pendwrite);
    if ((err != CAN_ERR_OK) && (err != CAN_ERR_QRCVEMPTY)) {
      //      ROS_WARN("CANbus status is 0x%x",err);
    }
    err = CAN_Write(handle,&msg);
  }
  if (err != CAN_ERR_OK) {
    snprintf(last_error,200,"canWrite failed: 0x%x",err);
    return OW_FAILURE;
  }

  return OW_SUCCESS;
#endif // PEAK_CAN


#ifdef ESD_CAN
  CMSG cmsg;

  cmsg.id = msgid;
  cmsg.len = (uint8_t)(msglen & 0x0F);
  for(i=0; i<msglen; i++)
    cmsg.data[i] = msgdata[i];
  
  if(block == true){
    if( (err=canWrite(handle, &cmsg, &len, NULL)) != NTCAN_SUCCESS){
      snprintf(last_error,200,"canWrite failed: 0x%x",err);
      return OW_FAILURE;
    }
  }
  else{
    if( (err=canSend(handle, &cmsg, &len)) != NTCAN_SUCCESS){
      snprintf(last_error,200,"canSend failed: 0x%x",err);
      return OW_FAILURE;
    }
  }
    
  return OW_SUCCESS;
#endif // ESD_CAN
}

// set safety limits (does it really work?)
// it always sets the same property!!
// 4.2 rad/s corresponds to 240deg/sec
int CANbus::limits(double jointVel, double tipVel, double elbowVel){
  int32_t conversion;
   
  // MVW 04-29-08
  if ((set_property(SAFETY_MODULE,TL1,6000,true) == OW_FAILURE) ||
      (set_property(SAFETY_MODULE,TL2,9000,true) == OW_FAILURE) ||
      (set_property(SAFETY_MODULE,VL1,(int32_t)(2*0x1000),true) == OW_FAILURE) ||
      (set_property(SAFETY_MODULE,VL2,(int32_t)(3*0x1000),true) == OW_FAILURE)) {
      return OW_FAILURE;
  }

  int32_t voltlevel;
#ifdef SET_VOLTAGE_LIMITS
  // set appropriate high-voltage levels for battery operation
  if (get_property(SAFETY_MODULE,VOLTH1,&voltlevel) == OW_FAILURE) {
      ROS_ERROR("CANbus::limits failed to get previous high voltage warning level.");
      return OW_FAILURE;
  }
  ROS_DEBUG_NAMED("canlimits","VOLTH1 was %d, changing to 54",voltlevel);
  if (set_property(SAFETY_MODULE,VOLTH1,54,true) == OW_FAILURE) {
      ROS_ERROR("CANbus::limits: set_prop failed");
  }

  if (get_property(SAFETY_MODULE,VOLTH2,&voltlevel) == OW_FAILURE) {
      ROS_ERROR("CANbus::limits failed to get previous high voltage warning level.");
      return OW_FAILURE;
  }
  ROS_DEBUG_NAMED("canlimits","VOLTH1 was %d, changing to 57",voltlevel);
  if (set_property(SAFETY_MODULE,VOLTH2,57,true) == OW_FAILURE) {
      ROS_ERROR("CANbus::limits: set_prop failed");
  }
#endif




  return OW_SUCCESS;
  
#ifdef OLD_VEL_LIMITS
  if(0<jointVel && jointVel<7){           // If the vel (rad/s) is reasonable
    conversion = (int32_t)(jointVel*0x1000); // Convert to Q4.12 rad/s
    if(set_property(SAFETY_MODULE, VL2, conversion, true) == OW_FAILURE){
      ROS_ERROR("WAM::set_limits: set_prop failed.");
      return OW_FAILURE;
    }
  }
   
  if(0<tipVel && tipVel<7){               // If the vel (m/s) is reasonable
    conversion = (int32_t)(tipVel*0x1000);   // Convert to Q4.12 rad/s
    if(set_property(SAFETY_MODULE, VL2, conversion, true) == OW_FAILURE){
      ROS_ERROR("WAM::set_limits: set_prop failed.");
      return OW_FAILURE;
    }
  }
   
  if(0<elbowVel && elbowVel<7){           // If the vel (m/s) is reasonable
    conversion = (int32_t)(elbowVel*0x1000); // Convert to Q4.12 rad/s
    if(set_property(SAFETY_MODULE, VL2, conversion, true) == OW_FAILURE){
      ROS_ERROR("WAM::set_limits: set_prop failed.");
      return OW_FAILURE;
    }
  }
  return OW_SUCCESS;
#endif // OLD_VEL_LIMITS
}

int CANbus::run(){
  int r;
  pthread_mutex_lock(&runmutex);
  r = bus_run;
  pthread_mutex_unlock(&runmutex);
  return r;
}


void CANbus::printpos(){
  for(int p=1; p<=npucks; p++)
    ROS_DEBUG("%f",pos[p]);
}


int32_t CANbus::get_puck_state() {
  int32_t pstate;
  pthread_mutex_lock(&statemutex);
  pstate = puck_state;
  pthread_mutex_unlock(&statemutex);
  return pstate;
}

void CANbus::set_puck_state() {
  int32_t puck1_state;
  static int count=0;
  static double timesum = 0.0f;
#ifdef RT_STATS
  RTIME t1 = rt_timer_ticks2ns(rt_timer_read());
#endif
  if (get_property(1,MODE,&puck1_state) == OW_FAILURE) {
    ROS_WARN("Failure while trying to get MODE of puck #1");
    puck1_state = -1;
  }

#ifdef RT_STATS
  RTIME t2 = rt_timer_ticks2ns(rt_timer_read());
  timesum += (t2-t1)/1e6;
  if (++count == 100) {
    stats.cansetpuckstate_time = timesum/100.0;
    count=0;
    timesum=0.0;
  }
#endif

  pthread_mutex_lock(&statemutex);
  puck_state = puck1_state;
  pthread_mutex_unlock(&statemutex);
}


#ifdef BH280

int CANbus::hand_activate() {
  for (int32_t nodeid=11; nodeid<15; ++nodeid) {
    if (wake_puck(nodeid) != OW_SUCCESS) {
      ROS_WARN_NAMED("can_bh280","Could not wake hand puck %d",nodeid);
      return OW_FAILURE;
    }
    ROS_DEBUG_NAMED("can_bh280","Woke up hand puck %d", nodeid);
  }
  return OW_SUCCESS;
}

int CANbus::hand_set_state() {
  // If we were already stationary, assume we're still
  // stationary.
  if (handstate != HANDSTATE_MOVING) {
    return OW_SUCCESS;
  }
  
#ifdef RT_STATS
  static double timesum=0.0f;
  static int count=0;
  RTIME t1 = rt_timer_ticks2ns(rt_timer_read());
#endif

  // otherwise, check one finger for movement
  int32_t mode;
  if (get_property(first_moving_finger,MODE,&mode) != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280",
		   "Failed to get MODE from hand puck %d",first_moving_finger);
    handstate = HANDSTATE_UNINIT;
    return OW_FAILURE;
  }

#ifdef RT_STATS
  RTIME t2 = rt_timer_ticks2ns(rt_timer_read());
  timesum += (t2-t1)/1e6;
  if (++count == 100) {
    stats.cansethandstate_time = timesum/100.0;
    count=0;
    timesum=0.0;
  }
#endif

  // first three motors are stationary if they have returned to MODE_IDLE
  if ((first_moving_finger < 14) && (mode != MODE_IDLE)) {
    handstate = HANDSTATE_MOVING;
    return OW_SUCCESS;
  }
  // spread motor is stationary if it's returned to MODE_PID
  if ((first_moving_finger == 14) && (mode != MODE_PID)) {
    handstate = HANDSTATE_MOVING;
    return OW_SUCCESS;
  }
  // this finger was stationary, so next time check the next one
  ++first_moving_finger;
  if (first_moving_finger > 14) {
    // we've checked them all; we're done!
    handstate=HANDSTATE_DONE;
    first_moving_finger=11;
  }
  return OW_SUCCESS;
}

int CANbus::finger_reset(int32_t nodeid) {
  // send the HI to this finger
  ROS_DEBUG_NAMED("can_bh280", "Sending HI");
  if (set_property(nodeid,CMD,13) != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280","Error sending HI to hand puck %d",nodeid);
    return OW_FAILURE;
  }
  // give the finger a little time to work
  usleep(250000); // 0.25 secs
  // now wait for the change in mode
  ROS_DEBUG_NAMED("can_bh280", "Waiting for MODE change to IDLE");
  int32_t mode;
  if (get_property(nodeid,MODE,&mode) != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280","Could not get MODE from hand puck %d; still waiting", nodeid);
  }
  unsigned int count = 80; // 4 seconds max
  while ((mode != PUCK_IDLE) && (--count)) {
    usleep(50000); // 50ms
    if (get_property(nodeid,MODE,&mode) != OW_SUCCESS) {
      ROS_WARN_NAMED("can_bh280","Could not get MODE from hand puck %d; still waiting", nodeid);
    }
  }
  if (mode != PUCK_IDLE) {
    ROS_WARN_NAMED("can_bh280","Finger puck %d didn't finish HI",nodeid);
    return OW_FAILURE;
  }
  ROS_DEBUG_NAMED("can_bh280", "Finger reset");
  usleep(1000000);
  pthread_mutex_lock(&busmutex);
  int32_t msgid, msglen;
  unsigned char msg[8];
  int clearcount=0;
  while (read(&msgid, msg, &msglen, false) == OW_SUCCESS) {
    clearcount++;
  }
  pthread_mutex_unlock(&busmutex);
  ROS_DEBUG("Cleared %d old CAN messages",clearcount);

  return OW_SUCCESS;
}

int CANbus::hand_reset() {
  // Reset Strategy (from Barrett):
  //    Repeat 3 times{
  //       Open F1
  //       Open F2
  //       Open F3
  //    }
  //    Open F4

  for (unsigned int attempts =0; attempts < 3; ++attempts) {
    // F1-F3
    for (int32_t nodeid=11; nodeid<14; ++nodeid) {
      ROS_DEBUG_NAMED("can_bh280", "Resetting finger puck %d", nodeid-10);
      if ((finger_reset(nodeid) != OW_SUCCESS) &&
	  (attempts == 2)) {
	ROS_WARN_NAMED("can_bh280","Failed to reset finger puck %d",nodeid-10);
	handstate = HANDSTATE_UNINIT;
	return OW_FAILURE;
      }
    }
  }
  // F4
  ROS_DEBUG_NAMED("can_bh280", "Resetting finger puck 4");
  if (finger_reset(14) != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280","Failed to reset finger puck 4");
    handstate = HANDSTATE_UNINIT;
    return OW_FAILURE;
  }

  // Set the torque stop value to 50 to reduce heating on stall
  for (int32_t nodeid=11; nodeid<=13; ++nodeid) {
    set_property(nodeid,TSTOP,50);
    usleep(100);
    set_property(nodeid,HOLD,0);
    usleep(100);
  }
  set_property(14,TSTOP,100);
  usleep(100);
  set_property(14,HOLD,1);
  usleep(100);
  for (int32_t nodeid=11; nodeid<=14; ++nodeid) {
    int32_t value;
    get_property(nodeid,HOLD,&value);
    ROS_INFO_NAMED("can_bh280","Puck %d HOLD is %d",nodeid,value);
    get_property(nodeid,GRPA,&value);
    ROS_INFO_NAMED("can_bh280","Puck %d GRPA is %d",nodeid,value);
    get_property(nodeid,GRPB,&value);
    ROS_INFO_NAMED("can_bh280","Puck %d GRPB is %d",nodeid,value);
    get_property(nodeid,GRPC,&value);
    ROS_INFO_NAMED("can_bh280","Puck %d GRPC is %d",nodeid,value);
  }
  handstate = HANDSTATE_DONE;
  return OW_SUCCESS;
}

int CANbus::hand_move(double p1, double p2, double p3, double p4) {
  /*  set_property(11,DP,finger_radians_to_encoder(p1));
  set_property(12,DP,finger_radians_to_encoder(p2));
  set_property(13,DP,finger_radians_to_encoder(p3));
  set_property(14,DP,spread_radians_to_encoder(p4));
  set_property(11,CMD,19);  // M = 19 ("move")
  set_property(12,CMD,19);
  set_property(13,CMD,19);
  set_property(14,CMD,19); */
  ROS_INFO_NAMED("bhd280", "executing hand_move");

  set_property(11,E,finger_radians_to_encoder(p1));
  set_property(12,E,finger_radians_to_encoder(p2));
  set_property(13,E,finger_radians_to_encoder(p3));
  set_property(14,E,spread_radians_to_encoder(p4));
  set_property(11,MODE,5);  // 5 = trapezoid
  set_property(12,MODE,5);
  set_property(13,MODE,5);
  set_property(14,MODE,5);
  first_moving_finger=11;
  handstate = HANDSTATE_MOVING;
  ROS_INFO_NAMED("bhd280", "done hand_move");
  return OW_SUCCESS;
}

int CANbus::hand_velocity(double v1, double v2, double v3, double v4) {
  ROS_INFO_NAMED("bhd280", "executing hand_velocity");
  set_property(11,V,finger_radians_to_encoder(v1)/1000.0);
  set_property(12,V,finger_radians_to_encoder(v2)/1000.0);
  set_property(13,V,finger_radians_to_encoder(v3)/1000.0);
  set_property(14,V,finger_radians_to_encoder(v4)/1000.0);
  
  set_property(11,MODE,MODE_VELOCITY);
  set_property(12,MODE,MODE_VELOCITY);
  set_property(13,MODE,MODE_VELOCITY);
  set_property(14,MODE,MODE_VELOCITY);
  first_moving_finger=11;
  handstate = HANDSTATE_MOVING;
  ROS_INFO_NAMED("bhd280", "done executing hand_velocity");
  return OW_SUCCESS;
}

int CANbus::hand_relax() {
  set_property(11,MODE,PUCK_IDLE);
  set_property(12,MODE,PUCK_IDLE);
  set_property(13,MODE,PUCK_IDLE);
  set_property(14,MODE,PUCK_IDLE);
  handstate = HANDSTATE_DONE;
  return OW_SUCCESS;
}

int CANbus::hand_get_positions(double &p1, double &p2, double &p3, double &p4) {
  pthread_mutex_lock(&handmutex);
  p1 = finger_encoder_to_radians(hand_positions[1]);
  p2 = finger_encoder_to_radians(hand_positions[2]);
  p3 = finger_encoder_to_radians(hand_positions[3]);
  p4 = spread_encoder_to_radians(hand_positions[4]);
  pthread_mutex_unlock(&handmutex);
  return OW_SUCCESS;
}
 
int CANbus::hand_get_state(int32_t &state) {
  state=handstate;
  return OW_SUCCESS;
}

double CANbus::finger_encoder_to_radians(int32_t enc) {
  // encoder range: 0 to -200,000
  // degree range: 0 to 140
  return  ((double)enc / -200000.0) * 140.0 * 3.1416/180.0;
}

int32_t CANbus::finger_radians_to_encoder(double radians) {
  return(radians * 180.0/3.1416 / 140.0 * -200000.0);
}

double CANbus::spread_encoder_to_radians(int32_t enc) {
  // encoder range: 0 to -35950
  // degree range: 0 to 180
  return ((double)enc / -35950.0) * 180.0 * 3.1416/180.0;
}

int32_t CANbus::spread_radians_to_encoder(double radians) {
  return(radians * 180.0/3.1416 / 180.0 * -35950.0);
}

#endif // BH280


void CANstats::rosprint() const {
#ifdef RT_STATS
  ROS_DEBUG_NAMED("times","CANbus::send %2.1fms per group (2 groups)",
		  cansend_time);
  ROS_DEBUG_NAMED("times","CANbus::read: send=%2.1fms, read=%2.1fms",
		  canread_sendtime, canread_readtime);
  ROS_DEBUG_NAMED("times","CANbus::set_puck_state: %2.2fms",
  		  cansetpuckstate_time);
  ROS_DEBUG_NAMED("times","CANbus::set_hand_state: %2.2fms",
  		  cansethandstate_time);
#endif
}
  

void CANbus::initPropertyDefs(int firmwareVersion){
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
     VERS = i++; // 0
      ROLE = i++; /* P=PRODUCT, R=ROLE: XXXX PPPP XXXX RRRR */
      SN = i++;
      ID = i++;
      ERROR = i++;
      STAT = i++;
      ADDR = i++;
      VALUE = i++;
      MODE = i++;
      TEMP = i++;
      PTEMP = i++; // 10
      OTEMP = i++;
      BAUD = i++;
      _LOCK = i++;
      DIG0 = i++;
      DIG1 = i++;
      FET0 = i++;
      FET1 = i++;
      ANA0 = i++;
      ANA1 = i++;
      THERM = i++; // 20
      VBUS = i++;
      IMOTOR = i++;
      VLOGIC = i++;
      ILOGIC = i++;
      SG = i++;
      GRPA = i++;
      GRPB = i++;
      GRPC = i++;
      CMD = i++; /* For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T */
      SAVE = i++; // 30
      LOAD = i++;
      DEF = i++;
      FIND = i++;
      X0 = i++;
      X1 = i++;
      X2 = i++;
      X3 = i++;
      X4 = i++;
      X5 = i++;
      X6 = i++; // 40
      X7 = i++;
      
      COMMON_END = i++; // 42
   
   /* Safety */
      i = COMMON_END; 
      ZERO = i++; // 42
      PEN = i++;
      SAFE = i++;
      VL1 = i++;
      VL2 = i++;
      TL1 = i++;
      TL2 = i++;
      VOLTL1 = i++;
      VOLTL2 = i++; // 50
      VOLTH1 = i++;
      VOLTH2 = i++;
      PWR = i++;
      MAXPWR = i++;
      IFAULT = i++;
      VNOM = i++;
      
      SAFETY_END = i++;
   
   /* Tater */
      i = COMMON_END;
      T = i++;  // 42
      MT = i++;
      V = i++;
      MV = i++;
      MCV = i++;
      MOV = i++;
      P = i++; /* 32-Bit Present Position */
      P2 = i++;
      DP = i++; // 50  /* 32-Bit Default Position */
      DP2 = i++;
      E = i++; /* 32-Bit Endpoint */
      E2 = i++;
      OT = i++; /* 32-Bit Open Target */
      OT2 = i++;
      CT = i++; /* 32-Bit Close Target */
      CT2 = i++;
      M = i++; /* 32-Bit Move command for CAN*/
      M2 = i++;
      _DS = i++; // 60
      MOFST = i++;
      IOFST = i++;
      UPSECS = i++;
      OD = i++;
      MDS = i++;
      MECH = i++; /* 32-Bit */
      MECH2 = i++;
      CTS = i++; /* 32-Bit */
      CTS2 = i++;
      PIDX = i++;  // 70
      HSG = i++;
      LSG = i++;
      IVEL = i++;
      IOFF = i++; /* 32-Bit */
      IOFF2 = i++;
      MPE = i++;
      EN = i++;
      TSTOP = i++;
      KP = i++;
      KD = i++;  // 80
      KI = i++;
      ACCEL = i++;
      TENST = i++;
      TENSO = i++;
      JIDX = i++;
      IPNM = i++;
      HALLS = i++;
      HALLH = i++; /* 32-Bit */
      HALLH2 = i++;
      POLES = i++;  // 90
      IKP = i++;
      IKI = i++;
      IKCOR = i++;
      HOLD = i++;
      TIE = i++;
      ECMAX = i++;
      ECMIN = i++;
      LFLAGS = i++;
      LCTC = i++;
      LCVC = i++;  // 100
      
      PROP_END = i++;
      
      AP = P; // Handle parameter name change
      TENSION = FET1;
   }
}
 
