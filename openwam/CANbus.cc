// #define IDF_DEBUG

#include "CANbus.hh"
#include "CANdefs.hh"
#include <ros/ros.h>
#include <native/task.h>

#define PUCK_IDLE 0 

CANbus::CANbus(int bus_id, int num_pucks) : 
  puck_state(-1),id(bus_id),trq(NULL),
  pos(NULL), pucks(NULL),npucks(num_pucks),
  simulation(false)
{
  //  pthread_mutex_init(&busmutex, NULL);
  pthread_mutex_init(&trqmutex, NULL);
  pthread_mutex_init(&posmutex, NULL);
  pthread_mutex_init(&runmutex, NULL);
  pthread_mutex_init(&statemutex, NULL);

  pucks = new Puck[npucks+1];
  trq = new long[npucks+1];
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
  //  pthread_mutex_lock(&busmutex);
   
#ifdef PEAK
  long  retvalue;

  handle = CAN_Open(HW_ISA_SJA, 0x300, 7);
  //handle = CAN_Open(HW_ISA_SJA, 0x320, 5);
  
	if (!handle)
	{
		ROS_ERROR("CANbus::open(): CAN_Open(): cannot open device");
			  //		pthread_mutex_unlock(&busmutex);
		return OW_FAILURE;
	}
     
	// Clear Status
	CAN_Status(handle);

	retvalue = CAN_Init(handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
	if (retvalue)
	{
	  ROS_ERROR("CANbus::open(): CAN_Init(): failed with %d",retvalue);
	  //		pthread_mutex_unlock(&busmutex);
		return OW_FAILURE;		
	}
	//	pthread_mutex_unlock(&busmutex);

	CAN_ResetFilter(handle);
	CAN_MsgFilter(handle, 0x0000, 0x053F, MSGTYPE_STANDARD);
	
#else   

	if(canOpen(id, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, 
		   TX_TIMEOUT, RX_TIMEOUT, &handle) != NTCAN_SUCCESS){  
    ROS_ERROR("CANbus::open: canOpen failed.");
    //    pthread_mutex_unlock(&busmutex);
    return OW_FAILURE;
  }   
  
  // 0 = 1Mbps, 2 = 500kbps, 4 = 250kbps
  if(canSetBaudrate(handle, 0) != NTCAN_SUCCESS){
    ROS_ERROR("CANbus::opent: canSetBaudrate failed.");
    //    pthread_mutex_unlock(&busmutex);
    return OW_FAILURE;
  }
  //  pthread_mutex_unlock(&busmutex);
    
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
#endif

  // Set the minimum required property values
  // (we'll set the rest once we know the pucks' firmware version
  VERS = 0;
  STAT = 5;
  PROP_END = 10;
   
  return OW_SUCCESS;
}

int CANbus::check(){
  long nodes[NUM_NODES+1];//, value;
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
            ROS_DEBUG("Node (id, status): (%d,%s)",n,(nodes[n] == STATUS_RESET)?"reset":"running");
          }
	 
    if(nodes[n]!=STATUS_OFFLINE && n!=SAFETY_MODULE) 
      online_pucks++;
  }

  if(online_pucks==0){
    ROS_INFO_NAMED("cancheck","The wam appears to be turned off");
    return OW_FAILURE;
  }
  if(online_pucks < npucks){
    ROS_INFO_NAMED("cancheck","Bus has %d pucks. We expected %d",online_pucks,npucks);
    return OW_FAILURE;
  }  
  if(nodes[SAFETY_MODULE] == STATUS_OFFLINE){
    ROS_DEBUG_NAMED("cancheck","CANbus::check: safety module is offline");
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
    ROS_ERROR_NAMED("cancheck","Some of the pucks must have reset unexpectedly");
    ROS_ERROR_NAMED("cancheck","ALL pucks should be in either running or reset state");
    return OW_FAILURE;
  }
      
  ROS_WARN("  Initializing pucks...");
  for(int p=1; p<=npucks; p++){

    ROS_DEBUG_NAMED("cancheck","Checking puck %d",pucks[p].id());
	 
    if(nodes[ pucks[p].id() ] == STATUS_RESET){
      ROS_DEBUG_NAMED("cancheck"," (reset) ->Waking up the puck...");
      if(wake_puck(p) == OW_FAILURE){
	ROS_DEBUG_NAMED("cancheck","CANbus::check: wake_puck failed.");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");
      usleep(10000);
      
      ROS_DEBUG_NAMED("cancheck","Setting PID values");
      if(set_property(pucks[p].id(),PIDX, pucks[p].order(),true)==OW_FAILURE){
	ROS_DEBUG_NAMED("cancheck","CANbus::check: set_prop failed (PID)");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");

      ROS_DEBUG_NAMED("cancheck","Setting max torque...");
      int max_torque;
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
	ROS_DEBUG_NAMED("cancheck","CANbus::check: set_prop failed (max torque)");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");
    }
    else{
      ROS_DEBUG_NAMED("cancheck"," (running)");
      ROS_DEBUG_NAMED("cancheck","setting idle mode...");
      // this was set to CONTROLLER_IDLE originally
      if(set_property(pucks[p].id(), MODE, PUCK_IDLE, true) == OW_FAILURE){
	ROS_DEBUG_NAMED("cancheck","CANbus::check: set_prop failed (MODE)");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");
    }
  }
  ROS_WARN("done.");
  return OW_SUCCESS;
}

void CANbus::dump(){
  ROS_DEBUG("Printing bus information.");
  for(int g=GROUP_ID_MIN; g<=GROUP_ID_MAX; g++){
    //    if(groups[g].id() != GROUP_INVALID)
      // ROS_DEBUG_STREAM(groups[g]);
  }
}
   
int CANbus::allow_message(int id, int mask){
  
#ifdef PEAK
  
	//Allows all messages
	CAN_ResetFilter(handle);
	
#else
	
  int i;
  //  pthread_mutex_lock(&busmutex);
  for(i=0; i<2048; i++){
      if((i & ~mask) == id){
          if(canIdAdd(handle, i) != NTCAN_SUCCESS){
              ROS_ERROR("CANbus::allow_message: canIdAdd failed.");
	      //              pthread_mutex_unlock(&busmutex);
              return OW_FAILURE;
          }
      }
  }
  //  pthread_mutex_unlock(&busmutex);
  
#endif

  return OW_SUCCESS;
}
 
int CANbus::wake_puck(int p){
    if(set_property(pucks[p].id(), STAT, STATUS_READY, false) == OW_FAILURE){
        // ROS_ERROR("CANbus::wake_puck: set_prop failed.");
        return OW_FAILURE;
    }
    usleep(500000); // 500ms                   // Wait 10ms for puck to initialize    
    return OW_SUCCESS;
}

// Got to make sure that nodes is NUM_NODES+1 long !!!
int CANbus::status(long* nodes){
  unsigned char msg[8];
  int msgid, msglen, nodeid, property;
  int firstFound = 0;
  long fw_vers;

  ROS_INFO_NAMED("canstatus","Probing for nodes on the CAN bus");

  for(int n=NODE_MIN; n<=NODE_MAX; n++){
    msg[0] = (unsigned char) 5;   // STAT = 5 for all firmware versions
    nodes[n] = STATUS_OFFLINE;      // Initialize node as offline

    //    pthread_mutex_lock(&busmutex);
    if(send(NODE2ADDR(n), msg, 1, true) == OW_FAILURE){
      ROS_DEBUG("CANbus::status: send failed");
      //      pthread_mutex_unlock(&busmutex);
      return OW_FAILURE;
    }
    ROS_DEBUG_NAMED("canstatus","Sent probe message to puck %d id %d",n,NODE2ADDR(n));

    usleep(1000);
    if(read(&msgid, msg, &msglen, false) == OW_FAILURE){
      //      pthread_mutex_unlock(&busmutex);
      ROS_DEBUG_NAMED("canstatus","CANbus::status: node %d didn't answer",NODE2ADDR(n));
    }
    else{
      //      pthread_mutex_unlock(&busmutex);
      if(parse(msgid, msg, msglen,&nodeid,&property,&nodes[n])==OW_FAILURE){
	ROS_DEBUG_NAMED("canstatus","CANbus::status: parse failed");
          return OW_FAILURE;
      } else {
          // parsed ok
	ROS_DEBUG_NAMED("canstatus","CANbus::status: parsed response from node %d",NODE2ADDR(n));
	if (!firstFound) {
	  if ((wake_puck(n) == OW_SUCCESS) &&
	      (get_property(NODE2ADDR(n), 0, &fw_vers) == OW_SUCCESS)){
	    ROS_DEBUG_NAMED("canstatus","CANbus::status: puck %d firmware version %ld",n,fw_vers);
	    initPropertyDefs(fw_vers);
	    firstFound = 1;
	  }
	  else {
	    ROS_DEBUG_NAMED("canstatus","CANbus::status: unable to get firmware vers from puck %d",n);
	  }
	}
      }
    }
    
  }
  if (!firstFound) {
      // we never got a firmware version, so we never initialized
      // the property definitions.
    ROS_DEBUG_NAMED("canstatus","WARNING: Could not determine puck firmware version");
    ROS_DEBUG_NAMED("canstatus","CANbus::status: Unable to continue");
    return OW_FAILURE;
  }

  return OW_SUCCESS;
}

int CANbus::set_property(int32_t nid, int32_t property, long value,bool check){
  uint8_t msg[8];
  long response;
  int32_t msglen;
   
  if(compile(property, value, msg, &msglen) ==OW_FAILURE){
    ROS_ERROR("CANbus::set_property: compile failed.");
    return OW_FAILURE;
  }
  msg[0] |= (uint8_t)0x80; // Set the 'Set' bit

  //  pthread_mutex_lock(&busmutex);
  if(send(NODE2ADDR(nid), msg, msglen, true) == OW_FAILURE){
      // ROS_ERROR("CANbus::set_property: send failed.");
    //    pthread_mutex_unlock(&busmutex);
    return OW_FAILURE;
  }
  //  pthread_mutex_unlock(&busmutex);
   
  if(check){
    // Get the new value of the property
    if(get_property(nid, property, &response) == OW_FAILURE){
        // ROS_ERROR("CANbus::set_property: get_prop failed.");
      return OW_FAILURE;
    }

    // Compare response to value
    if(response != value){
      ROS_ERROR("CANbus::set_property: response and value mismatch.");
      ROS_ERROR("Puck %d, Property %d Set Value %d Response Value %d",nid,property,value,response);
      ROS_ERROR("Property P=%d, AP=%d",P,AP);
          
      return OW_FAILURE;
    }
  }   

  return OW_SUCCESS;
}

int CANbus::get_property(int32_t nid, int32_t property, long* value){
  uint8_t msg[8];
  int32_t msgid, msglen, nodeid, prop;

  msg[0] = (uint8_t)property;   

  //  pthread_mutex_lock(&busmutex);
  if(send(NODE2ADDR(nid), msg, 1, true) == OW_FAILURE){
    ROS_ERROR("CANbus::get_prop: send failed.");
    //    pthread_mutex_unlock(&busmutex);
    return OW_FAILURE;
  }
  if(read(&msgid, msg, &msglen, true) == OW_FAILURE){
    ROS_ERROR("CANbus::get_prop: read failed.");
    //    pthread_mutex_unlock(&busmutex);
    return OW_FAILURE;
  }
  //  pthread_mutex_unlock(&busmutex);
   
  // Parse the reply
  if(parse(msgid, msg, msglen, &nodeid, &prop, value) == OW_FAILURE){
    ROS_ERROR("CANbus::get_prop: parse failed.");
    return OW_FAILURE;
  }
      
  // Check that the ids and properties match
  if(nodeid!=nid || prop!=property){
    ROS_ERROR("CANbus::get_prop: id or property mismatch.");
    return OW_FAILURE;
  }   
  return OW_SUCCESS;
}

int CANbus::send_torques(long* torques){
  pthread_mutex_trylock(&trqmutex);
  for(int p=1; p<=npucks; p++)
    trq[p] = torques[p];
  pthread_mutex_unlock(&trqmutex);

  return OW_SUCCESS;
}

int CANbus::send_torques(){
  uint8_t msg[8];
  int torques[NUM_ORDERS+1];
  Puck* puck;

  static double sendtime=0.0f;
  static unsigned int sendcount=0;

  // make a quick copy so that we can release the mutex
  static long *mytorqs = (long *) malloc(NUM_NODES * sizeof(long));

  pthread_mutex_lock(&trqmutex);
  memcpy(mytorqs,trq,NUM_NODES*sizeof(long));
  pthread_mutex_unlock(&trqmutex);

  static int DEBUGCOUNT=0;
  if (++DEBUGCOUNT == 1000) {
    DEBUGCOUNT = 0;
  }

  for(int g=GROUP_ID_MIN; g<=GROUP_ID_MAX; g++){
      
      if(groups[g].id() != GROUP_INVALID){
          
          for(int p=PUCK_ORDER_MIN; p<=PUCK_ORDER_MAX; p++){
              
              puck = groups[g].puck(p); 
              if(puck){ 
                  torques[p] = clip(mytorqs[puck->motor()], 
                                    Puck::MIN_TRQ[puck->id()], 
                                    Puck::MAX_TRQ[puck->id()] );
		  if (DEBUGCOUNT==0) {
		    ROS_DEBUG_NAMED("torques","Puck %d Torque %d",puck->id(),torques[p]);
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
          
	  //          pthread_mutex_lock(&busmutex);

	  RTIME bt1 = rt_timer_ticks2ns(rt_timer_read());
          if(send(GROUPID(groups[g].id()), msg, 8, false) == OW_FAILURE){
              ROS_ERROR("CANbus::set_torques: send failed.");
	      //              pthread_mutex_unlock(&busmutex);
              return OW_FAILURE;
          }
	  //          pthread_mutex_unlock(&busmutex);

	  RTIME bt2 = rt_timer_ticks2ns(rt_timer_read());
	  sendtime += (bt2-bt1) * 1e-6; // ns to ms
	  ++sendcount;
      }

  }

  if (sendcount > 2000) {
    stats.cansend_time = sendtime/sendcount;
    sendcount=0;
    sendtime=0.0f;
  }
  
  return OW_SUCCESS;
}

int CANbus::read_torques(long* mtrq){
  uint8_t  msg[8];
  long int value;
  int32_t msgid, msglen, nodeid, property;

  // Compile the packet
  msg[0] = (uint8_t)TORQ;

  //  pthread_mutex_lock(&busmutex);
  if(send(GROUPID(0), msg, 1, true) == OW_FAILURE){
    ROS_ERROR("CANbus::get_positions: send failed.");;
    //    pthread_mutex_unlock(&busmutex);
    return OW_FAILURE;
  }
  for(int p=1; p<=npucks; ){
    if(read(&msgid, msg, &msglen, true) == OW_FAILURE){
      ROS_ERROR("CANbus::get_positions: read failed.");
      //      pthread_mutex_unlock(&busmutex);
      return OW_FAILURE;
    }
    if(parse(msgid, msg, msglen, &nodeid, &property, &value) == OW_FAILURE){
      ROS_ERROR("CANbus::get_positions: parse failed.");
      //      pthread_mutex_unlock(&busmutex);
      return OW_FAILURE;
    }
    if(property == TORQ){
      trq[nodeid] = value;
      p++;
    }
  }
  //  pthread_mutex_unlock(&busmutex);

  return OW_SUCCESS;
}

int CANbus::read_positions(double* positions){
  //  if(read_positions() == OW_FAILURE){
  //    ROS_ERROR("CANbus::read_positions: read_positions failed.");
  //    return OW_FAILURE;
  //  }

  pthread_mutex_lock(&posmutex);
  for(int p=1; p<=npucks; p++) {
    positions[p] = pos[p];
  }
  pthread_mutex_unlock(&posmutex);

  return OW_SUCCESS;
}

int CANbus::read_positions(){
  uint8_t  msg[8];
  long data[NUM_NODES+1], value;
  int32_t msgid, msglen, property, nodeid;
  static double sendtime=0.0f;
  static double readtime=0.0f;
  static unsigned int loopcount=0;

  // Compile the packet
  msg[0] = (uint8_t)AP;

  //  pthread_mutex_lock(&busmutex);

  RTIME bt1 = rt_timer_ticks2ns(rt_timer_read());
  if(send(GROUPID(0), msg, 1, false) == OW_FAILURE){
    ROS_ERROR("CANbus::get_positions: send failed.");
    //    pthread_mutex_unlock(&busmutex);
    return OW_FAILURE;
  }
  RTIME bt2 = rt_timer_ticks2ns(rt_timer_read());
  sendtime += (bt2-bt1) * 1e-6; // ns to ms

  for(int p=1; p<=npucks; ){

    if(read(&msgid, msg, &msglen, true) == OW_FAILURE){
      ROS_ERROR("CANbus::get_positions: read failed.");
      //      pthread_mutex_unlock(&busmutex);
      return OW_FAILURE;
    }

    if(parse(msgid, msg, msglen, &nodeid, &property, &value) == OW_FAILURE){
      ROS_ERROR("CANbus::get_positions: parse failed.");
      //      pthread_mutex_unlock(&busmutex);
      return OW_FAILURE;
    }
    if(property == AP){
      data[nodeid] = value;
      p++;
    }
  }
  //  pthread_mutex_unlock(&busmutex);

  bt1 = rt_timer_ticks2ns(rt_timer_read());
  readtime += (bt1-bt2) * 1e-6; // ns to ms

  // convert the results
  pthread_mutex_lock(&posmutex);
  for(int p=1; p<=npucks; p++)
    pos[ pucks[p].motor() ] = 2.0*M_PI*( (double) data[ pucks[p].id() ] )/ 
                                       ( (double) pucks[p].CPR() );
  pthread_mutex_unlock(&posmutex);

  if (++loopcount == 1000) {
    stats.canread_sendtime=sendtime/1000.0;
    stats.canread_readtime=readtime/1000.0;
    sendtime=readtime=0.0f;
    loopcount=0;
  }

  return OW_SUCCESS;
}

int CANbus::send_positions(double* mpos){
  long position[npucks+1];

  for(int p=1; p<=npucks; p++){
    position[p] = (long)floor( mpos[ pucks[p].motor() ]*
			    pucks[p].CPR()/(2.0*M_PI) );
  }   
  return send_AP(position);
}

int CANbus::send_AP(long* apval){

  //
  // WARNING
  // If the next call crashes we should put back the SAFETY_MODULE back on
  //
  if(set_property(SAFETY_MODULE, IFAULT, 8, true) == OW_FAILURE){
    ROS_ERROR("CANbus::send_positions: set_property(1) failed.");
    return OW_FAILURE;
  }

#ifdef IDF_DEBUG
    printf("CANbus.cc: Setting positions for %d pucks\n",npucks);
#endif

  for(int p=1; p<=npucks; p++){
    if(set_property(pucks[p].id(), AP, apval[p], false) == OW_FAILURE){
      ROS_ERROR("CANbus::send_positions: set_property failed.");
      return OW_FAILURE;
    }
  }

#ifdef IDF_DEBUG
  sleep(2);
  printf("CANbus.cc: positions set\n");
#endif

  // let the safety module see the new positions
  read_positions();
   
#ifdef IDF_DEBUG
  sleep(2);
  printf("CANbus.cc: New positions read from pucks\n");
#endif

  // start monitoring tip velocity again
  if(set_property(SAFETY_MODULE, ZERO, 1, true) == OW_FAILURE){
    ROS_ERROR("CANbus::send_positions: set_property(2) failed.");
    return OW_FAILURE;
  }

#ifdef IDF_DEBUG
  printf("CANbus.cc: Velocity checking re-enabled\n");
#endif

  return OW_SUCCESS;
}

int CANbus::parse(int32_t msgid, uint8_t* msg, int32_t msglen,
		  int32_t* nodeid, int32_t* property, long* value){
  int i;
  int dataHeader;

  *nodeid = ADDR2NODE(msgid);
  if(*nodeid == -1){
    ROS_ERROR("CANbus::parse: invalid node id %d",*nodeid);
    return OW_FAILURE;
  }
   
  dataHeader = ((msg[0] >> 6) & 0x0002) | ((msgid & 0x041F) == 0x0403);

  switch (dataHeader){
      
  case 3:  // Data is a packed 22-bit position, SET 
    *value = 0x00000000;
    *value |= ( (long)msg[0] << 16) & 0x003F0000;
    *value |= ( (long)msg[1] << 8 ) & 0x0000FF00;
    *value |= ( (long)msg[2] )      & 0x000000FF;
      
    if(*value & 0x00200000) // If negative 
      *value |= 0xFFC00000; // Sign-extend 
      
    *property = AP;

#ifdef JOINT_ENCODERS
      jointPosition[*nodeid] = 0;
      jointPosition[*nodeid] |= ( (long)messageData[3] << 16) & 0x003F0000;
      jointPosition[*nodeid] |= ( (long)messageData[4] << 8 ) & 0x0000FF00;
      jointPosition[*nodeid] |= ( (long)messageData[5] ) & 0x000000FF;
      
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
      *value |= ((unsigned long)msg[i + 2] << (i * 8))
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
    ROS_ERROR("CANbus::parse: Illegal message header.");
    return OW_FAILURE;
  }
  return OW_SUCCESS;
}

int CANbus::compile(int32_t property, long value, 
		    uint8_t *msg, int32_t *msglen){
  int i;
   
  // Check the property
  if(PROP_END < property){
    ROS_ERROR("CANbus::compile: invalid property.");
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

int CANbus::read(int32_t* msgid, uint8_t* msg, int32_t* msglen, bool block){
    //    ROS_ERROR("CANbus:: READ");

  
  CMSG cmsg;
  int32_t len = 1;
  int i, err;
  
  if(block){
    err = canRead(handle, &cmsg, &len, NULL);
    if (err != NTCAN_SUCCESS) {
      ROS_ERROR("CANbus::read: canRead failed: %x",err);
      return OW_FAILURE;
    }
  }
  else{
    if((err=canTake(handle, &cmsg, &len)) != NTCAN_SUCCESS){
      ROS_ERROR("CANbus::read: canTake failed: %x",err);
      return OW_FAILURE;
    }
  }

  // Just to avoid printing anoying output during status checkup
  if(len == 0){
    return OW_FAILURE;
  }
  else if(len != 1){
    ROS_ERROR("CANbus::read: received a message of length: %d",len);
    return OW_FAILURE;
  }

  *msgid = cmsg.id;
  *msglen = cmsg.len;
  for(i=0; i<*msglen; i++)
    msg[i] = cmsg.data[i];
 
  return OW_SUCCESS;
}

int CANbus::send(int32_t msgid, uint8_t* msg, int32_t msglen, bool block){


  CMSG cmsg;
  int32_t len = 1;
  int i, err;
  
  cmsg.id = msgid;
  cmsg.len = (uint8_t)(msglen & 0x0F);
  for(i=0; i<msglen; i++)
    cmsg.data[i] = msg[i];
  
  if(block == true){
    if( (err=canWrite(handle, &cmsg, &len, NULL)) != NTCAN_SUCCESS){
      // ROS_ERROR("CANbus::send: canWrite failed: %x",err);
      return OW_FAILURE;
    }
  }
  else{
    if( (err=canSend(handle, &cmsg, &len)) != NTCAN_SUCCESS){
      //      ROS_ERROR("CANbus::send: canSend failed: %x",err);
      return OW_FAILURE;
    }
  }
    
  return OW_SUCCESS;
}

// set safety limits (does it really work?)
// it always sets the same property!!
// 4.2 rad/s corresponds to 240deg/sec
int CANbus::limits(double jointVel, double tipVel, double elbowVel){
  long conversion;
   
  // MVW 04-29-08
  if ((set_property(SAFETY_MODULE,TL1,6000,true) == OW_FAILURE) ||
      (set_property(SAFETY_MODULE,TL2,9000,true) == OW_FAILURE) ||
      (set_property(SAFETY_MODULE,VL1,(long)(2*0x1000),true) == OW_FAILURE) ||
      (set_property(SAFETY_MODULE,VL2,(long)(3*0x1000),true) == OW_FAILURE)) {
      return OW_FAILURE;
  }

  long voltlevel;
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
    conversion = (long)(jointVel*0x1000); // Convert to Q4.12 rad/s
    if(set_property(SAFETY_MODULE, VL2, conversion, true) == OW_FAILURE){
      ROS_ERROR("WAM::set_limits: set_prop failed.");
      return OW_FAILURE;
    }
  }
   
  if(0<tipVel && tipVel<7){               // If the vel (m/s) is reasonable
    conversion = (long)(tipVel*0x1000);   // Convert to Q4.12 rad/s
    if(set_property(SAFETY_MODULE, VL2, conversion, true) == OW_FAILURE){
      ROS_ERROR("WAM::set_limits: set_prop failed.");
      return OW_FAILURE;
    }
  }
   
  if(0<elbowVel && elbowVel<7){           // If the vel (m/s) is reasonable
    conversion = (long)(elbowVel*0x1000); // Convert to Q4.12 rad/s
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
    ROS_DEBUG("%d",pos[p]);
}


long CANbus::get_puck_state() {
  long pstate;
  pthread_mutex_lock(&statemutex);
  pstate = puck_state;
  pthread_mutex_unlock(&statemutex);
  return pstate;
}

void CANbus::set_puck_state() {
  long puck1_state;
  static int count=0;
  static double timesum = 0.0f;
  RTIME t1 = rt_timer_ticks2ns(rt_timer_read());
  if (get_property(1,MODE,&puck1_state) == OW_FAILURE) {
    ROS_WARN("Failure while trying to get MODE of puck #1");
    puck1_state = -1;
  }

  RTIME t2 = rt_timer_ticks2ns(rt_timer_read());
  timesum += (t2-t1)/1e6;
  if (++count == 100) {
    stats.cansetpuckstate_time = timesum/100.0;
    count=0;
    timesum=0.0;
  }

  pthread_mutex_lock(&statemutex);
  puck_state = puck1_state;
  pthread_mutex_unlock(&statemutex);
}

void CANstats::rosprint() const {
  //  ROS_DEBUG_NAMED("times","CANbus::send %2.1fms per group (2 groups)",
  //		  cansend_time);
  //  ROS_DEBUG_NAMED("times","CANbus::read: send=%2.1fms, read=%2.1fms",
  //		  canread_sendtime, canread_readtime);
  ROS_DEBUG_NAMED("times","CANbus::set_puck_state: %2.2fms",
  		  cansetpuckstate_time);
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
 
