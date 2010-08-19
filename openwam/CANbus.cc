// #define IDF_DEBUG

#include "CANbus.hh"
#include "CANdefs.hh"
#include <ros/ros.h>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>

#ifdef BH280
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif // BH280

#define PUCK_IDLE 0 
#define MODE_IDLE      0
#define MODE_TORQUE    2
#define MODE_PID       3
#define MODE_VELOCITY  4
#define MODE_TRAPEZOID 5

#define CMD_HI 13
#define CMD_M 19


CANbus::CANbus(int32_t bus_id, int num_pucks) : 
  puck_state(-1),id(bus_id),trq(NULL),
  pos(NULL), pucks(NULL),npucks(num_pucks),
  simulation(false),
#ifdef CAN_RECORD
  candata(100000),
#endif // CAN_RECORD
  unread_packets(0)
{
#ifdef BH280
  // hand_queue_mutex is used to manage access to the hand command/response queues
  mutex_init(&hand_queue_mutex);

  // hand_cmd_queue is used to prevent trouble with multiple ROS service calls occurring at once.  it
  // makes sure that the response you get from the queue corresponds to the command you sent.
  mutex_init(&hand_cmd_mutex);

  handstate = HANDSTATE_UNINIT;
  first_moving_finger=11;
#ifdef OWD_RT
  int err = rt_pipe_create(&handpipe,"HANDPIPE",P_MINOR_AUTO,0);
  if (err) {
    ROS_ERROR("Could not create RT message pipe for hand communications: %d",err);
    throw OW_FAILURE;
  }
  handpipe_fd = ::open("/proc/xenomai/registry/native/pipes/HANDPIPE",O_RDWR);
  if (handpipe_fd < 0) {
    ROS_ERROR("Could not open user-side of RT message pipe: %d",errno);
    throw OW_FAILURE;
  }
#endif // OWD_RT
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

int CANbus::init(){ 
  if(open() == OW_FAILURE){
    ROS_ERROR("CANbus::init: open failed.");
    return OW_FAILURE;
  }

  if(check() == OW_FAILURE){
    ROS_ERROR("CANbus::init: check failed.");
    return OW_FAILURE;
  }

#ifdef OWD_RT
#ifdef ESD_CAN
  //  int err = rt_intr_create(&rt_can_intr, "ESDCAN_IRQ", 12, I_PROPAGATE);
#endif
#ifdef PEAK_CAN
  //  int err = rt_intr_create(&rt_can_intr, "PEAKCAN_IRQ", 19, I_PROPAGATE);
#endif
  
  //  if (err) {
  //    ROS_ERROR("Failed to create interrupt management object: %d",err);
  //    return OW_FAILURE;
  //  }

  //  err = rt_intr_enable(&rt_can_intr);
  //  if (err) {
  //    ROS_ERROR("Failed to enable interrupt for CAN interface: %d",err);
  //    return OW_FAILURE;
  //  }
#endif // OWD_RT

  return OW_SUCCESS;
}

int CANbus::open(){ 
   
#ifdef PEAK_CAN
  DWORD  err;

  char devicename[20];
  snprintf(devicename,20,"/dev/pcan%d",id);
  handle = LINUX_CAN_Open(devicename,O_RDWR);
  
  if (!handle) {
    ROS_ERROR("CANbus::open(): CAN_Open(): cannot open device");
    throw OW_FAILURE;
  }
  
  // Clear Status
  err = CAN_Status(handle);
  ROS_DEBUG("CANbus::open(): bus status = 0x%x",err);
  
  err = CAN_Init(handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
  if (err) {
    ROS_ERROR("CANbus::open(): CAN_Init(): failed with 0x%x",err);
    return OW_FAILURE;		
  }
  
  if ((err = CAN_ResetFilter(handle))) {
    ROS_ERROR("CANbus::open(): Could not Reset Filter: 0x%x",err);
  }
  if ((err = CAN_MsgFilter(handle, 0x0000, 0x07FF, MSGTYPE_STANDARD))) {
    ROS_ERROR("CANbus::open(): Could not set Msg Filter: 0x%x",err);
  }
	
#endif // PEAK_CAN
#ifdef ESD_CAN

  if(canOpen(id, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, 
	     TX_TIMEOUT, RX_TIMEOUT, &handle) != NTCAN_SUCCESS){  
    ROS_ERROR("CANbus::open: canOpen failed.");
    return OW_FAILURE;
  }   
  
  // 0 = 1Mbps, 2 = 500kbps, 4 = 250kbps
  if(canSetBaudrate(handle, 0) != NTCAN_SUCCESS){
    ROS_ERROR("CANbus::opent: canSetBaudrate failed.");
    return OW_FAILURE;
  }
    
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

int CANbus::check(){
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
    ROS_INFO_NAMED("cancheck","Bus has %d pucks. We expected %d",online_pucks,npucks+4);
    ROS_INFO_NAMED("cancheck","Attempting to reset all pucks");
    for (int32_t node=1; node<=15; ++node) {
      if (node != 10) {
	// reset everything except the safety puck
	set_property_rt(node,STAT,STATUS_RESET,false);
      }
    }
    return OW_FAILURE;
  }
#else
  if(online_pucks < npucks){
    ROS_INFO_NAMED("cancheck","Bus has %d pucks. We expected %d",online_pucks,npucks);
    return OW_FAILURE;
  }  
#endif

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
      ROS_DEBUG_NAMED("cancheck","Waking up the puck %d",pucks[p].id());
      if(wake_puck(pucks[p].id()) == OW_FAILURE){
	ROS_WARN_NAMED("cancheck","wake_puck failed.");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");
      usleep(10000);
      
      ROS_DEBUG_NAMED("cancheck","Setting PID values");
      if(set_property_rt(pucks[p].id(),PIDX, pucks[p].order(),true,15000)==OW_FAILURE){
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
          
      if(set_property_rt(pucks[p].id(), MT, max_torque, true,15000) == OW_FAILURE){
	ROS_WARN_NAMED("cancheck","set_property failed (max torque)");
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("cancheck","OK");
    }
    else{
      ROS_DEBUG_NAMED("cancheck"," (running)");
      ROS_DEBUG_NAMED("cancheck","setting idle mode...");
      if(set_property_rt(pucks[p].id(), MODE, PUCK_IDLE, true, 10000) == OW_FAILURE){
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
  if (hand_activate(nodes) != OW_SUCCESS) {
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
   
int CANbus::allow_message(int32_t id, int32_t mask){
  
#ifdef PEAK_CAN
  DWORD err;
  if ((err = CAN_ResetFilter(handle)) ||
      (err = CAN_MsgFilter(handle, 0x0000, 0x07FF, MSGTYPE_STANDARD))) {
    ROS_WARN("CANbus::allow_message: Could not set Msg Filter: 0x%x",err);
    return OW_FAILURE;
  }
#endif // PEAK_CAN

#ifdef ESD_CAN
  int i;
  for(i=0; i<2048; i++){
    if((i & ~mask) == id){
      if(canIdAdd(handle, i) != NTCAN_SUCCESS){
	ROS_WARN("CANbus::allow_message: canIdAdd failed.");
	return OW_FAILURE;
      }
    }
  }
#endif  // ESD_CAN

  return OW_SUCCESS;
}
 
int CANbus::wake_puck(int32_t puck_id){ 
    if(set_property_rt(puck_id, STAT, STATUS_READY, false) == OW_FAILURE){
      ROS_WARN("CANbus::wake_puck: set_property failed for puck %d.",puck_id);
        return OW_FAILURE;
    }
    usleep(750000); // Wait 500ms for puck to initialize    
    return OW_SUCCESS;
}

int CANbus::clear() {
  unsigned char msg[8];
  int32_t msgid, msglen;
  
  int count(0);
  while (read_rt(&msgid, msg, &msglen, 0) == OW_SUCCESS) {
    ++count;
  }
  return count;
}


// Got to make sure that nodes is NUM_NODES+1 long !!!
int CANbus::status(int32_t* nodes){
  unsigned char msg[8];
  int32_t msgid, msglen, nodeid, property;
  int firstFound = 0;
  int32_t fw_vers;

  ROS_INFO_NAMED("canstatus","Probing for nodes on the CAN bus");


  for(int n=NODE_MIN; n<=NODE_MAX; n++){
    msg[0] = (unsigned char) 5;   // STAT = 5 for all firmware versions
    nodes[n] = STATUS_OFFLINE;      // Initialize node as offline

    if(send_rt(NODE2ADDR(n), msg, 1, 100) == OW_FAILURE){
      ROS_WARN_NAMED("canstatus","send failed: %s",last_error);
      return OW_FAILURE;
    }
    ROS_DEBUG_NAMED("canstatus","Sent probe message to puck %d id %d",n,NODE2ADDR(n));

    if(read_rt(&msgid, msg, &msglen, 40000) == OW_FAILURE){
      ROS_DEBUG_NAMED("canstatus","node %d didn't answer: %s",NODE2ADDR(n), last_error);
    }
    else{
      if(parse(msgid, msg, msglen,&nodeid,&property,&nodes[n])==OW_FAILURE){
	ROS_DEBUG_NAMED("canstatus","parse failed: %s",last_error);
	return OW_FAILURE;
      } else {
          // parsed ok
	ROS_DEBUG_NAMED("canstatus","parsed response from node %d",NODE2ADDR(n));
	if (!firstFound) {
	  ROS_DEBUG_NAMED("canstatus","trying to wake node %d",n);
	  if ((wake_puck(NODE2ADDR(n)) == OW_SUCCESS) &&
	      (get_property_rt(NODE2ADDR(n), 0, &fw_vers) == OW_SUCCESS)){
	    ROS_DEBUG_NAMED("canstatus","puck %d firmware version %d",n,fw_vers);
	    initPropertyDefs(fw_vers);
	    firstFound = 1;
	  }
	  else {
	    ROS_DEBUG_NAMED("canstatus","unable to get firmware vers from puck %d",n);
	    return OW_FAILURE;
	  }
	}
      }
    }
  }

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

 int CANbus::set_property_rt(int32_t nid, int32_t property, int32_t value,bool check, int32_t usecs){
  uint8_t msg[8];
  int32_t response;
  int32_t msglen;
   
  if(compile(property, value, msg, &msglen) ==OW_FAILURE){
    ROS_WARN("CANbus::set_property: compile failed.");
    return OW_FAILURE;
  }
  msg[0] |= (uint8_t)0x80; // Set the 'Set' bit

  if(send_rt(NODE2ADDR(nid), msg, msglen, usecs) == OW_FAILURE){
    ROS_WARN("CANbus::set_property: send failed: %s",last_error);
    return OW_FAILURE;
  }
   
  if(check){
    // Get the new value of the property
    if(get_property_rt(nid, property, &response, usecs) == OW_FAILURE){
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

 int CANbus::get_property_rt(int32_t nid, int32_t property, int32_t* value, int32_t usecs){
  uint8_t msg[8];
  int32_t msgid, msglen, nodeid, prop;
  *value=0;

  if ((property > PROP_END) || (property < 0)) {
    ROS_WARN("get_property: requested property is not valid for this puck's firmware version");
    return OW_FAILURE;
  }

  msg[0] = (uint8_t)property;   

  if(send_rt(NODE2ADDR(nid), msg, 1, usecs) == OW_FAILURE){
    ROS_WARN("CANbus::get_property: send failed: %s",last_error);
    return OW_FAILURE;
  }
 REREAD:
  if(read_rt(&msgid, msg, &msglen, usecs) == OW_FAILURE){
    ROS_WARN("CANbus::get_property: read failed: %s",last_error);
    ++unread_packets;
    return OW_FAILURE;
  }
   
  // Parse the reply
  if(parse(msgid, msg, msglen, &nodeid, &prop, value) == OW_FAILURE){
    ROS_WARN("CANbus::get_property: parse failed: %s",last_error);
    return OW_FAILURE;
  }
      
  // Check that the ids and properties match
  if(nodeid!=nid) {
    ROS_WARN("Asked for property %d from node %d but got answer from node %d, %d previously missed messages",
    	     property, nid, nodeid, unread_packets);
    if (unread_packets > 0) {
      ROS_WARN("Throwing out packet and waiting for next one (%d missed messages remaining)", unread_packets);
      --unread_packets;
      goto REREAD;
    }
    return OW_FAILURE;
  }
  else if (prop!=property) {
    ROS_WARN("Asked node %d for property %d but got property %d",
    	     nid, property, prop);
    if (unread_packets > 0) {
      ROS_WARN("Throwing out packet and waiting for next one (%d missed messages remaining)",
	       unread_packets);
      --unread_packets;
      goto REREAD;
    }
    return OW_FAILURE;
  }   
  return OW_SUCCESS;
}

int CANbus::send_torques(int32_t* torques){
  for(int p=1; p<=npucks; p++) {
    trq[p] = torques[p];
  }
  return OW_SUCCESS;
}

int CANbus::send_torques_rt(){
  uint8_t msg[8];
  int32_t torques[NUM_ORDERS+1];
  Puck* puck;

  static double sendtime=0.0f;
  static unsigned int sendcount=0;

  static int32_t *mytorqs = (int32_t *) malloc(NUM_NODES * sizeof(int32_t));

  memcpy(mytorqs,trq,NUM_NODES*sizeof(int32_t));

  static int DEBUGCOUNT=0;
  if (++DEBUGCOUNT == 1000) {
    DEBUGCOUNT = 0;
  }

#ifdef RT_STATS
  RTIME bt1 = time_now_ns();
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
          
          if(send_rt(GROUPID(groups[g].id()), msg, 8, 100) == OW_FAILURE) {
	    ROS_ERROR("CANbus::set_torques: send failed: %s",last_error);
	    return OW_FAILURE;
          }
	  
      }
      
  }

#ifdef BH280
  static uint8_t msgbuf[20];
  CANmsg handmsg;
  bool message_received(false);
#ifdef OWD_RT
  ssize_t bytecount = rt_pipe_read(&handpipe,&handmsg,sizeof(CANmsg),TM_NONBLOCK);
  if (bytecount > 0) {
    if (bytecount < sizeof(CANmsg)) {
      snprintf(last_error,200,"Incomplete read of CANmsg from RT message pipe");
      return OW_FAILURE;
    }
    message_received=true;
  }
#else // ! OWD_RT
  bool mutex_locked=false;
  if (!mutex_trylock(&hand_queue_mutex)) {
    mutex_locked=true;
    if (hand_command_queue.size() > 0) {
      handmsg = hand_command_queue.front();
      hand_command_queue.pop();
      message_received=true;
    }
  }
#endif // ! OWD_RT
  if (message_received) {
    if (handmsg.property & 0x80) {
      // if bit 7 is 1 it's a set
      
      if (set_property_rt(handmsg.nodeid,handmsg.property & 0x7F,handmsg.value,false)
	  != OW_SUCCESS) {
	snprintf(last_error,200,"Error setting property %d = %d on hand puck %d",
		       handmsg.property, handmsg.value, handmsg.nodeid);
#ifndef OWD_RT
	mutex_unlock(&hand_queue_mutex);
#endif // ! OWD_RT
	return OW_FAILURE;
      }
    } else {
      if (get_property_rt(handmsg.nodeid, handmsg.property, &handmsg.value) != OW_SUCCESS) {
	snprintf(last_error,200,"Error getting property %d from hand puck %d",
		       handmsg.property, handmsg.nodeid);
#ifndef OWD_RT
	mutex_unlock(&hand_queue_mutex);
#endif // ! OWD_RT
	return OW_FAILURE;
      }
#ifdef OWD_RT
      bytecount = rt_pipe_write(&handpipe,&handmsg,sizeof(CANmsg),P_NORMAL);
      if (bytecount < sizeof(CANmsg)) {
	if (bytecount < 0) {
	  snprintf(last_error,200,"Error writing to hand message pipe: %d",bytecount);
	} else {
	  snprintf(last_error,200,"Incomplete write to hand message pipe: only %d of %d bytes were written",
		   bytecount,sizeof(CANmsg));
	}
	return OW_FAILURE;
      }
#else // ! OWD_RT
      hand_response_queue.push(handmsg);
#endif // ! OWD_RT
    }
  }
#ifndef OWD_RT
  if (mutex_locked) {
    mutex_unlock(&hand_queue_mutex);
  }
#endif // ! OWD_RT

#endif // BH280
  
#ifdef RT_STATS
#ifdef OWD_RT
  RTIME bt2 = time_now_ns();
  sendtime += (bt2-bt1) * 1e-6; // ns to ms
  if (++sendcount == 1000) {
    stats.cansend_time = sendtime/1000.0;
    sendcount=0;
    sendtime=0.0f;
  }
#else // ! OWD_RT
  // use gettimeofday()
#endif // ! OWD_RT
#endif // RT_STATS
  
  return OW_SUCCESS;
}

#ifdef NDEF
/*
int CANbus::read_torques(int32_t* mtrq){
  uint8_t  msg[8];
  int32_t value;
  int32_t msgid, msglen, nodeid, property;

  // Compile the packet
  msg[0] = (uint8_t)TORQ;


  if(send_rt(GROUPID(0), msg, 1, 100) == OW_FAILURE){
    ROS_WARN("CANbus::read_torques: send failed: %s", last_error);
    return OW_FAILURE;
  }
  for(int p=1; p<=npucks; ){
    if(read(&msgid, msg, &msglen, 100) == OW_FAILURE){
      ROS_WARN("CANbus::read_torques: read failed on puck %d: %s",p,last_error);
      return OW_FAILURE;
    }
    if(parse(msgid, msg, msglen, &nodeid, &property, &value) == OW_FAILURE){
      ROS_WARN("CANbus::read_torques: parse failed: %s",last_error);
      return OW_FAILURE;
    }
    if(property == TORQ){
      trq[nodeid] = value;
      p++;
    }
  }

  return OW_SUCCESS;
}
*/
#endif

int CANbus::read_positions(double* positions){

  for(int p=1; p<=npucks; p++) {
    positions[p] = pos[p];
  }

  return OW_SUCCESS;
}

int CANbus::read_positions_rt(){
  uint8_t  msg[8];

  int32_t *data=NULL;
  int32_t value;
  int32_t msgid, msglen, property, nodeid;
  static double sendtime=0.0f;
  static double readtime=0.0f;
  static unsigned int loopcount=0;
  static int missing_data_cycles=0;

  // we want to keep data around between calls so that we
  // can reuse previous joint values in case of a missed
  // CANbus message, but we need to initialize it to zero
  // the first time, so the best way is to allocate it once
  if (!data) {
    data = (int32_t *) calloc(NUM_NODES+1, sizeof(int32_t));
    if (!data) {
      return OW_FAILURE;
    }
  }

  // Compile the packet
  msg[0] = (uint8_t)AP;


#ifdef RT_STATS
#ifdef OWD_RT
  RTIME bt1 = time_now_ns();
#else // ! OWD_RT
  // use gettimeofday()
#endif // ! OWD_RT
#endif // RT_STATS

  if(send_rt(GROUPID(0), msg, 1, 100) == OW_FAILURE){
    ROS_WARN("CANbus::get_positions: send failed: %s",last_error);
    return OW_FAILURE;
  }

#ifdef RT_STATS
#ifdef OWD_RT
  RTIME bt2 = time_now_ns();
  sendtime += (bt2-bt1) * 1e-6; // ns to ms
#else // ! OWD_RT
  // use gettimeofday()
#endif // ! OWD_RT
#endif // RT_STATS

  int missed_reads(0);
#ifdef BH280
  for(int p=1; p<=npucks + 4; ){
#else
  for(int p=1; p<=npucks; ){
#endif // BH280
    if(read_rt(&msgid, msg, &msglen, 3000) == OW_FAILURE){
      // ROS_WARN("CANbus::read_positions: read failed: %s",last_error);
      ++missed_reads;
      p++;
      continue;
    }

    if(parse(msgid, msg, msglen, &nodeid, &property, &value) == OW_FAILURE){
      ROS_WARN("CANbus::read_positions: parse failed: %s",last_error);
      return OW_FAILURE;
    }

    if (property == AP) {
      data[nodeid] = value;
    } else {
      // count bad packets
      stats.canread_badpackets++;
      //  ROS_WARN("CANbus::read_positions: unexpected packet received.");
      //  ROS_WARN("  from node %d, property %d, value %d",nodeid,property,value);
    }
    p++;

  }
  if (missed_reads > 0) {
    //    if (missed_reads > 1) {
    //      // we're missing too many messages
    //      ROS_WARN("Missed CANbus replies from %d pucks in a single read cycle",missed_reads);
    //      return OW_FAILURE;
    //    } else 

    if (++missing_data_cycles == 10) {
      // we went 10 cycles in a row while missing values from at
      // least 1 puck; give up!
      ROS_WARN("Missed CANbus replies from 10 cycles in a row");
      return OW_FAILURE;
    }
  } else {
    missing_data_cycles = 0;
    if (unread_packets > 0) {
      // if we got a full read with no misses, but we still missed a packet
      // sometime in the past, it may be showing up now, so do a quick
      // check of the bus to see if there are any leftover packets that
      // we can throw away.
      if(read_rt(&msgid, msg, &msglen, 0) != OW_FAILURE){
	--unread_packets;
      }
    }
  }


#ifdef RT_STATS
#ifdef OWD_RT
  bt1 = time_now_ns();
  readtime += (bt1-bt2) * 1e-6; // ns to ms
#else // ! OWD_RT
  // use gettimeofday()
#endif // ! OWD_RT
#endif // RT_STATS

  // convert the results
  for(int p=1; p<=npucks; p++)
    pos[ pucks[p].motor() ] = 2.0*M_PI*( (double) data[ pucks[p].id() ] )/ 
                                       ( (double) pucks[p].CPR() );

#ifdef BH280
  for (int p=1; p<=4; ++p) {
    hand_positions[p] = data[p+10];
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

  if(set_property_rt(SAFETY_MODULE, IFAULT, 8, true,15000) == OW_FAILURE){
    ROS_WARN("CANbus::send_AP: set_property IFAULT=8 failed.");
    return OW_FAILURE;
  }

  for(int p=1; p<=npucks; p++){
    if(set_property_rt(pucks[p].id(), AP, apval[p], false) == OW_FAILURE){
      ROS_WARN("CANbus::send_AP: set_property AP failed.");
      return OW_FAILURE;
    }
  }

  // let the safety module see the new positions
  read_positions_rt();
   
  // start monitoring tip velocity again
  if(set_property_rt(SAFETY_MODULE, ZERO, 1, true,15000) == OW_FAILURE){
    ROS_WARN("CANbus::send_AP: set_property ZERO=1 failed.");
    return OW_FAILURE;
  }

  return OW_SUCCESS;
}

int CANbus::parse(int32_t msgid, uint8_t* msg, int32_t msglen,
		  int32_t* nodeid, int32_t* property, int32_t* value){

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
   
  if (property < 0) {
    ROS_WARN("CANbus::compile: property was not defined for this firmware version");
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

int CANbus::read_rt(int32_t* msgid, uint8_t* msgdata, int32_t* msglen, int32_t usecs){

  int32_t len;
  int i, err;
  
  RTIME sleeptime; // time to wait for interrupts, in microseconds
  if (usecs < 10000) {
    sleeptime=200; // for short delays, sleep in 200 microsecond intervals
  } else {
    sleeptime=4000; // for longer delays, sleep for 4ms
  }
  int retrycount = usecs / sleeptime + 0.5; // round to nearest int
  

#ifdef CAN_RECORD
  std::vector<canio_data> crecord;
  canio_data cdata;
  struct timeval tv;
#endif // CAN_RECORD


#ifdef PEAK_CAN
  TPCANRdMsg cmsg;
  int pendread=0;
  int pendwrite=0;
  
  bool done=false;
#ifdef OWD_RT
  while (!done) {
    err=LINUX_CAN_Read_Timeout(handle,&cmsg,0);
    if (err == CAN_ERR_QRCVEMPTY) {
      if (retrycount-- > 0) {
	if (!rt_task_self()) {
	  // we're not being called from an RT context, so use regular usleep
	  usleep(sleeptime);
	} else {
	  if ((err=rt_task_sleep(sleeptime * 1000))) {  // (convert usecs to nsecs)
	    snprintf(last_error,200,"Error during rt_task_sleep: %d",err);
	    return OW_FAILURE;
	  }
	}
#ifdef CAN_RECORD
	gettimeofday(&tv,NULL);
	cdata.secs = tv.tv_sec;
	cdata.usecs = tv.tv_usec;
	cdata.send=false;
	cdata.msgid = -1;
	cdata.msglen = 1;
	cdata.msgdata[0] = sleeptime;
	for (unsigned int i=1; i<8; ++i) {
	  cdata.msgdata[i] = 0; // must pad the extra space with zeros
	}
	crecord.push_back(cdata);
	candata.add(crecord);
	crecord.clear();
#endif // CAN_RECORD
      } else {
	snprintf(last_error,200,"timeout during read after %d microseconds",usecs);
	return OW_FAILURE;
      }
    }
    if (err == CAN_ERR_OK) {
      done=true;
      break;
    } else {
      snprintf(last_error,200,"LINUX_CAN_Read_Timeout failed: 0x%x",err);
      return OW_FAILURE;
    }
  }
#else // ! OWD_RT
  err = LINUX_CAN_Read_Timeout(handle,&cmsg,usecs);
  if (err == CAN_ERR_QRCVEMPTY) {
    snprintf(last_error,200,"timeout during read after %d microseconds",usecs);
    return OW_FAILURE;
  } else if (err != CAN_ERR_OK) {
    snprintf(last_error,200,"LINUX_CAN_Read_Timeout failed: 0x%x",err);
    return OW_FAILURE;
  }
#endif // ! OWD_RT

  *msgid = cmsg.Msg.ID;
  *msglen = cmsg.Msg.LEN;
  for (i=0; i< *msglen; ++i) {
    msgdata[i] = cmsg.Msg.DATA[i];
  }

#endif // PEAK_CAN

#ifdef ESD_CAN
  CMSG cmsg;

  int zerocount(0);
  len=1;
  bool done=false;
  while (!done) {
    err=canTake(handle, &cmsg, &len);
    if ((err == NTCAN_RX_TIMEOUT)
	|| ((err == NTCAN_SUCCESS) && (len == 0))) {
      if (retrycount-- > 0) {
#ifdef OWD_RT
	if (!rt_task_self()) {
	  // we're not being called from an RT context, so use regular usleep
	  usleep(sleeptime);
	} else {
	  if ((err=rt_task_sleep(sleeptime * 1000))) {  // (convert usecs to nsecs)
	    snprintf(last_error,200,"Error during rt_task_sleep: %d",err);
	    return OW_FAILURE;
	  }
	}
#else // ! OWD_RT
	usleep(sleeptime);	// give time for the CAN message to arrive
#endif // ! OWD_RT
#ifdef CAN_RECORD
	gettimeofday(&tv,NULL);
	cdata.secs = tv.tv_sec;
	cdata.usecs = tv.tv_usec;
	cdata.send=false;
	cdata.msgid = -1;
	cdata.msglen = 1;
	cdata.msgdata[0] = sleeptime;
	for (unsigned int i=1; i<8; ++i) {
	  cdata.msgdata[i] = 0; // must pad the extra space with zeros
	}
	crecord.push_back(cdata);
	candata.add(crecord);
	crecord.clear();
#endif // CAN_RECORD
	
	len=1; // make sure we pass in the right len each time
      } else {
	snprintf(last_error,200,"timeout during read after %d microseconds",usecs);
	return OW_FAILURE;
      }
    } else if (err == NTCAN_SUCCESS) {
      done=true;
      break;
    } else {
      snprintf(last_error,200,"canTake failed: 0x%x",err);
      return OW_FAILURE;
    }
  }
  
  if (len != 1) {
    snprintf(last_error,200,"received a message of length: %d",len);
    return OW_FAILURE;
  }

  *msgid = cmsg.id;
  *msglen = cmsg.len;
  for(i=0; i<*msglen; i++)
    msgdata[i] = cmsg.data[i];
 
#endif // ESD_CAN

#ifdef CAN_RECORD
  gettimeofday(&tv,NULL);
  cdata.secs = tv.tv_sec;
  cdata.usecs = tv.tv_usec;
  cdata.send=false;
  cdata.msgid = *msgid;
  cdata.msglen = *msglen;
  for (unsigned int i=0; i<8; ++i) {
    if (i < *msglen) {
      cdata.msgdata[i] = msgdata[i];
    } else {
      cdata.msgdata[i] = 0; // must pad the extra space with zeros
    }
  }
  crecord.push_back(cdata);
  candata.add(crecord);
#endif // CAN_RECORD

  return OW_SUCCESS;
}

int CANbus::send_rt(int32_t msgid, uint8_t* msgdata, int32_t msglen, int32_t usecs) {

  int32_t len = 1;
  int i;
  int32_t err;

  int32_t sleeptime;
  if (usecs < 2000) {
    sleeptime=100; // for short delays, sleep in 100 microsecond intervals
  } else {
    sleeptime=1000; // for longer delays, sleep for 1ms
  }
  int retrycount = usecs / sleeptime + 0.5;

#ifdef CAN_RECORD
  std::vector<canio_data> crecord;
  canio_data cdata;
  //  RTIME t1 = time_now_ns();
  //  cdata.secs = t1 / 1e9;
  //  cdata.usecs = (t1 - cdata.secs*1e9) / 1e3;
  struct timeval tv;
  gettimeofday(&tv,NULL);
  cdata.secs = tv.tv_sec;
  cdata.usecs = tv.tv_usec;
  cdata.send=true;
  cdata.msgid = msgid;
  cdata.msglen = msglen;
  for (unsigned int i=0; i<8; ++i) {
    if (i < msglen) {
      cdata.msgdata[i] = msgdata[i];
    } else {
      cdata.msgdata[i] = 0; // must pad the extra space with zeros
    }
  }
  crecord.push_back(cdata);
  candata.add(crecord);
#endif // CAN_RECORD
  
  
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
  
  while (((err = CAN_Write(handle,&msg)) != CAN_ERR_OK) &&
	 (retrycount-- > 0)) {
#ifdef OWD_RT
    if (!rt_task_self()) {
      // we're not being called from an RT context, so use regular usleep
      usleep(sleeptime);
    } else {
      if ((err=rt_task_sleep(sleeptime * 1000))) {  // (convert usecs to nsecs)
	snprintf(last_error,200,"Error during rt_task_sleep: %d",err);
	return OW_FAILURE;
      }
    }
#else // ! OWD_RT
    usleep(sleeptime);
#endif // ! OWD_RT
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
  
  while (((err=canSend(handle, &cmsg, &len)) != NTCAN_SUCCESS) && 
	 (retrycount-- > 0)) {
#ifdef OWD_RT
    if (!rt_task_self()) {
      // we're not being called from an RT context, so use regular usleep
      usleep(sleeptime);
    } else {
      if ((err=rt_task_sleep(sleeptime * 1000))) {  // (convert usecs to nsecs)
	snprintf(last_error,200,"Error during rt_task_sleep: %d",err);
	return OW_FAILURE;
      }
    }
#else // ! OWD_RT
    usleep(sleeptime);
#endif // ! OWD_RT
  }
  if (err != NTCAN_SUCCESS) {
    snprintf(last_error,200,"canSend failed: 0x%x",err);
    return OW_FAILURE;
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
  if ((set_property_rt(SAFETY_MODULE,TL1,6000,true,15000) == OW_FAILURE) ||
      (set_property_rt(SAFETY_MODULE,TL2,9000,true,15000) == OW_FAILURE) ||
      (set_property_rt(SAFETY_MODULE,VL1,(int32_t)(2*0x1000),true,15000) == OW_FAILURE) ||
      (set_property_rt(SAFETY_MODULE,VL2,(int32_t)(3*0x1000),true,15000) == OW_FAILURE)) {
      return OW_FAILURE;
  }

  int32_t voltlevel;
#ifdef SET_VOLTAGE_LIMITS
  // set appropriate high-voltage levels for battery operation
  if (get_property_rt(SAFETY_MODULE,VOLTH1,&voltlevel) == OW_FAILURE) {
      ROS_ERROR("CANbus::limits failed to get previous high voltage warning level.");
      return OW_FAILURE;
  }
  ROS_DEBUG_NAMED("canlimits","VOLTH1 was %d, changing to 54",voltlevel);
  if (set_property_rt(SAFETY_MODULE,VOLTH1,54,true,15000) == OW_FAILURE) {
      ROS_ERROR("CANbus::limits: set_prop failed");
      return OW_FAILURE;
  }

  if (get_property_rt(SAFETY_MODULE,VOLTH2,&voltlevel) == OW_FAILURE) {
      ROS_ERROR("CANbus::limits failed to get previous high voltage warning level.");
      return OW_FAILURE;
  }
  ROS_DEBUG_NAMED("canlimits","VOLTH1 was %d, changing to 57",voltlevel);
  if (set_property_rt(SAFETY_MODULE,VOLTH2,57,true,15000) == OW_FAILURE) {
      ROS_ERROR("CANbus::limits: set_prop failed");
      return OW_FAILURE;
  }
#endif

  return OW_SUCCESS;
  
#ifdef OLD_VEL_LIMITS
  if(0<jointVel && jointVel<7){           // If the vel (rad/s) is reasonable
    conversion = (int32_t)(jointVel*0x1000); // Convert to Q4.12 rad/s
    if(set_property_rt(SAFETY_MODULE, VL2, conversion, true,15000) == OW_FAILURE){
      ROS_ERROR("WAM::set_limits: set_prop failed.");
      return OW_FAILURE;
    }
  }
   
  if(0<tipVel && tipVel<7){               // If the vel (m/s) is reasonable
    conversion = (int32_t)(tipVel*0x1000);   // Convert to Q4.12 rad/s
    if(set_property_rt(SAFETY_MODULE, VL2, conversion, true,15000) == OW_FAILURE){
      ROS_ERROR("WAM::set_limits: set_prop failed.");
      return OW_FAILURE;
    }
  }
   
  if(0<elbowVel && elbowVel<7){           // If the vel (m/s) is reasonable
    conversion = (int32_t)(elbowVel*0x1000); // Convert to Q4.12 rad/s
    if(set_property_rt(SAFETY_MODULE, VL2, conversion, true,15000) == OW_FAILURE){
      ROS_ERROR("WAM::set_limits: set_prop failed.");
      return OW_FAILURE;
    }
  }
  return OW_SUCCESS;
#endif // OLD_VEL_LIMITS
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
  int32_t puck1_state;
  static int count=0;
  static double timesum = 0.0f;
#ifdef RT_STATS
  RTIME t1 = time_now_ns();
#endif
  if (get_property_rt(1,MODE,&puck1_state) == OW_FAILURE) {
    ROS_WARN("Failure while trying to get MODE of puck #1");
    puck1_state = -1;
    return OW_FAILURE;
  }

#ifdef RT_STATS
  RTIME t2 = time_now_ns();
  timesum += (t2-t1)/1e6;
  if (++count == 100) {
    stats.cansetpuckstate_time = timesum/100.0;
    count=0;
    timesum=0.0;
  }
#endif

  puck_state = puck1_state;
  return OW_SUCCESS;
}


#ifdef BH280

int CANbus::hand_set_property(int32_t id, int32_t prop, int32_t val) {
  CANmsg msg;
  msg.nodeid=id;
  msg.property=prop | 0x80; // set the high bit to 1
  msg.value=val;

#ifdef OWD_RT
  // use mutex to protect against re-entry from multiple service calls
  mutex_lock(&hand_cmd_mutex);
  ssize_t bytes = ::write(handpipe_fd,&msg,sizeof(CANmsg));
  mutex_unlock(&hand_cmd_mutex);
  if (bytes < sizeof(CANmsg)) {
    if (bytes < 0) {
      ROS_ERROR_NAMED("can_bh280","Error writing to hand message pipe: %d",bytes);
    } else {
      ROS_ERROR_NAMED("can_bh280","Incomplete write of data to hand message pipe: only %d of %d bytes written",
		      bytes,sizeof(CANmsg));
    }
    return OW_FAILURE;
  }
#else // ! OWD_RT

  if (!mutex_lock(&hand_queue_mutex)) {
    hand_command_queue.push(msg);
    mutex_unlock(&hand_queue_mutex);
  } else {
    ROS_ERROR_NAMED("can_bh280","Unable to acquire hand_queue_mutex; hand_set_property Node %d Prop %d=%d failed.",id,prop,val);
    return OW_FAILURE;
  }

#endif // ! OWD_RT

  return OW_SUCCESS;
}
 
int CANbus::hand_get_property(int32_t id, int32_t prop, int32_t *value) {
  CANmsg msg;
  msg.nodeid=id;
  msg.property=prop;
  msg.value=0;

  if (mutex_lock(&hand_cmd_mutex)) {
    ROS_ERROR_NAMED("can_bh280","Could not lock hand command mutex");
    return OW_FAILURE;
  }

#ifdef OWD_RT
  int bytes = ::write(handpipe_fd,&msg,sizeof(CANmsg));
  if (bytes < sizeof(CANmsg)) {
    if (bytes < 0) {
      ROS_ERROR_NAMED("can_bh280","Error writing to hand message pipe: %d", bytes);
    } else {
      ROS_ERROR_NAMED("can_bh280","Incomplete write of data to hand message pipe: only %d of %d bytes written",
		      bytes,sizeof(CANmsg));
    }
    mutex_unlock(&hand_cmd_mutex);
    return OW_FAILURE;
  }
  // now read from the pipe; the read will return as soon as the data is available
  bytes = ::read(handpipe_fd, &msg, sizeof(CANmsg));
  if (bytes < 0) {
    ROS_ERROR_NAMED("can_bh280","Error reading data from hand message pipe: %d", errno);
    mutex_unlock(&hand_cmd_mutex);
    return OW_FAILURE;
  }
  if (bytes < sizeof(CANmsg)) {
    ROS_ERROR_NAMED("can_bh280","Incomplete read of message from hand message pipe: expected %d but got %d bytes",
		    sizeof(CANmsg),bytes);
    mutex_unlock(&hand_cmd_mutex);
    return OW_FAILURE;
  }
#else // ! OWD_RT
  mutex_lock(&hand_queue_mutex);
  hand_command_queue.push(msg);
  mutex_unlock(&hand_queue_mutex);

  // wait for the response
  bool done=false;
  do {
    usleep(1000);
    mutex_lock(&hand_queue_mutex);
    if (hand_response_queue.size() > 0) {
      msg = hand_response_queue.front();
      hand_response_queue.pop();
      done=true;
    }
    mutex_unlock(&hand_queue_mutex);
  } while (!done);
#endif // ! OWD_RT

  mutex_unlock(&hand_cmd_mutex);

  if (msg.nodeid != id) {
    ROS_ERROR_NAMED("can_bh280","Expecting response from hand puck %d but got message from puck %d",
	     id,msg.nodeid);
    return OW_FAILURE;
  }
  if (msg.property != prop) {
    ROS_ERROR_NAMED("can_bh280","Asked hand puck %d for property %d but got property %d",id,prop,msg.property);
    return OW_FAILURE;
  }
  *value = msg.value;
  return OW_SUCCESS;
}


int CANbus::hand_activate(int32_t *nodes) {
  for (int32_t nodeid=11; nodeid<15; ++nodeid) {
    if (nodes[nodeid] == STATUS_RESET) {
      ROS_DEBUG_NAMED("can_bh280","Waking up puck %d...",nodeid);
      if (wake_puck(nodeid) != OW_SUCCESS) {
	ROS_WARN_NAMED("can_bh280","Could not wake hand puck %d",nodeid);
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("can_bh280","done");
    } else {
      ROS_DEBUG_NAMED("can_bh280","setting puck %d to idle mode...",nodeid);
      // this was set to CONTROLLER_IDLE originally
      if(set_property_rt(nodeid, MODE, PUCK_IDLE, true, 10000) == OW_FAILURE){
	ROS_WARN_NAMED("cancheck","Failed to set MODE=PUCK_IDLE on puck %d",nodeid);
	return OW_FAILURE;
      }
      ROS_DEBUG_NAMED("can_bh280","done");
    }
  }
  return OW_SUCCESS;
}

int CANbus::hand_set_state_rt() {
  // THIS FUNCTION IS ALREADY CALLED FROM THE RT LOOP, SO WE USE
  // GET_PROPERTY DIRECTLY INSTEAD OF HAND_GET_PROPERTY.

  // If we were already stationary, assume we're still
  // stationary.
  if (handstate != HANDSTATE_MOVING) {
    return OW_SUCCESS;
  }
  
#ifdef RT_STATS
  static double timesum=0.0f;
  static int count=0;
  RTIME t1 = time_now_ns();
#endif

  // otherwise, check one finger for movement
  int32_t mode;
  if (get_property_rt(first_moving_finger,MODE,&mode) != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280",
		   "Failed to get MODE from hand puck %d: ",first_moving_finger, last_error);
    //    handstate = HANDSTATE_UNINIT;
    return OW_FAILURE;
  }

#ifdef RT_STATS
  RTIME t2 = time_now_ns();
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
    static int no_movement_count = 0;
    // make sure we're actually still moving
    static int32_t last_f4_pos = 0; // the initial value doesn't really matter; we're just
                                    // checking to see if it's the same 10 times in a row
    if (llabs(last_f4_pos - hand_positions[4]) < 50) {
      if (++no_movement_count < 10) {
	handstate = HANDSTATE_MOVING;
	return OW_SUCCESS;
      }
    }
    last_f4_pos = hand_positions[4];
    no_movement_count=0;
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
  if (set_property_rt(nodeid,CMD,CMD_HI) != OW_SUCCESS) {
    ROS_WARN_NAMED("can_bh280","Error sending HI to hand puck %d",nodeid);
    return OW_FAILURE;
  }
  // now wait for the change in mode
  ROS_DEBUG_NAMED("can_bh280", "Waiting for MODE change to IDLE");
  int32_t mode = MODE_VELOCITY;
  int32_t sleepcount=0;
  while ((get_property_rt(nodeid,MODE,&mode,1000000) == OW_SUCCESS) &&
	 (mode == MODE_VELOCITY) &&
	 ros::ok()) {
    usleep(50000);
    if (++sleepcount ==20) {
      ROS_WARN_NAMED("can_bh280","Still waiting for finger to finish HI; mode is %d", mode);
      sleepcount=0;
    }
  }
  if (!ros::ok()) {
    return OW_FAILURE;
  }
  if (mode == MODE_VELOCITY) {
    ROS_WARN_NAMED("can_bh280","No response within 700ms from finger puck %d while waiting for HI",nodeid);
    return OW_FAILURE;
  }
  ROS_DEBUG_NAMED("can_bh280", "Finger reset");
  
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
  bool ready=true;
  for (int nodeid=11; nodeid<=14; ++nodeid) {
    int32_t tstop;
    if (get_property_rt(nodeid,TSTOP,&tstop,4000) != OW_SUCCESS) {
      ROS_ERROR("Could not get TSTOP property from hand puck %d",
		nodeid);
      return OW_FAILURE;
    }
    if (((nodeid < 14) && (tstop != 50)) 
	|| ((nodeid == 14) && (tstop != 200))) {
      ready=false;
      break;
    }
  }
  
  if (ready) {
    // all the fingers have already been HI'd
    handstate=HANDSTATE_DONE;
    return OW_SUCCESS;
  }
  
  ROS_FATAL("Please move the hand to a safe position and hit <RETURN> to reset the hand");
  char *line=NULL;
  size_t linelen = 0;
  getline(&line,&linelen,stdin);
  free(line);
  
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
    if (set_property_rt(nodeid,TSTOP,50) != OW_SUCCESS) {
      return OW_FAILURE;
    }
    usleep(100);
    // F1 to F3 have HOLD=0 because they're not backdrivable
    if (set_property_rt(nodeid,HOLD,0) != OW_SUCCESS) {
      return OW_FAILURE;
    }
    usleep(100);
  }
  if (set_property_rt(14,TSTOP,200) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  usleep(100);
  // F4 has HOLD=1 so that it won't flop around
  if (set_property_rt(14,HOLD,1) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  usleep(100);
  for (int32_t nodeid=11; nodeid<=14; ++nodeid) {
    int32_t value;
    if (get_property_rt(nodeid,HOLD,&value,20000) != OW_SUCCESS) {
      return OW_FAILURE;
    }
    ROS_INFO_NAMED("can_bh280","Puck %d HOLD is %d",nodeid,value);
    if (get_property_rt(nodeid,GRPA,&value,20000) != OW_SUCCESS) {
      return OW_FAILURE;
    }
    ROS_INFO_NAMED("can_bh280","Puck %d GRPA is %d",nodeid,value);
    if (get_property_rt(nodeid,GRPB,&value,20000) != OW_SUCCESS) {
      return OW_FAILURE;
    }
    ROS_INFO_NAMED("can_bh280","Puck %d GRPB is %d",nodeid,value);
    if (get_property_rt(nodeid,GRPC,&value,20000) != OW_SUCCESS) {
      return OW_FAILURE;
    }
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
  set_property(11,CMD,CMD_M);
  set_property(12,CMD,CMD_M);
  set_property(13,CMD,CMD_M);
  set_property(14,CMD,CMD_M); */
  ROS_INFO_NAMED("bhd280", "executing hand_move");

  if (hand_set_property(11,E,finger_radians_to_encoder(p1)) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  if (hand_set_property(12,E,finger_radians_to_encoder(p2)) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  if (hand_set_property(13,E,finger_radians_to_encoder(p3)) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  if (hand_set_property(14,E,spread_radians_to_encoder(p4)) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  if (hand_set_property(11,MODE,MODE_TRAPEZOID) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  if (hand_set_property(12,MODE,MODE_TRAPEZOID) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  if (hand_set_property(13,MODE,MODE_TRAPEZOID) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  if (hand_set_property(14,MODE,MODE_TRAPEZOID) != OW_SUCCESS) {
      return OW_FAILURE;
    }
  first_moving_finger=11;
  handstate = HANDSTATE_MOVING;
  ROS_INFO_NAMED("bhd280", "done hand_move");
  return OW_SUCCESS;
}

int CANbus::hand_velocity(double v1, double v2, double v3, double v4) {
  ROS_INFO_NAMED("bhd280", "executing hand_velocity");
  if (hand_set_property(11,V,finger_radians_to_encoder(v1)/1000.0) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  if (hand_set_property(12,V,finger_radians_to_encoder(v2)/1000.0) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  if (hand_set_property(13,V,finger_radians_to_encoder(v3)/1000.0) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  if (hand_set_property(14,V,finger_radians_to_encoder(v4)/1000.0) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  
  if ((v1 != 0.0)
      && (hand_set_property(11,MODE,MODE_VELOCITY) != OW_SUCCESS)) {
    return OW_FAILURE;
  }
 if ((v2 != 0.0)
     && (hand_set_property(12,MODE,MODE_VELOCITY) != OW_SUCCESS)) {
    return OW_FAILURE;
  }
 if ((v3 != 0.0)
     && (hand_set_property(13,MODE,MODE_VELOCITY) != OW_SUCCESS)) {
    return OW_FAILURE;
 }
 if ((v4 != 0.0)
     && (hand_set_property(14,MODE,MODE_VELOCITY) != OW_SUCCESS)) {
   return OW_FAILURE;
 }
 first_moving_finger=11;
 handstate = HANDSTATE_MOVING;
 ROS_INFO_NAMED("bhd280", "done executing hand_velocity");
 return OW_SUCCESS;
}
 
int CANbus::hand_relax() {
  if (hand_set_property(11,MODE,PUCK_IDLE) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  if (hand_set_property(12,MODE,PUCK_IDLE) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  if (hand_set_property(13,MODE,PUCK_IDLE) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  if (hand_set_property(14,MODE,PUCK_IDLE) != OW_SUCCESS) {
    return OW_FAILURE;
  }
  handstate = HANDSTATE_DONE;
  return OW_SUCCESS;
}

int CANbus::hand_get_positions(double &p1, double &p2, double &p3, double &p4) {
  p1 = finger_encoder_to_radians(hand_positions[1]);
  p2 = finger_encoder_to_radians(hand_positions[2]);
  p3 = finger_encoder_to_radians(hand_positions[3]);
  p4 = spread_encoder_to_radians(hand_positions[4]);
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


void CANstats::rosprint() {
#ifdef RT_STATS
  ROS_DEBUG_NAMED("times","CANbus::send %2.1fms per group (2 groups)",
		  cansend_time);
  ROS_DEBUG_NAMED("times","CANbus::read: send=%2.1fms, read=%2.1fms",
		  canread_sendtime, canread_readtime);
  ROS_DEBUG_NAMED("times","CANbus::read: bad packets = %d",
		  canread_badpackets);
  canread_badpackets = 0;
  ROS_DEBUG_NAMED("times","CANbus::set_puck_state: %2.2fms",
  		  cansetpuckstate_time);
  ROS_DEBUG_NAMED("times","CANbus::set_hand_state: %2.2fms",
  		  cansethandstate_time);
#endif
}
  
 RTIME CANbus::time_now_ns() {
#ifdef OWD_RT
   return rt_timer_ticks2ns(rt_timer_read());
#else // ! OWD_RT
   struct timeval tv;
   gettimeofday(&tv,NULL);
   return (tv.tv_sec * 1e6 + tv.tv_usec);
#endif // ! OWD_RT
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
     HOLD = i++;
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
     EN = i++;
     EN2 = i++;
     JP = i++;
     JP2 = i++;
     JOFST = i++;
     JOFST2 = i++;
     TIE = i++;
     ECMAX = i++;
     ECMIN = i++;
     LFLAGS = i++;
     LCTC = i++;
     LCVC = i++;
     TACT = i++;
     TACTID = i++;

     PROP_END = i++;

     AP = P; // Handle parameter name change
     TENSION = FET1;
     TORQ=T;
   }
}

#ifdef CAN_RECORD 
 template<> inline bool DataRecorder<CANbus::canio_data>::dump (const char *fname) {
  FILE *csv = fopen(fname,"w");
  if (csv) {
    for (unsigned int i=0; i<count; ++i) {
      CANbus::canio_data cdata = data[i];
      char timestring[100];
      time_t logtime = cdata.secs;
	//	+ 4*3600; // shift by 4 hours to account for EDT - GMT shift
      strftime(timestring,100,"%F %T",localtime(&logtime));
	
      fprintf(csv,"[%s,%06d] ",timestring,cdata.usecs);
      if (cdata.msgid == -1) {
	fprintf(csv,"SLEEP %d\n",cdata.msgdata[0]);
	continue;
      }
      if (cdata.send) {
	fprintf(csv,"SEND ");
      } else {
	fprintf(csv,"READ ");
      }
      int32_t recv_id = cdata.msgid & 0x1F; // bits 0-4
      int32_t send_id = (cdata.msgid >> 5) & 0x1F;  // bits 5-9
      fprintf(csv,"%02d ", send_id);
      if (cdata.msgid & 0x400) {
	fprintf(csv,"G%02d ",recv_id);
      } else {
	fprintf(csv," %02d ",recv_id);
      }
      if (cdata.msgdata[0] & 0x80) {
	if (cdata.msgdata[0] == (42 | 0x80)) {
	  // Packed Torque message
	  int32_t tq1 = (cdata.msgdata[1] << 6) +
	    (cdata.msgdata[2] >> 2);
	  int32_t tq2 = ((cdata.msgdata[2] & 0x03) << 12) + 
	    (cdata.msgdata[3] << 4) +
	    (cdata.msgdata[4] >> 4);
	  int32_t tq3 = ((cdata.msgdata[4] & 0x0F) << 10) +
	    (cdata.msgdata[5] << 2) +
	    (cdata.msgdata[6] >> 6);
	  int32_t tq4 = ((cdata.msgdata[6] & 0x3F) << 8) +
	    cdata.msgdata[7];
	  fprintf(csv,"SET %03d=%d,%d,%d,%d",cdata.msgdata[0] & 0x7F, tq1,tq2,tq3,tq4);
	} else if ((cdata.msgid & 0x41F) == 0x403) {
	  // message set to GROUP 3 are 22-bit position updates
	  int32_t value = (cdata.msgdata[0] << 16) +
	    (cdata.msgdata[1] << 8) +
	    cdata.msgdata[2];
	  fprintf(csv,"SET P=%d",value);
	} else {
	  // regular property
	  int32_t value = (cdata.msgdata[3] << 8) + cdata.msgdata[2];
	  if (cdata.msglen == 6) {
	    value += (cdata.msgdata[4] << 16) + (cdata.msgdata[5] << 24);
	  }
	  fprintf(csv,"SET %03d=%d",cdata.msgdata[0] & 0x7F, value);
	}
      } else {
	fprintf(csv,"GET %03d",cdata.msgdata[0]);
      }
      
      fprintf(csv,"\n");
    }
    fclose(csv);
    return true;
  } else {
    return false;
  }
}
#endif // CAN_RECORD

 CANbus::~CANbus(){
   //   rt_intr_delete(&rt_can_intr);   
   if(pucks!=NULL) delete pucks; 
   if(trq!=NULL) delete trq; 
   if(pos!=NULL) delete pos;
#ifdef CAN_RECORD
   char dumpname[200];
   snprintf(dumpname,200,"candata%d.log",id);
   candata.dump(dumpname);
   ROS_DEBUG("dumped CANbus logs to %s",dumpname);
#endif    
 }
 
