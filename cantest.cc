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

#include <ntcan.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#define TX_QUEUE_SIZE       (32)
#define RX_QUEUE_SIZE       (32)
#define TX_TIMEOUT          (500)
#define RX_TIMEOUT          (50)

// compile with:
//   g++ -Iesdcan/lib32 -o cantest cantest.cc -Lesdcan/lib32 -lntcan


NTCAN_HANDLE handle;


int allow_message(int id, int mask){
  
	
  int i;
  for(i=0; i<2048; i++){
      if((i & ~mask) == id){
          if(canIdAdd(handle, i) != NTCAN_SUCCESS){
	    return -1;
          }
      }
  }
  

  return 0;
}

int send(int32_t msgid, uint8_t* msg, int32_t msglen, bool block){


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
      return err;
    }
  }
  else{
    if( (err=canSend(handle, &cmsg, &len)) != NTCAN_SUCCESS){
      //      ROS_ERROR("CANbus::send: canSend failed: %x",err);
      return err;
    }
  }
    
  return 0;
}

static double mtime(void)
{
  struct timeval  tv;
  struct timezone tz;

  gettimeofday(&tv, &tz);
  return( tv.tv_sec *1000000.0 + tv.tv_usec);
}

const char *err2str(NTCAN_RESULT ntstatus);

int main() {
  double sendtime = 0.0f;
  unsigned int sendcount = 0;

  printf("opening CAN device\n");
  int err = (canOpen(0, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, 
		     TX_TIMEOUT, RX_TIMEOUT, &handle));
  if (err != NTCAN_SUCCESS){  
    printf("Can't open CAN device: %s\n",err2str(err));
    return -1;
  }

  if(canSetBaudrate(handle, 0) != NTCAN_SUCCESS){
    printf("CANbus::open: canSetBaudrate failed.\n");
    return -1;
  }
    
  // Mask 3E0: 0000 0011 1110 0000
  // Messages sent directly to host
  if(allow_message(0x0000, 0x03E0) == -1){
    printf("CANbus::open: allow_message failed.\n");
    return -1;
  }
  // Group 3 messages
  if(allow_message(0x0403, 0x03E0) == -1){
    printf("CANbus::open: allow_message failed.\n");
    return -1;
  }
  // Group 6 messages
  if(allow_message(0x0406, 0x03E0) == -1){
    printf("CANbus::open: allow_message failed.\n");
    return -1;
  }

  printf("Sending 5000 packets\n");
  while (sendcount++ < 1000) {

    // Compile the packet
    uint8_t  msg[8];
    msg[0] = (uint8_t) 26;
    
    double bt2 = mtime();
    int err = send(1, msg, 1, true);
    if (err) {
      printf("cannot send: error %s\n",err2str(err));
    }
    err = send(1, msg, 1, true);
    if (err) {
      printf("cannot send: error %s\n",err2str(err));
    }
    err = send(1, msg, 1, true);
    if (err) {
      printf("cannot send: error %s\n",err2str(err));
    }
    err = send(1, msg, 1, true);
    if (err) {
      printf("cannot send: error %s\n",err2str(err));
    }
    err = send(1, msg, 1, true);
    if (err) {
      printf("cannot send: error %s\n",err2str(err));
    }
    
    double bt1 = mtime();
    sendtime += (bt1-bt2) * 1e-3; // us to ms
  }
  printf("Send time %2.1fms\n",sendtime/5000.0);
}

const char *err2str(NTCAN_RESULT ntstatus)
{
  struct ERR2STR {
    NTCAN_RESULT  ntstatus;
    const char   *str;
  };

  static const struct ERR2STR err2str[] = {
    { NTCAN_SUCCESS            , "NTCAN_SUCCESS"            },
    { NTCAN_RX_TIMEOUT         , "NTCAN_RX_TIMEOUT"         },
    { NTCAN_TX_TIMEOUT         , "NTCAN_TX_TIMEOUT"         },
    { NTCAN_TX_ERROR           , "NTCAN_TX_ERROR"           },
    { NTCAN_CONTR_OFF_BUS      , "NTCAN_CONTR_OFF_BUS"      },
    { NTCAN_CONTR_BUSY         , "NTCAN_CONTR_BUSY"         },
    { NTCAN_CONTR_WARN         , "NTCAN_CONTR_WARN"         },
    { NTCAN_NO_ID_ENABLED      , "NTCAN_NO_ID_ENABLED"      },
    { NTCAN_ID_ALREADY_ENABLED , "NTCAN_ID_ALREADY_ENABLED" },
    { NTCAN_ID_NOT_ENABLED     , "NTCAN_ID_NOT_ENABLED"     },
    { NTCAN_INVALID_FIRMWARE   , "NTCAN_INVALID_FIRMWARE"   },
    { NTCAN_MESSAGE_LOST       , "NTCAN_MESSAGE_LOST"       },
    { NTCAN_INVALID_PARAMETER  , "NTCAN_INVALID_PARAMETER"  },
    { NTCAN_INVALID_HANDLE     , "NTCAN_INVALID_HANDLE"     },
    { NTCAN_NET_NOT_FOUND      , "NTCAN_NET_NOT_FOUND"      },
#ifdef NTCAN_IO_INCOMPLETE
    { NTCAN_IO_INCOMPLETE      , "NTCAN_IO_INCOMPLETE"      },
#endif
#ifdef NTCAN_IO_PENDING
    { NTCAN_IO_PENDING         , "NTCAN_IO_PENDING"         },
#endif
#ifdef NTCAN_INVALID_HARDWARE
    { NTCAN_INVALID_HARDWARE   , "NTCAN_INVALID_HARDWARE"   },
#endif
#ifdef NTCAN_PENDING_WRITE
    { NTCAN_PENDING_WRITE      , "NTCAN_PENDING_WRITE"      },
#endif
#ifdef NTCAN_PENDING_READ
    { NTCAN_PENDING_READ       , "NTCAN_PENDING_READ"       },
#endif
#ifdef NTCAN_INVALID_DRIVER
    { NTCAN_INVALID_DRIVER     , "NTCAN_INVALID_DRIVER"     },
#endif
#ifdef NTCAN_OPERATION_ABORTED
    { NTCAN_OPERATION_ABORTED  , "NTCAN_OPERATION_ABORTED"  },
#endif
#ifdef NTCAN_WRONG_DEVICE_STATE
    { NTCAN_WRONG_DEVICE_STATE , "NTCAN_WRONG_DEVICE_STATE"  },
#endif
    { NTCAN_INSUFFICIENT_RESOURCES, "NTCAN_INSUFFICIENT_RESOURCES"},
#ifdef NTCAN_HANDLE_FORCED_CLOSE
    { NTCAN_HANDLE_FORCED_CLOSE, "NTCAN_HANDLE_FORCED_CLOSE"  },
#endif
#ifdef NTCAN_NOT_IMPLEMENTED
    { NTCAN_NOT_IMPLEMENTED    , "NTCAN_NOT_IMPLEMENTED"  },
#endif
#ifdef NTCAN_NOT_SUPPORTED
    { NTCAN_NOT_SUPPORTED      , "NTCAN_NOT_SUPPORTED"  },
#endif
#ifdef NTCAN_SOCK_CONN_TIMEOUT
    { NTCAN_SOCK_CONN_TIMEOUT  , "NTCAN_SOCK_CONN_TIMEOUT"     },
#endif
#ifdef NTCAN_SOCK_CMD_TIMEOUT
    { NTCAN_SOCK_CMD_TIMEOUT   , "NTCAN_SOCK_CMD_TIMEOUT"      },
#endif
#ifdef NTCAN_SOCK_HOST_NOT_FOUND
    { NTCAN_SOCK_HOST_NOT_FOUND, "NTCAN_SOCK_HOST_NOT_FOUND"   },
#endif
#ifdef NTCAN_CONTR_ERR_PASSIVE
    { NTCAN_CONTR_ERR_PASSIVE  , "NTCAN_CONTR_ERR_PASSIVE"   },
#endif
    { 0xffffffff               , "NTCAN_UNKNOWN"               }    /* stop-mark */
  };

  const struct ERR2STR *es = err2str;

  while ((es->ntstatus != ntstatus) &&
	 ((uint32_t)es->ntstatus != 0xffffffff)) {
    es++;
  }
  if (es->ntstatus != 0xffffffff) {
    return es->str;
  } else {
    return "";
  }
}
