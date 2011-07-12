/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
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

#include <CANbus.hh>
#include <CANdefs.hh>

CANbus *btinterface_bus = NULL;

#define mbxID               (0)
#define BASE_ID             (0)

#define ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BASE_ID)
#define NODE2ADDR(x) (((mbxID + BASE_ID) << 5) | ((x) + BASE_ID))
#define GROUPID(n)   (((mbxID + BASE_ID) << 5) | (0x0400 + (n)))
#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif
#define MODE_IDLE      0
#define MODE_TORQUE    2
#define MODE_PID       3
#define MODE_VELOCITY  4
#define MODE_TRAPEZOID 5
enum {
    ROLE_TATER,
    ROLE_GIMBALS,
    ROLE_SAFETY,
    ROLE_WRAPTOR,
    ROLE_TRIGGER,
    ROLE_BHAND,
    ROLE_FORCE
};
long jointPosition[32];


void setPropertySlow(int b, int id, int prop, bool verify, long value) {
  if (! btinterface_bus) {
    throw "Must allocate a CANbus object for btinterface_bus pointer";
  }
  
  btinterface_bus->set_property_rt(id,prop,value,verify,15000);
  usleep(1000);
}

int canReadMsg(int bus, int *id, int *len, unsigned char *data, int block) {
  if (! btinterface_bus) {
    throw "Must allocate a CANbus object for btinterface_bus pointer";
  }
  int32_t usecs;
  if (block) {
    usecs=10000000; 
  } else {
    usecs=2000;
  }
  return btinterface_bus->read_rt(id,data,len,usecs);
}

int canSendMsg(int bus, int id, int len, unsigned char *data, int block) {
  if (! btinterface_bus) {
    throw "Must allocate a CANbus object for btinterface_bus pointer";
  }
  int32_t usecs;
  if (block) {
    usecs=10000000; 
  } else {
    usecs=2000;
  }
  return btinterface_bus->send_rt(id,data,len,usecs);
}

void mvprintw(int line, int pos, const char *format, ...) {
  printf("\n%s",format);
}

int wakePuck(int bus, int who)
{
   setPropertySlow(bus, who, 5, FALSE, STATUS_READY); // Must use '5' for STAT
   usleep(1000000); // Wait 500ms for puck to initialize

   return(0);
}

void initPropertyDefs(int firmwareVersion){
  btinterface_bus->initPropertyDefs(firmwareVersion);
}

int parseMessage(
   /* Input */
   int id                      /** The message ID */,
   int len                     /** The data payload length */,
   unsigned char *messageData  /** Pointer to the message data payload */,

   /* Output */
   int *node       /** The controller node ID of the received message */,
   int *property   /** The property this message applies to */,
   long *value     /** The value of the property being processed */)
{
   int i;
   int dataHeader;

   *node = ADDR2NODE(id);
   if (*node == -1)
      fprintf(stderr,"msgID:%x ",id);
   dataHeader = ((messageData[0] >> 6) & 0x0002) | ((id & 0x041F) == 0x0403) | ((id & 0x41F) == 0x0407);
   //messageData[0] &= 0x7F;
   //fprintf(stderr,"Entering parsemessage");
   switch (dataHeader)
   {
   case 3:  /* Data is a packed 22-bit position, SET */
      *value = 0x00000000;
      *value |= ( (long)messageData[0] << 16) & 0x003F0000;
      *value |= ( (long)messageData[1] << 8 ) & 0x0000FF00;
      *value |= ( (long)messageData[2] ) & 0x000000FF;

      if (*value & 0x00200000) /* If negative */
         *value |= 0xFFC00000; /* Sign-extend */

      if((id & 0x041F) == 0x0403)
        *property = P;
      else
        *property = JP;

      jointPosition[*node] = 0;
      if(len > 3){
        jointPosition[*node] |= ( (long)messageData[3] << 16) & 0x003F0000;
        jointPosition[*node] |= ( (long)messageData[4] << 8 ) & 0x0000FF00;
        jointPosition[*node] |= ( (long)messageData[5] ) & 0x000000FF;

        if (jointPosition[*node] & 0x00200000) /* If negative */
           jointPosition[*node] |= 0xFFC00000; /* Sign-extend */
      }

      //fprintf(stderr,"Received packed set property: %d from node: %d value:%d",*property,*node,*value);
      break;
   case 2:  /* Data is normal, SET */
      *property = messageData[0] & 0x7F;
      //fprintf(stderr, "Received property: %d", *property);
      /* Store the value, second byte of message is zero (for DSP word alignment) */
      *value = messageData[len-1] & 0x80 ? -1L : 0;
      for (i = len-1; i >= 2; i--)
         *value = *value << 8 | messageData[i];

      //fprintf(stderr, "Received normal set property: %d from node: %d value:%d", *property, *node, *value);
      //fprintf(stderr,"parsemessage after %d",value);
      break;
   case 0:  /* Assume firmware request (GET) */
         *property = -(messageData[0] & 0x7F); /* A negative (or zero) property means GET */
      *value = 0;
      //fprintf(stderr, "Received normal get property: %d from node: %d value:%d", *property, *node, *value);
      break;
   default:
         fprintf(stderr, "<Illegal Message Header> %d\n", dataHeader);
      return(1);
   }
   //if (*property != 8) fprintf(stderr,"Value in parsemessage is: %d",*value);
   return (0);

}

int getProperty(int bus, int id, int property, long *reply)
{
   int err;
   unsigned char data[8];
   int len_in;
   int id_in;
   int property_in;


   // Compile the packet
   data[0] = (unsigned char)property;

    // Send the packet
   err = canSendMsg(bus, NODE2ADDR(id), 1, data, TRUE);

   if (err) {
     fprintf(stderr,"getProperty(): canSendMsg error = %d\n", err);
     return (1);
   }

   // Wait for 1 reply
   err = canReadMsg(bus, &id_in, &len_in, data, TRUE);

   if(!err) {
     // Parse the reply
     err = parseMessage(id_in, len_in, data, &id_in, &property_in, reply);
     
     
     // Check that the id and property match
     if((id == id_in) && (property == property_in)) {
       return(0);
     } else {
       fprintf(stderr, "getProperty(): returned id or property do not match");
       fprintf(stderr, "asked for %d from %d but got %d from %d\n",
	       property, id, property_in, id_in);
       return(1);
     }
   } else {
     fprintf(stderr,"getProperty(): canReadMsg error = %d\n", err);
     return(1);
   }
}
