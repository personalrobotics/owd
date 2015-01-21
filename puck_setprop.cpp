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

int entryLine;
const int FALSE=0;
bool curses(FALSE);
#include "btinterface.h"
#include "openwam/CANbus.hh"
#include "openwam/CANdefs.hh"
#include "btutil_changeID.h"

void usage(char *name) {
  printf("Usage: %s <bus> <puck> <property> <value>\n",name);
  printf("  <bus>:  numerical value of the CANbus device (/dev/pcan##)\n");
  printf("  <puck>: CANbus ID of the puck to query (1-14)\n");
  printf("  <property>: Numerical property identifier\n");
  printf("  <value>: Value to assign\n");
}

int main(int argc, char **argv ) {
  if (argc ==5) {
    int bus = atol(argv[1]);
    int puck = atol(argv[2]);
    int32_t property = atol(argv[3]);
    int32_t value = atol(argv[4]);
    if ((puck < 1) || (puck > 14)) {
      fprintf(stderr,"Error: puck id %d not in the range 1-14\n",puck);
      usage(argv[0]);
      exit(1);
    }

    CANbus canbus(bus,0,true,false,false,true);
    // set the global bus pointer for the btutil code
    btinterface_bus = &canbus;

    try {
      
      if (canbus.open() == OW_FAILURE) {
	fprintf(stderr,"Unable to open CANbus device\n");
	return 1;
      }
      
      if (canbus.wake_puck(puck) != OW_SUCCESS) {
	fprintf(stderr,"Unable to wake puck %d\n",puck);
	return 1;
      }
      
#ifdef GET_VERSION
      // get the puck version
      int32_t val;
      if (canbus.get_property_rt(puck,VERS,&val,20000) != OW_SUCCESS) {
	fprintf(stderr,"Unable to get VERS from puck %d\n",puck);
	return 1;
      }
      printf("Puck firmware version %d\n",val);
      canbus.initPropertyDefs(val);
#else
      canbus.initPropertyDefs(200);
#endif
      if ((property < 0) || (property >=PROP_END)) {
	fprintf(stderr,"Error: property %d out of range (0 to %d)\n",
		property, PROP_END);
	usage(argv[0]);
	exit(1);
      }
      
      // set the value
      if (property == 1) {
	// use the btutil code for changing the ROLE
	changeID(puck,puck,value); // no return value
	printf("ROLE set to %d on puck %d\n",value,puck);

      } else if (canbus.set_property_rt(puck,property, value, false, 10000)
		 != OW_SUCCESS) {
	fprintf(stderr,"Unable to set property %d=%d: %s\n",
		property, value, canbus.last_error);
	exit(1);
      }
    } catch (const char *canerr) {
      fprintf(stderr,"CANbus class threw an error: %s\n",canerr);
      return 1;
    }
    return 0;
  } else {
    usage(argv[0]);
    return 1;
  }
}
