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
int print_properties(int puck, CANbus *bus);

void usage(char *name) {
  printf("Usage: %s <bus> <puck>\n",name);
  printf("  <bus>:  numerical value of the CANbus device (/dev/pcan##)\n");
  printf("  <puck>: CANbus ID of the puck to query (1-14)\n");
}

int main(int argc, char **argv ) {
  if (argc ==3) {
    int bus = atol(argv[1]);
    int puck = atol(argv[2]);
    if ((puck < 1) || (puck > 14)) {
      printf("Error: puck id %d not in the range 1-14\n",puck);
      usage(argv[0]);
      exit(1);
    }
    printf("Property values for puck %d:\n", puck);
    CANbus *canbus = new CANbus(bus,0,true,false,false,true);

    if (canbus->open() == OW_FAILURE) {
      printf("Unable to open CANbus device\n");
      return 1;
    }
    int result=print_properties(puck,canbus);
    delete canbus;
    return result;
  } else {
    usage(argv[0]);
    exit(1);
  }
}

int print_properties(int id, CANbus *bus) {
  VERS = 0;
  STAT = 5;

  // wake the puck
  if (bus->wake_puck(id) != OW_SUCCESS) {
    fprintf(stderr,"Unable to wake puck %d\n",id);
    return 1;
  }
  
  int32_t val;
  if (bus->get_property_rt(id,VERS,&val,20000) != OW_SUCCESS) {
    fprintf(stderr,"Unable to get VERS from puck %d\n",id);
    return 1;
  }
  bus->initPropertyDefs(val);

  for (int i=0; i<PROP_END; ++i) {
    if (bus->get_property_rt(id,i,&val,20000) != OW_SUCCESS) {
      fprintf(stderr,"No response for property %d\n",i);
      continue;
    }
    if (CANbus::propname.find(i) != CANbus::propname.end()) {
      printf("%s=%d\n",CANbus::propname[i].c_str(), val);
    } else {
      printf("%d=%d\n",i, val);
    }
  }
  return 0;
}
