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
#include "btutil_setMofst.h"

void usage(char *name) {
  printf("Usage: %s <bus> <puck>\n",name);
  printf("  <bus>:  numerical value of the CANbus device (/dev/pcan##)\n");
  printf("  <puck>: CANbus ID of the puck to set (1-14)\n");
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
    printf("Finding MOFST for puck %d...", puck);
    btinterface_bus = new CANbus(bus,0,true,false,false,true);

    if (btinterface_bus->open() == OW_FAILURE) {
      printf("Unable to open CANbus device");
      delete btinterface_bus;
      return 1;
    }

    // wake the puck
    if (btinterface_bus->wake_puck(puck) != OW_SUCCESS) {
      fprintf(stderr,"Unable to wake puck %d\n",puck);
      delete btinterface_bus;
      return 1;
    }
    
    int32_t val;
    if (btinterface_bus->get_property_rt(puck,VERS,&val,20000) != OW_SUCCESS) {
      fprintf(stderr,"Unable to get VERS from puck %d\n",puck);
      delete btinterface_bus;
      return 1;
    }
    btinterface_bus->initPropertyDefs(val);

    setMofst(puck);
    printf("done.\n");
    delete btinterface_bus;
    exit(0);
  } else {
    usage(argv[0]);
    exit(1);
  }
}
