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
#include "btutil_firmwareDL.h"

void usage(char *name) {
  printf("Usage: %s <bus> <puck> <file>\n",name);
  printf("  <bus>:  numerical value of the CANbus device (/dev/pcan##)\n");
  printf("  <puck>: CANbus ID of the puck to update (1-14)\n");
  printf("  <file>: name of the firmware file, e.g. puck2.tek.r174\n");
}

int main(int argc, char **argv ) {
  if (argc ==4) {
    int bus = atol(argv[1]);
    int puck = atol(argv[2]);
    char *file = argv[3];
    if ((puck < 1) || (puck > 14)) {
      printf("Error: puck id %d not in the range 1-14\n",puck);
      usage(argv[0]);
      exit(1);
    }
    printf("Downloading firmware %s to puck %d...",
	   file, puck);
    btinterface_bus = new CANbus(bus,0,true,false,false);
    btinterface_bus->initPropertyDefs(160);
    if (btinterface_bus->open() == OW_FAILURE) {
      printf("Unable to open CANbus device");
      return 1;
    }
    return firmwareDL(puck,file);
    printf("\n");
  } else {
    usage(argv[0]);
    exit(1);
  }
}
