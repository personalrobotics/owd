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

CANbus *btinterface_bus = NULL;

void setPropertySlow(int b, int id, int prop, bool verify, long value) {
  if (! btinterface_bus) {
    throw "Must allocate a CANbus object for btinterface_bus pointer";
  }
  
  btinterface_bus->set_property_rt(id,prop,value,verify);
  usleep(100);
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
