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
