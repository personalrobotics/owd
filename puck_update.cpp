
// #include <curses.h>
int entryLine;
const int FALSE=0;
bool curses(FALSE);
#include "btinterface.h"
#include "btutil_firmwareDL.h"

int main(int argc, char **argv ) {
  if (argc >=5) {
    char *a1 = argv[1];
    while(*a1 == '-') a1++;
    *a1 = toupper(*a1);
    char *a3 = argv[3];
    while(*a3 == '-') a3++;
    *a3 = toupper(*a3);
    if ((*a1 == 'D') && (*a3 == 'F')) {
      printf("\n\nDownload firmware to puck ID: ");
      int puck = atol(argv[2]);
      char *file = argv[4];
      btinterface_bus = new CANbus(41,0,true,false,false);
      btinterface_bus->initPropertyDefs(160);
      if (btinterface_bus->open() == OW_FAILURE) {
	printf("Unable to open CANbus device");
	return 1;
      }
      return firmwareDL(puck,file);
      printf("\n");
    }
  }  
}
