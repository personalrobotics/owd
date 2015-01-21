#include "Motor.hh"
#include <stdio.h>
#include <algorithm>

int main() {
  Motor m;
  //  if (!m.load_cogging_data("/homes/vandeweg/cogging_motor_data_clipped.dat")) {
  if (!m.load_cogging_data("/homes/vandeweg/pr/trunk/src/system/systemconf/pantilt_M1_cogging_comp.dat")) {
    printf("Could not load data\n");
    return -1;
  }
  printf("Dump of cogging torque table: \n");
  for (unsigned int i=0; i<m.cogging_torques.size(); ++i) {
    printf(" a=%f t=%f\n",m.cogging_torques[i].first,
	   m.cogging_torques[i].second);
  }
  printf("\nTest of cogging torque for every motor position:\n");
  for (unsigned int a=0; a<4096; ++a) {
    try {
      printf("torque for %d is %f\n",
	     a, m.cogging_comp_torque(a/4096.0 * 2.0 * M_PI));
    } catch (...) {
      printf("no torque for a=%d (%f)\n",
	     a, a/4096.0 * 2.0 * M_PI);
    }
  }
  return 0;
  printf("torque for 0.9 is %f\n-------------------\n",
	 m.cogging_comp_torque(0.9));
  printf("torque for 1.0 is %f\n-------------------\n",
	 m.cogging_comp_torque(1.0));
  try {
    printf("torque for 2PI is %f\n-------------------\n",
	 m.cogging_comp_torque(2*M_PI));
  } catch (const int x) {
    printf("caught error\n");
  }
}
