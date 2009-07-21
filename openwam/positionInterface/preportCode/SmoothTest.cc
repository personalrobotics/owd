#include "SmoothArm.h"
#include "TPSTime.h"
#include <vector>

#define N_TARG 10 

int main(void) {

  int coarseTargTimes[N_TARG] =  
  {
   73637163, 73677462, 73718134, 73743879, 73812434, 73859885, 73919697, 73964375, 74010895, 74050165,
  };


  float targPos[N_TARG] = 
  { 
   1.000000, 2.000000, 3.000000, 4.000000, 5.000000, 6.000000, 7.000000, 8.000000, 9.000000, 10.000000,
  };

  int i_targ;
  int hostTime;
  TPSTimeType hostCurTime;

  std::vector<float> targVec;
  //player_wamarm_jointset smoothSet;

  printf("targT=[];\n");
  printf("targP=[];\n");

  SmoothArm mySmoothArm;

  i_targ = 0;
  hostTime = coarseTargTimes[i_targ];
  hostCurTime.tv_sec = hostTime/1000000;
  hostCurTime.tv_usec = hostTime%1000000;
  targVec.clear();
  targVec.push_back(targPos[i_targ]);
  mySmoothArm.reset(hostCurTime, targVec); 

  for (i_targ = 1; i_targ < N_TARG; i_targ++) {
    hostTime = coarseTargTimes[i_targ];
    hostCurTime.tv_sec = hostTime/1000000;
    hostCurTime.tv_usec = hostTime%1000000;
    targVec.clear();
    targVec.push_back(targPos[i_targ]);
    mySmoothArm.setCoarseTarg(hostCurTime, targVec); 

    printf("targT=[targT, %d];\n", hostTime);
    printf("targP=[targP,%f];\n", targPos[i_targ] );
  }
}
