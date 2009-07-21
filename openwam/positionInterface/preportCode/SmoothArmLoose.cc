#include "SmoothArmLoose.h"
#include "SmoothJointLoose.h"

SmoothArmLoose::SmoothArmLoose() {
}

SmoothArmLoose::SmoothArmLoose( long long startTime, int lagTime, const 
                                std::vector<float> &startPosVec) 
  : SmoothArm( startTime, lagTime, startPosVec) {} 


void SmoothArmLoose::reset(long long startTime, int _lagTime, const std::vector<float> &startPosVec) {

  assert(_lagTime > 0);
  lagTime = _lagTime;

  timeBuf.clear();
  timeBuf.push_back(startTime-lagTime);


  for (unsigned int i=0; i<numJoints; i++) {
    delete jointVec.at(i);
  }
  jointVec.clear();

  numJoints = startPosVec.size();

  for (unsigned int i=0; i<numJoints; i++) {
    jointVec.push_back(new SmoothJointLoose(this, i, startPosVec.at(i)));
  }
}
