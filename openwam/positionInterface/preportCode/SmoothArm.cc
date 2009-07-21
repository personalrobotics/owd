#include "SmoothJoint.h"
#include "SmoothArm.h"
#include <assert.h>
#include <stdio.h>
SmoothArm::SmoothArm() {
}


SmoothArm::SmoothArm( long long startTime, int lagTime, const std::vector<float> &startPosVec) {
  numJoints = 0;
  reset(startTime, lagTime, startPosVec);
}


void SmoothArm::reset(long long startTime, int _lagTime, const std::vector<float> &startPosVec) {

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
    jointVec.push_back(new SmoothJoint(this, i, startPosVec.at(i)));
  }
}

SmoothArm::~SmoothArm() {
  for (unsigned int i=0; i<numJoints; i++) {
    delete jointVec.at(i);
  }
}

void SmoothArm::getSmoothedPos( long long curTime, std::vector<float>* pSmoothPosVec ) {
  std::vector<float> unusedVelVec;  
  getSmoothedState(curTime, pSmoothPosVec, &unusedVelVec);
}


void SmoothArm::getSmoothedState(long long curTime, 
		            std::vector<float>* pPosVec, 
			    std::vector<float>* pVelVec) {

  long long laggedTime = curTime - lagTime;
  float pos;
  float vel;

  while (timeBuf.size() > 1 && laggedTime >= timeBuf.at(1) ) {
    // we need to slide down
    curPosVec.clear();
    curVelVec.clear();
    for (unsigned int i = 0; i<numJoints; i++) {
      SmoothJoint* pJoint = jointVec.at(i);
      pJoint->evaluateSpline(timeBuf.at(1), &pos, &vel);
      curPosVec.push_back(pos);
      curVelVec.push_back(vel);
    }

    timeBuf.pop_front();
    doArmSpline();
  }

  pPosVec->clear();
  pVelVec->clear();
  for (unsigned int i = 0; i<numJoints; i++) {
    SmoothJoint* pJoint = jointVec.at(i);
    pJoint->evaluateSpline(curTime - lagTime, &pos, &vel);
    pPosVec->push_back(pos);
    pVelVec->push_back(vel);
  }
}


void SmoothArm::doJointSplines(float* pSmoothnessMerit) {

  assert(curPosVec.size() == numJoints);
  assert(curVelVec.size() == numJoints);

  
  float jointMerit;
  float totalMerit = 0.0;
  for (unsigned int i = 0; i<numJoints; i++) {
    SmoothJoint* pJoint = jointVec.at(i);
    pJoint->createSpline(curPosVec.at(i), curVelVec.at(i), &jointMerit);
    totalMerit += jointMerit;
  }
  *pSmoothnessMerit = totalMerit;
}


void SmoothArm::setCoarseTarg(long long curTime, const std::vector<float> &targPosVec) {

  assert(targPosVec.size() == numJoints);
  
  // early out if no time has passed since last call
  if ( timeBuf.size() > 0 && curTime == timeBuf.back() ) {
    printf("Warning: SmoothArm::setCoarseTarg called with zero time increment\n");
    return;
  }

  // evaluate state at curTime 
  getSmoothedState(curTime, &curPosVec, &curVelVec);

  // modify the front side of the time buffer
  long long laggedTime = curTime - lagTime;
  // front pop all of the times that come before newTime0 
  // (within a tolerance to avoid points that are too close to each other)
  int tolDen = 100;
  long long tol;
  if (timeBuf.size() > 1) {
    tol = (timeBuf.at(1) - timeBuf.at(0)) / tolDen;
  } else {
    tol = 0;
  }
  while ( !timeBuf.empty()  && timeBuf.at(0) <= laggedTime + tol) {
    if (timeBuf.size() > 1) { 
      tol = ( timeBuf.at(1) - timeBuf.at(0) ) / tolDen;
    } else {
      tol = 0;
    }
    timeBuf.pop_front();
  }
  timeBuf.push_front(laggedTime);

  // back push curTime onto buffer 
  timeBuf.push_back(curTime);

  // now update joints
  for (unsigned int i = 0; i<numJoints; i++) {
    SmoothJoint* pJoint = jointVec.at(i);
    pJoint->appendCoarseTarg(targPosVec.at(i));
  }

  doArmSpline();

  #ifdef DEBUG
  // LLL debug
  printf("\n%%Final dumps\n");
  jointVec.at(0)->splineDump('b');
  jointVec.at(0)->debugDump();
  #endif
} 



void SmoothArm::doArmSpline() {
  
  float meritTry;
  doJointSplines(&meritTry);
}
