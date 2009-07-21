#include "SmoothJoint.h"
#include "SmoothArm.h"
#include <assert.h>
#include <string>
#include <stdio.h>
#include <sys/time.h>
#include <ros/node.h>
#include <unistd.h>

SmoothArm::SmoothArm() {
  state = INVALID;
  lagTime = 150000; //uSec
  setupSpeedLimits();
}


SmoothArm::SmoothArm( const std::vector<double> &startPosVec) {
  reset(startPosVec);
}


void SmoothArm::reset(const std::vector<double> &startPosVec) {

  long long startTime = getCurTimeUSec();

  timeBuf.clear();
  timeBuf.push_back(startTime-lagTime);

  for (unsigned int i=0; i<jointVec.size(); i++) {
    delete jointVec.at(i);
  }
  jointVec.clear();

  // initialize curPosVec, curVelVec
  completePosVecSize = startPosVec.size();
  curPosVec.clear();
  curVelVec.clear();
  for (unsigned int i = 0; i<completePosVecSize; i++) {
    curPosVec.push_back(startPosVec.at(i));
    curVelVec.push_back(0.0);
  }

  // initialize pendingReset
  pendingResetTarg.bPending = false;

  // initialize pendingCoarseTarg
  pendingCoarseTarg.bPending = false;
  pendingCoarseTarg.completeTargPosVec = startPosVec;

  // presumably this pos is safe
  lastSafePosVec = startPosVec;

  for (unsigned int i=0; i<completePosVecSize; i++) {
    jointVec.push_back(new SmoothJoint(this, i, startPosVec.at(i)));
  }

  state=VALID;
}

void SmoothArm::reset(const double* startPosArray, int size)
{
  std::vector<double> startPosVec;
  for(int j=0; j<size; j++) {
    startPosVec.push_back(startPosArray[j]);
  }
  this->reset(startPosVec);
}
  


SmoothArm::~SmoothArm() {
  for (unsigned int i=0; i<completePosVecSize; i++) {
    delete jointVec.at(i);
  }
}

void SmoothArm::getSmoothedPVA( std::vector<double>* pSmoothPosVec,
                                std::vector<double>* pSmoothVelVec,
                                std::vector<double>* pSmoothAccelVec ) {

  processPendingReset();

  // this could be dangerous if we are not in a valid state
  // TODO do something besides just crash (what?) 
  assert(state == VALID);

  processPendingCoarseTarg();

  std::vector<double> velVec;  
  if (pSmoothVelVec == NULL) {
    pSmoothVelVec = &velVec;
  }
  std::vector<double> accelVec;  
  if (pSmoothVelVec == NULL) {
    pSmoothAccelVec = &accelVec;
  }
  
  long long curTime = getCurTimeUSec();
  getSmoothedState(curTime, pSmoothPosVec,  pSmoothVelVec, pSmoothAccelVec);

  // verify that velocity limits have not been exceeded
  if (jointSpeedLimitExceededVec.size() < completePosVecSize) {
    jointSpeedLimitExceededVec.assign(completePosVecSize, false);
  }
  bool bSafe = true; 
  for (unsigned int i=0; i<completePosVecSize; i++) { 
    // Note: velVec units are radians per uSec
    double velVecRadPerSec = pSmoothVelVec->at(i)*1000000;
    if (velVecRadPerSec < 0.0) velVecRadPerSec = -velVecRadPerSec;
    if (velVecRadPerSec > jointSpeedLimitVec.at(i)) {
      bSafe = false;
      jointSpeedLimitExceededVec.at(i) = true;
    }
  }
  if (bSafe == true) {
    lastSafePosVec = *pSmoothPosVec;
  } else {
    *pSmoothPosVec = lastSafePosVec;
    // TODO do we need to set pSmoothVelVec, and pSmoothAccelVec?
    reset(lastSafePosVec);
  }
}


void SmoothArm::getSmoothedPVA( double* smoothPosArray,
                                double* smoothVelArray,
                                double* smoothAccelArray ) {

  std::vector<double> smoothPosVec;
  std::vector<double> smoothVelVec;
  std::vector<double> smoothAccelVec;
  getSmoothedPVA(&smoothPosVec, &smoothVelVec, &smoothAccelVec);

  assert(smoothPosVec.size() == smoothVelVec.size());
  assert(smoothPosVec.size() == smoothAccelVec.size());
  for (unsigned int i = 0; i<smoothPosVec.size(); i++) {
    smoothPosArray[i] = smoothPosVec.at(i);
    if (smoothVelArray != NULL) {
      smoothVelArray[i] = smoothVelVec.at(i);
    }
    if (smoothAccelArray != NULL) {
      smoothAccelArray[i] = smoothAccelVec.at(i);
    }
  }
}

void SmoothArm::getSmoothedState(long long curTime, 
                                 std::vector<double>* pPosVec, 
                                 std::vector<double>* pVelVec,
                                 std::vector<double>* pAccelVec ) {

  long long laggedTime = curTime - lagTime;
  double pos;
  double vel;
  double accel;

  while (timeBuf.size() > 1 && laggedTime >= timeBuf.at(1) ) {
    // we need to slide down
    curPosVec.clear();
    curVelVec.clear();
    for (unsigned int i = 0; i<completePosVecSize; i++) {
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
  for (unsigned int i = 0; i<completePosVecSize; i++) {
    SmoothJoint* pJoint = jointVec.at(i);
    pJoint->evaluateSpline(curTime - lagTime, &pos, &vel, &accel);
    pPosVec->push_back(pos);
    pVelVec->push_back(vel);
    if (pAccelVec != NULL) {
      pAccelVec->push_back(accel);
    }
  }
}


void SmoothArm::doJointSplines(double* pSmoothnessMerit) {

  if (curPosVec.size() != completePosVecSize) {
    printf("curPosVec.size(): %d\n", curPosVec.size());
    printf("completePosVecSize: %d\n", completePosVecSize);
  }
  assert(curPosVec.size() == completePosVecSize);
  assert(curVelVec.size() == completePosVecSize);

  
  double jointMerit;
  double totalMerit = 0.0;
  for (unsigned int i = 0; i<completePosVecSize; i++) {
    SmoothJoint* pJoint = jointVec.at(i);
    pJoint->createSpline(curPosVec.at(i), curVelVec.at(i), &jointMerit);
    totalMerit += jointMerit;
  }
  *pSmoothnessMerit = totalMerit;
}

void SmoothArm::invalidate() {
  state = INVALID;
}


bool SmoothArm::isValid() {
  return state == VALID;
}


void SmoothArm::setLagTime(int lagTimeUSec) {

  //early out 
  if (state != INVALID) {
    ROS_WARN("Attempted to set lagTime on a non-stopped smoother\n");
    ROS_WARN("You must stop the smoother to make this setting change\n");
    return;
  }

  lagTime = lagTimeUSec;
}


void SmoothArm::setReset() {
  state = PENDING_RESET;
}


void SmoothArm::setCoarseTarg( const std::vector<int> &jointIndicesVec,
                               const std::vector<double> &partialTargPosVec) {

  long long curTime = getCurTimeUSec();

  reportStatus();

  //early out 
  if (state == INVALID) {
    ROS_WARN("Attempted to setCoarseTarg on an INVALID smoother\n");
    return;
  }

  while (pendingCoarseTarg.bPending == true) {
    // wait for real time thread to consume the pendingCoarseTarg
    // it shouldn't take long...
    usleep(1000);
  }

  assert(partialTargPosVec.size() == jointIndicesVec.size());
  unsigned int n = jointIndicesVec.size();
  // reuse pendingCoarseTarg.completeTargPosVec, changing only the specified joints
  for (unsigned int i=0; i<n; i++) {
    int index = jointIndicesVec.at(i);
    assert (index >= 0);
    assert (index < completePosVecSize);
    pendingCoarseTarg.completeTargPosVec.at(index) = partialTargPosVec.at(i);
  }
  pendingCoarseTarg.timeStamp = curTime;
  pendingCoarseTarg.bPending = true;
}

void SmoothArm::setCoarseTarg(const std::vector<double> &completeTargPosVec) {
  long long curTime = getCurTimeUSec();

  reportStatus();

  //early out 
  if (state == INVALID) {
    ROS_WARN("Attempted to setCoarseTarg (2) on a INVALID smoother\n");
    return;
  }

  while (pendingCoarseTarg.bPending == true) {
    // wait for real time thread to consume the pendingCoarseTarg
    // it shouldn't take long...
    usleep(1000);
  }
  pendingCoarseTarg.timeStamp = curTime;
  pendingCoarseTarg.completeTargPosVec = completeTargPosVec;
  pendingCoarseTarg.bPending = true;
}


void SmoothArm::reportStatus() {
  // report status outside of realtime thread 
  for (unsigned int i=0; i<jointSpeedLimitExceededVec.size(); i++) {
    if (jointSpeedLimitExceededVec.at(i) == true) {
      ROS_WARN("jointSpeedLimit[%d] exceeded", i);
      jointSpeedLimitExceededVec.at(i) = false;
    }
  }
}

void SmoothArm::processPendingReset() {
  if (pendingResetTarg.bPending == true) {
    reset(pendingResetTarg.completeTargPosVec);
    pendingResetTarg.bPending = false;
  }
}

void SmoothArm::processPendingCoarseTarg() {

  // early out if pendingCoarseTarg is invalid
  if (pendingCoarseTarg.bPending == false) {
    return;
  }

  // early out if no time has passed since last call
  if ( timeBuf.size() > 0 && pendingCoarseTarg.timeStamp == timeBuf.back() ) {
    // not a good idea to print in a real time thread
    //printf("Warning: SmoothArm::processPendingCoarseTarg called with zero time increment\n");
    return;
  }

  assert(pendingCoarseTarg.completeTargPosVec.size() == completePosVecSize);

  long long laggedTime = pendingCoarseTarg.timeStamp - lagTime;
  // laggedTime is a lower limit to the required buffering.
  // However, it is possible that a higher lower bound has already been
  // discovered, by e.g. getSmoothedState.  
  if ( laggedTime > timeBuf.at(0) ) {
    // a new lower limit has been established

    // evaluate state at pendingCoarseTarg.timeStamp 
    getSmoothedState(pendingCoarseTarg.timeStamp, &curPosVec, &curVelVec);

    // modify the front side of the time buffer
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
  }

  // back push pendingCoarseTarg.timeStamp onto buffer 
  timeBuf.push_back(pendingCoarseTarg.timeStamp);

  // now update joints
  for (unsigned int i = 0; i<completePosVecSize; i++) {
    SmoothJoint* pJoint = jointVec.at(i);
    pJoint->appendCoarseTarg(pendingCoarseTarg.completeTargPosVec.at(i));
  }

  doArmSpline();

  #ifdef DEBUG
  // LLL debug
  printf("\n%%Final dumps\n");
  jointVec.at(0)->splineDump('b');
  jointVec.at(0)->debugDump();
  #endif

  pendingCoarseTarg.bPending = false;
} 


void SmoothArm::setupSpeedLimits(){

  // Note: the rest of this module is agnostic to the number of joint
  // and/or joint numbering conventions.  This function, however, makes
  // the assumption that there are 8 joint elements, with the 0th joint 
  // being unused, thus making the vector indices aligned with the joint
  // indices

  jointSpeedLimitVec.clear();
  

  double speedLimit;

  // retrieve speed limits from ros param server, if available
  ros::Node* node = ros::Node::instance();

  // unused J0
  jointSpeedLimitVec.push_back(0.0);

  // J1
  node->param(std::string("owd/positionInterface/speedLimitJ1"), speedLimit, 2.0);
  ROS_INFO("positionInterface/speedLimitJ1 %f", speedLimit);
  jointSpeedLimitVec.push_back(speedLimit); 

  // J2
  node->param(std::string("owd/positionInterface/speedLimitJ2"), speedLimit, 3.0);
  ROS_INFO("positionInterface/speedLimitJ2 %f", speedLimit);
  jointSpeedLimitVec.push_back(speedLimit); 

  // J3
  node->param(std::string("owd/positionInterface/speedLimitJ3"), speedLimit, 3.0);
  ROS_INFO("positionInterface/speedLimitJ3 %f", speedLimit);
  jointSpeedLimitVec.push_back(speedLimit); 

  // J4
  node->param(std::string("owd/positionInterface/speedLimitJ4"), speedLimit, 3.0);
  ROS_INFO("positionInterface/speedLimitJ4 %f", speedLimit);
  jointSpeedLimitVec.push_back(speedLimit); 

  // J5
  node->param(std::string("owd/positionInterface/speedLimitJ5"), speedLimit, 5.0);
  ROS_INFO("positionInterface/speedLimitJ5 %f", speedLimit);
  jointSpeedLimitVec.push_back(speedLimit); 

  // J6
  node->param(std::string("owd/positionInterface/speedLimitJ6"), speedLimit, 5.0);
  ROS_INFO("positionInterface/speedLimitJ6 %f", speedLimit);
  jointSpeedLimitVec.push_back(speedLimit); 

  // J7
  node->param(std::string("owd/positionInterface/speedLimitJ7"), speedLimit, 5.0);
  ROS_INFO("positionInterface/speedLimitJ7 %f", speedLimit);
  jointSpeedLimitVec.push_back(speedLimit); 
}



long long SmoothArm::getCurTimeUSec() {
  timeval curtv;
  gettimeofday( &curtv, NULL );
  long long curTimeUSec = (long long)(curtv.tv_sec)*1000000 + (long long)(curtv.tv_usec);
  return curTimeUSec;
}


void SmoothArm::doArmSpline() {
  
  double meritTry;
  doJointSplines(&meritTry);
}
