#include "SmoothArmJitter.h"
#include "SmoothJoint.h"

SmoothArmJitter::SmoothArmJitter() {
}

SmoothArmJitter::SmoothArmJitter( long long startTime, int lagTime, const 
                                  std::vector<float> &startPosVec) 
  : SmoothArm( startTime, lagTime, startPosVec) {} 


#include <sys/time.h>
void SmoothArmJitter::doArmSpline() {

  float meritTry;
  doJointSplines(&meritTry);
  // early out
  if (timeBuf.size() < 3) {
    return;
  }
  
  // do a search for the best t1...
  //
  long long timeBase = timeBuf.at(1);

  long long upperBound = timeBuf.at(1);
  long long lowerBound = timeBuf.at(0) + 1;;
  long long range = upperBound - lowerBound;

  // initialize best with base  
  long long timeTry = timeBase;
  long long timeBest = timeTry;
  float meritBest = meritTry;


  // timeTry from lowerBound to upperBound
  unsigned int numEval = 4;
  // special cases
  if (range <= 0) {
    // we can't do anything
    return;
  } else if (range < numEval) {
    // the maximum number of evals is: 
    numEval = range;
  } 

  for (unsigned int i = 1; i<numEval; i++) {
    timeTry = lowerBound + ( range * i )/numEval;
    assert(timeTry < upperBound);

    timeBuf.at(1) = timeTry;
    doJointSplines(&meritTry);

    if (meritTry < meritBest) {
      timeBest = timeTry;
      meritBest = meritTry;
    }

    #ifdef DEBUG 
    printf("%%lowerBound: %lld, upperBound %lld, timeTry: %lld, meritTry: %e, timeBest: %lld, meritBest: %e \n", 
              lowerBound,       upperBound,      timeTry,       meritTry,     timeBest,       meritBest);
    #endif
  }

  // now we have the best, so set it and be done
  timeBuf.at(1) = timeBest;
  doJointSplines(&meritTry);

  #ifdef DEBUG 
  // LLL debug
  SmoothJoint* pJoint = jointVec.at(0);
  printf("\n%%Modified dumps\n");
  pJoint->splineDump('r');
  pJoint->debugDump();
  #endif
}
