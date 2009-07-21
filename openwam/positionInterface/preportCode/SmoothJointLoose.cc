#include "SmoothArmLoose.h"
#include "SmoothJointLoose.h"
#include <math.h>

SmoothJointLoose::SmoothJointLoose( SmoothArm* _pArm, int _jointIndex, float startPosition) : 
  SmoothJoint( _pArm, _jointIndex, startPosition) {} 


// create a spline  
void SmoothJointLoose::createSpline(float pos0, float vel0, float* pSmoothnessMerit) {

  // manage the front of the posBuffer
  while (posBuf.size() > pArm->timeBuf.size()) {
    posBuf.pop_front();
  }

  unsigned int numTargPts = posBuf.size();
  assert(numTargPts > 0);

  int t1;
  if (numTargPts == 1) {

    C0 = pos0;
    C1 = 0.0;
    C2 = 0.0;
    C3 = 0.0;
    t1 = 0;

  } else if (numTargPts == 2) {
     
    // create a spline from pos0, vel0 to p1 at t1 ending in zero slope 
    float t1 = (int) (pArm->timeBuf.at(1) - pArm->timeBuf.at(0)); 
    float p1 = posBuf.at(1);
    createSplineSegmentType1(pos0, vel0, t1, p1); 

  } else {

    assert(numTargPts > 2);

    // create a spline from pos0, vel0 through p2 at t2 with minimum integrated accel^2
    
    // these aren't used
    float t1 = (int) (pArm->timeBuf.at(1) - pArm->timeBuf.at(0)); 
    float p1dummy = posBuf.at(1);

    float t2 = (int) (pArm->timeBuf.back() - pArm->timeBuf.at(0));
    float p2 = posBuf.back();

    createSplineSegmentType2(pos0, vel0, t1, p1dummy, t2, p2); 

    // modify p1 to be consistent with spline at t1 
    posBuf.at(1) = C0+(C1+(C2+C3*t1)*t1)*t1;
  }

  // the figure of merit for smoothness is:
  // integral(acceleration^2, dt) ftom 0 to t
  // or integral((2*C2a + 6*C3a*t)^2, dt)
  // or integral(4*C2a^2 + 24*C2a*C3a*t + 36*C3a^2*t^2, dt)
  // or 4*C2a^2*t + 12C2a*C3a*t^2 + 12*C3a^2*t^3
  *pSmoothnessMerit = (4*C2*C2 + 12*(C2*C3 + C3*C3*t1)*t1)*t1; 
}



void SmoothJointLoose::createSplineSegmentType2(float pInitial, float vInitial, 
                                           float tMid, float pMid,
                                           float tFinal, float pFinal) {
    
  // type 2: starting at time 0, go to a new position, 
  // with consideration of where we are going in the future
  C0 = pInitial;
  C1 = vInitial;

  // a spline that goes through (tFinal,pFinal) with minimum integrated acceleration^2
  C2=-3.0*(-pFinal+pInitial+vInitial*tFinal)/2.0/tFinal/tFinal;
  C3=(pFinal-pInitial-vInitial*tFinal-C2*tFinal*tFinal)/tFinal/tFinal/tFinal;

  // we guard against big overshoots
  // maxima occur at d/dt(C0+C1*t+C2*t^2+C3*t^3) = 0
  // tmaxima1 = 1/6/C3*(-2*C2+2*sqrt(C2^2-3*C3*C1))
  // tmaxima2 = 1/6/C3*(-2*C2-2*sqrt(C2^2-3*C3*C1))
  bool bDanger = false;
  float discriminant = C2*C2-3.0*C3*C1;
  float tmax;
  if (discriminant >= 0 && C3 != 0.0 ) {
    // the plus case
    tmax =  (-2.0*C2 + 2.0 * sqrt(discriminant))/6.0/C3;
    if (tmax > 0 && tmax < tMid) {
      float pmax =  C0 + (C1 + (C2 + C3*tmax) * tmax) * tmax;
      float dangerMeasure = pmax - (pInitial + (pMid-pInitial)*tmax/tMid);
      if (dangerMeasure < 0) dangerMeasure = -dangerMeasure;
      if (dangerMeasure > overshootDangerThreshold) bDanger = true;
    }
    // the minus case
    tmax =  (-2.0*C2 - 2.0 * sqrt(discriminant))/6.0/C3;
    if (tmax > 0 && tmax < tMid) {
      float pmax =  C0 + (C1 + (C2 + C3*tmax) * tmax) * tmax;
      float dangerMeasure = pmax - (pInitial + (pMid-pInitial)*tmax/tMid);
      if (dangerMeasure < 0) dangerMeasure = -dangerMeasure;
      if (dangerMeasure > overshootDangerThreshold) bDanger = true;
    }
  }
  if (bDanger) {
    // fall back to type 1
    createSplineSegmentType1(pInitial, vInitial, tMid, pMid); 
  }
}
