#include "SmoothJoint.h"
#include "SmoothArm.h"
#include <limits.h>
#include <assert.h>
#include <iostream>
#include <math.h>

SmoothJoint::SmoothJoint( SmoothArm* _pArm, int _jointIndex, float startPosition) {

  assert(_pArm != NULL);

  pArm = _pArm;
  jointIndex = _jointIndex;

  reset(startPosition);
}

void SmoothJoint::reset(float startPosition) {

  posBuf.clear();
  posBuf.push_back(startPosition);

  maxAbsPosDelta = 0.0;
  overshootDangerFraction = 0.1;
  overshootDangerThreshold = maxAbsPosDelta*overshootDangerFraction;

  C0 = startPosition;
  C1 = 0.0;
  C2 = 0.0;
  C3 = 0.0;
}


SmoothJoint::~SmoothJoint() {
}


void SmoothJoint::evaluateSpline(long long time, float* pPos, float* pVel) {

  assert(!pArm->timeBuf.empty());

  long long tll = time - pArm->timeBuf.at(0);
  assert(tll >= 0);
  float t = (float)tll;

  *pPos = C0 + (C1 + (C2 + C3 * t) * t) * t;
  *pVel = C1 + (2*C2 + 3*C3 * t) * t;
}


void SmoothJoint::appendCoarseTarg(float newP) {

  posBuf.push_back(newP);
  if (posBuf.size() >= 2) {
    float absPosDelta = posBuf.back() - posBuf.at(posBuf.size() - 2);
    if (absPosDelta < 0) absPosDelta = -absPosDelta;
    if (absPosDelta > maxAbsPosDelta) {
      maxAbsPosDelta = absPosDelta;
      overshootDangerThreshold = maxAbsPosDelta*overshootDangerFraction;
    }
  }

  // manage the front of the posBuffer
  while (posBuf.size() > pArm->timeBuf.size()) {
    posBuf.pop_front();
  }
} 




bool SmoothJoint::isSyncWithArm() {


  bool bResult = (posBuf.size() == pArm->timeBuf.size());
    
  return bResult;
}

// create a spline  
void SmoothJoint::createSpline(float pos0, float vel0, float* pSmoothnessMerit) {

  
  // manage the front of the posBuffer
  while (posBuf.size() > pArm->timeBuf.size()) {
    posBuf.pop_front();
  }

  unsigned int numTargPts = posBuf.size();
  assert(numTargPts > 0);

  float t1;
  if (numTargPts == 1) {

    C0 = pos0;
    C1 = 0.0;
    C2 = 0.0;
    C3 = 0.0;
    t1 = 0.0;

  } else if (numTargPts == 2) {
     
    // create a spline from pos0, vel0 to p1 at t1 ending in zero slope 
    t1 = (float) (pArm->timeBuf.at(1) - pArm->timeBuf.at(0));
    float p1 = posBuf.at(1);
    createSplineSegmentType1(pos0, vel0, t1, p1); 

  } else {

    assert(numTargPts > 2);

    // create a spline from pos0, vel0 though p1 at t1 and p2 at t2 
    t1 = (float) (pArm->timeBuf.at(1) - pArm->timeBuf.at(0)); 
    float p1 = posBuf.at(1);

    float t2 = (float) (pArm->timeBuf.at(2) - pArm->timeBuf.at(0));
    float p2 = posBuf.at(2);

    createSplineSegmentType2(pos0, vel0, t1, p1, t2, p2); 

  }

  // the figure of merit for smoothness is:
  // integral(acceleration^2, dt) ftom 0 to t
  // or integral((2*C2a + 6*C3a*t)^2, dt)
  // or integral(4*C2a^2 + 24*C2a*C3a*t + 36*C3a^2*t^2, dt)
  // or 4*C2a^2*t + 12C2a*C3a*t^2 + 12*C3a^2*t^3
  *pSmoothnessMerit = (4*C2*C2 + 12*(C2*C3 + C3*C3*t1)*t1)*t1; 
}


void SmoothJoint::createSplineSegmentType1(float pInitial, float vInitial, 
                                           float tFinal, float pFinal ) {
  // type 1: starting at time 0, go to a new position, and get there with zero slope 

  C0 = pInitial;
  C1 = vInitial;
  C2 = ((3*(pFinal-pInitial)/tFinal) - (2*vInitial))/tFinal;
  C3 = (2*(pInitial-pFinal)/tFinal + vInitial)/tFinal/tFinal;

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
    if (tmax > 0 && tmax < tFinal) {
      float pmax =  C0 + (C1 + (C2 + C3*tmax) * tmax) * tmax;
      float dangerMeasure = pmax - (pInitial + (pFinal-pInitial)*tmax/tFinal);
      if (dangerMeasure < 0) dangerMeasure = -dangerMeasure;
      if (dangerMeasure > overshootDangerThreshold) bDanger = true;
    }
    // the minus case
    tmax =  (-2.0*C2 - 2.0 * sqrt(discriminant))/6.0/C3;
    if (tmax > 0 && tmax < tFinal) {
      float pmax =  C0 + (C1 + (C2 + C3*tmax) * tmax) * tmax;
      float dangerMeasure = pmax - (pInitial + (pFinal-pInitial)*tmax/tFinal);
      if (dangerMeasure < 0) dangerMeasure = -dangerMeasure;
      if (dangerMeasure > overshootDangerThreshold) bDanger = true;
    }
  }
  if (bDanger) {
    // fall back to linear 
    C0 = pInitial;
    C1 = (pFinal-pInitial)/tFinal;
    C2 = 0;
    C3 = 0;
  }
}

void SmoothJoint::createSplineSegmentType2(float pInitial, float vInitial, 
                                           float tMid, float pMid,
                                           float tFinal, float pFinal) {
    
  // type 2: starting at time 0, go to a new position, 
  // with consideration of where we are going in the future
  C0 = pInitial;
  C1 = vInitial;

  // a spline that starts at pInitial, with slope vInital
  // and goes through (tMid, pMid) and (tFinal,pFinal)
  float tFinal2 = (float)tFinal * (float)tFinal;
  float tFinal3 = tFinal2 * (float)tFinal;
  float tMid2 = (float)tMid * (float)tMid;
  float tMid3 = tMid2 * (float)tMid;
  float den = tMid2*tFinal2*(float)(tMid-tFinal);

  //a0 = -((v0*t2-p2+p0)*t1^3-t2^3*v0*t1+(p1-p0)*t2^3)/t2^2/t1^2/(t1-t2);
  //*pC2 = -((v0*t2-p2+p0)*t1^3 + (p1-p0-v0*t1)*t2^3)/den;
  C2 = -((vInitial*tFinal-pFinal+pInitial)*tMid3 + (pMid-pInitial-vInitial*tMid)*tFinal3)/den;

  //a0 = ((p1-p0-v0*t1)*t2^2+t1^2*v0*t2+(-p2+p0)*t1^2)/t2^2/t1^2/(t1-t2); 
  //*pC3 = ((p1-p0-v0*t1)*t2^2  + (-p2+p0+v0*t2)*t1^2)/den; 
  C3 = ((pMid-pInitial-vInitial*tMid)*tFinal2  + (-pFinal+pInitial+vInitial*tFinal)*tMid2)/den; 


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


void SmoothJoint::debugDump() {

  printf("%%SmoothJoint Debug dump\n");
  printf("%%-------------------------------\n");
  printf("%%JointIndex: %d\n", jointIndex);
  printf("\n");

  printf("%%Arm times: \n");
  printf("%%pArm->lagTime: %d\n", pArm->lagTime);
  for (unsigned int i=0; i<pArm->timeBuf.size(); i++) {
    printf("%%pArm->timeBuf.at(%d): %lld\n", i, pArm->timeBuf.at(i));
  }
  printf("\n");

  printf("%%Joint target positions: \n");
  for (unsigned int i=0; i<posBuf.size(); i++) {
    printf("%%posBuf.at(%d): %f\n", i, posBuf.at(i));
  }
  printf("\n");

  printf("%%Joint spline: \n");
  printf("%%spline data: C0 %e C1 %e C2 %e C3 %e\n", C0, C1, C2, C3);
  printf("\n");
}



void SmoothJoint::splineDump(char color) {

  printf("[t,p]=plotSpline(%lld, %lld, %e, %e, %e, %e);\n", 
                  pArm->timeBuf.at(0), pArm->timeBuf.at(1), 
                                           C0, C1, C2, C3); 
  printf("plot(t,p,'%c');\n", color); 
  printf("hold on;\n\n"); 
  printf("plot(%lld, %f, '%cx');\n", pArm->timeBuf.at(1), posBuf.at(1), color);
  printf("plot(%lld, %f, '%cx');\n", pArm->timeBuf.back(), posBuf.back(), color);
  printf("\n");
}
