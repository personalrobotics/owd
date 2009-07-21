#ifndef SMOOTHJOINTLOOSE_H 
#define SMOOTHJOINTLOOSE_H 

#include "SmoothJoint.h"

class SmoothJointLoose : public SmoothJoint
{
  public:
  
    //ctor, dtor
    SmoothJointLoose() {}
    SmoothJointLoose(SmoothArm* pArm, int jointIndex, float startPosition);

    virtual void createSpline(float pos0, float vel0, float* pSmoothnessMerit);

  protected:
    void createSplineSegmentType2(float pInitial, float vInitial, 
                                  float tMid, float pMid,
                                  float tFinal, float pFinal);
};

#endif
