#ifndef SMOOTHJOINT_H 
#define SMOOTHJOINT_H 

#include <deque>

class SmoothArm;

class SmoothJoint
{
  public:
  
    //ctor, dtor
    SmoothJoint() {}
    SmoothJoint(SmoothArm* pArm, int jointIndex, float startPosition);
    virtual ~SmoothJoint();

    virtual void reset(float startPosition);

    virtual void createSpline(float pos0, float vel0, float* pSmoothnessMerit);

    virtual void evaluateSpline(long long time, float* pPos, float* pVel);  

    virtual void appendCoarseTarg( float newP );


    virtual void splineDump(char color);
    virtual void debugDump();

  protected:
    // methods
    virtual bool isSyncWithArm();


    // type 1: starting at time 0, go to a new position, and get there with zero slope 
    void createSplineSegmentType1(float pInitial, float vInitial, 
                                  float tFinal, float pFinal);

    // type 2:  a spline that starts at pInitial, vInitial, and goes 
    // through (tMid, pMid) and (tFinal, pFinal)
    virtual void createSplineSegmentType2(float pInitial, float vInitial, 
                                          float tMid, float pMid,
                                          float tFinal, float pFinal);

    //
    // data
    // 
    SmoothArm* pArm; 
    int jointIndex; 

    // positions
    std::deque<float> posBuf;
    float maxAbsPosDelta;
    float overshootDangerFraction;
    float overshootDangerThreshold; 

    // spline coeff's
    float C0;
    float C1;
    float C2;
    float C3;
};

#endif
