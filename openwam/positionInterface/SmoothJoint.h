#ifndef SMOOTHJOINT_H 
#define SMOOTHJOINT_H 

#include <deque>

class SmoothArm;

class SmoothJoint
{
  public:
  
    //ctor, dtor
    SmoothJoint() {}
    SmoothJoint(SmoothArm* pArm, int jointIndex, double startPosition);
    virtual ~SmoothJoint();

    virtual void reset(double startPosition);

    virtual void createSpline(double pos0, double vel0, double* pSmoothnessMerit);

    virtual void evaluateSpline(long long time, double* pPos, double* pVel, double* pAccel=NULL);  

    virtual void appendCoarseTarg( double newP );


    virtual void splineDump(char color);
    virtual void debugDump();

  protected:
    // methods
    virtual bool isSyncWithArm();


    // type 1: starting at time 0, go to a new position, and get there with zero slope 
    void createSplineSegmentType1(double pInitial, double vInitial, 
                                  double tFinal, double pFinal);

    // type 2:  a spline that starts at pInitial, vInitial, and goes 
    // through (tMid, pMid) and (tFinal, pFinal)
    virtual void createSplineSegmentType2(double pInitial, double vInitial, 
                                          double tMid, double pMid,
                                          double tFinal, double pFinal);

    //
    // data
    // 
    SmoothArm* pArm; 
    int jointIndex; 

    // positions
    std::deque<double> posBuf;
    double maxAbsPosDelta;
    double overshootDangerFraction;
    double overshootDangerThreshold; 

    // spline coeff's
    double C0;
    double C1;
    double C2;
    double C3;
};

#endif
