#ifndef SMOOTHARM_H 
#define SMOOTHARM_H 

#include <vector>
#include <deque>
#include <assert.h>

// Algorithm description:
// 1) At startup:
//  - The initial position and lag time are specified.
//  - The target point queue is initialized with (curTime - lagTime, posInit).
//  -  A trivial constant=posInit spline is constructed.
// 
// 2) When we need the current smoothed state:
//  - It is verified that the spline is valid for (current time - lagTime).
//    If not, the first point in the queue is popped, and the spline is updated.
//  - The spline is evaluated at (current time - lagTime) and the state
//    is returned
// 
// 3) When a new target point arrives:
//  - It is placed in the queue,  time stamped with the current
//    time (not lagged).
//  - The current smoothed state is evaluated.
//  - All points in the queue that come before curTime - lagTime
//    are popped off.
//  - The spline is updated.  The new spline interval starts
//    at curTime - lagTime.
// 
// 4) When we need to up update the spline:
//  a)
//     - If there is only one point in the queue, then the spline is
//       a trivial constant spline that is valid forever
//  b)
//      - If there are two points in the queue, we create a spline that
//        goes from the current state to the second point in the queue,
//        ending with zero slope.
//      - This spline is valid to the time of the second point.
//  c)
//      - If there are more than two points in the queue, we create 
//        splines that go from the current state, through the second
//        point and through the third point.
//      - Dangerous splines that oveshoot the target points are detected 
//        and eliminated.
//      - This spline is valid to the time of the second point.


class SmoothJoint;

class SmoothArm
{
  friend class SmoothJoint;
  friend class SmoothJointLoose;

  public:
  
    //ctor, dtor
    SmoothArm();
    SmoothArm( long long startTime, int lagTime, const std::vector<float> &startPosVec);
    virtual ~SmoothArm();

    virtual void reset( long long startTime, int lagTime, const std::vector<float> &startPosVec);

    virtual void setCoarseTarg( long long curTime, const std::vector<float> &targPosVec);

    void getSmoothedPos( long long curTime, std::vector<float>* pSmoothPosVec);

  protected:
    virtual void doArmSpline();
    void doJointSplines(float* pSmoothnessMerit);

    virtual void getSmoothedState(long long curTime, 
                                  std::vector<float>* pPosVec, 
                                  std::vector<float>* pVelVec);


    std::deque<long long> timeBuf;
    int lagTime;

    std::vector<float> curPosVec;
    std::vector<float> curVelVec;

    unsigned int numJoints;
    std::vector<SmoothJoint*> jointVec;
};

#endif
