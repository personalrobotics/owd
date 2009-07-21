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
//  - (curTime - lagTime) is a lower limit to the required buffering.  However,
//    to avoid blocking the real-time thread, processing of new targets is 
//    can be delayed, and a higher lower limit may be established via step 2.  
//    If it is determined that (curTime - lagTime) is actually a new lower limit
//    then:
//        - The current smoothed state is evaluated.
//        - All points in the queue that come before curTime - lagTime
//          are popped off.
//        - The spline is updated.  The new spline interval starts
//          at curTime - lagTime.
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

struct PendingTargPos
{
  bool bPending;
  long long timeStamp;
  std::vector<double> completeTargPosVec;
};

class SmoothArm
{
  friend class SmoothJoint;
  friend class SmoothJointLoose;

  public:
    enum SA_State { INVALID, VALID, PENDING_RESET };
  
    //ctor, dtor
    SmoothArm();
    SmoothArm(const std::vector<double> &startPosVec);
    ~SmoothArm();

    // non-real-time interface

    // accessors
    void setLagTime(int lagTimeUSec);
    void invalidate();
    bool isValid();

    void setReset();
    void setCoarseTarg( const std::vector<int> &jointIndicesVec,
                                const std::vector<double> &partialTargPosVec);
    void setCoarseTarg(const std::vector<double> &completeTargPosVec);

    // real-time interface
    void reset(const std::vector<double> &startPosVec);
    void reset(const double* startPosArray, int size);
    void getSmoothedPVA(std::vector<double>* pSmoothPosVec, 
                        std::vector<double>* pSmoothVelVec = NULL,
                        std::vector<double>* pSmoothAccelVec = NULL);
    void getSmoothedPVA(double* smoothPosArray,
                        double* smoothVelArray = NULL,
                        double* smoothAccelArray = NULL);

  protected:
    void setupSpeedLimits();
    long long getCurTimeUSec();
    void doArmSpline();
    void doJointSplines(double* pSmoothnessMerit);

    void reportStatus();

    void processPendingReset();
    void processPendingCoarseTarg();

    void getSmoothedState(long long curTime, 
                                  std::vector<double>* pPosVec, 
                                  std::vector<double>* pVelVec,
                                  std::vector<double>* pAccelVec=NULL );

    SA_State state;

    std::deque<long long> timeBuf;
    int lagTime;

    PendingTargPos pendingResetTarg;
    PendingTargPos pendingCoarseTarg;

    std::vector<double> curPosVec;
    std::vector<double> curVelVec;

    std::vector<double> lastSafePosVec;
    std::vector<double> jointSpeedLimitVec;
    std::vector<bool> jointSpeedLimitExceededVec;

    unsigned int completePosVecSize;
    std::vector<SmoothJoint*> jointVec;
};


#endif
