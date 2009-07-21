#ifndef SMOOTHARMLOOSE_H 
#define SMOOTHARMLOOSE_H 

#include "SmoothArm.h" 

// Algorithm description same as SmoothArm (See SmoothArm.h) except:
// 
// When we need to up update the splines and the are more than two points in the queue:
//   - Create splines from the current position and velocity, through the last point in the queue
//     with minimum integrated accel^2. 
//   - Reset the second position in the queue to match the spline evaluated at its time.

class SmoothArmLoose : public SmoothArm
{
  friend class SmoothJoint;
  friend class SmoothJointLoose;

  public:
  
    //ctor, dtor
    SmoothArmLoose();
    SmoothArmLoose( long long startTime, int lagTime, const std::vector<float> &startPosVec);

    virtual void reset(long long startTime, int lagTime, const std::vector<float> &startPosVec);

  protected:
};

#endif
