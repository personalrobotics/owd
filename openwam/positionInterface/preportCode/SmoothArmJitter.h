#ifndef SMOOTHARMJITTER_H 
#define SMOOTHARMJITTER_H 

#include "SmoothArm.h" 

// Algorithm description same as SmoothArm (See SmoothArm.h) except:
// 
//  When we need to up update the splines and the are more than two points in the queue:
//   - Create splines that go from the current state, through the next point with a splope maximum
//   - Several splines are created  for a range of adjusted timestamps of the second point.
//   - The adjusted time satifies:
//           time(firstPoint) <
//                         adjustedTime(secondPoint)
//                                                 <= originalTime(secondPoint)
//   - The smoothest spline is kept, and the second point's time is adjusted accordingly.
//   - This spline is valid to the time of the second point.
//

class SmoothArmJitter : public SmoothArm
{
  friend class SmoothJoint;

  public:
  
    //ctor, dtor
    SmoothArmJitter();
    SmoothArmJitter( long long startTime, int lagTime, const std::vector<float> &startPosVec);

  protected:
    virtual void doArmSpline();
};

#endif
