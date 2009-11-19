#include <iostream>
#include <math.h>
#include "ForceTrajectory.hh"
#include "TrajType.hh"
#include <vector>

#ifndef __CONSTRAINEDFORCETRAJECTORY_HH
#define __CONSTRAINEDFORCETRAJECTORY_HH

class ConstrainedForceTrajectory : public ForceTrajectory {
public:

  class EndCondition {
  public:
    typedef enum {
      ANGLE=1,
      DISTANCE=2
    } Type;
    Type type;
    std::vector<double> value;
    double stall_vel;
  };
    
  ConstrainedForceTrajectory(const JointPos &staring_force_vector,
			     const EndCondition end_condition,
			     double max_velocity,
			     int trajid);
  virtual ~ConstrainedForceTrajectory();
    
  inline void lock(){pthread_mutex_lock(&mutex);}
  inline void unlock(){pthread_mutex_unlock(&mutex);}
    
  void run();
  void stop();
  int  state();
  void evaluate(double y[], double t[], double dt);

private:
  int DOF;
  JointPos starting_force;
  EndCondition end_cond;
  double max_vel;
};

#endif  // __CONSTRAINEDFORCETRAJECTORY_HH
