#include <iostream>
#include <math.h>
#include "Trajectory.hh"
#include "Kinematics.hh"
#include <vector>

#ifndef __CONSTRAINEDFORCETRAJECTORY_HH
#define __CONSTRAINEDFORCETRAJECTORY_HH

class ConstrainedForceTrajectory : public Trajectory {
public:

  class EndCondition {
  public:
    typedef enum {
      ANGLE=1,
      DISTANCE=2
    } Type;
    Type type;
    double value;
    double stall_vel;
  };
    
  ConstrainedForceTrajectory(const JointPos &start,
			     const JointPos &staring_force_vector,
			     const EndCondition end_condition,
			     Link wam_links[],
			     double max_velocity,
			     int trajid);
  virtual ~ConstrainedForceTrajectory();
    
  void evaluate(double y[], double yd[], double ydd[], double dt);
  void update_torques(double t[]);

private:
  int DOF;
  JointPos starting_force;
  EndCondition end_cond;
  double max_vel;
  double *old_y;
  Link *links;
  double distance_moved;
};

#endif  // __CONSTRAINEDFORCETRAJECTORY_HH
