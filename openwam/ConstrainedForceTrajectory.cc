#include "ConstrainedForceTrajectory.hh"

ConstrainedForceTrajectory::ConstrainedForceTrajectory(
      const JointPos &starting_force_vector,
      const EndCondition end_condition,
      double max_velocity,
      int trajid) :
  starting_force(starting_force_vector),
  end_cond(end_condition)
{
  id=trajid;
  throw "incomplete class"; // have to set num of DOF
  // may need pointer to links
}

void ConstrainedForceTrajectory::evaluate(double y[], double t[], double dt) {
  for (unsigned int i=0; i<DOF; ++i) {
    t[i]=0.0;
  }
}

ConstrainedForceTrajectory::~ConstrainedForceTrajectory() {
}
