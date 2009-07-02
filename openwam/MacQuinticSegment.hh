// Implementation of jerk-limited concatenation of quintics, based
// on the 2001 thesis of Sonja Macfarlane, University of British Columbia

#ifndef MACQUINTICSEGMENT_H
#define MACQUINTICSEGMENT_H

#define TRAJ_TOLERANCE 0.003f

#include "MacQuinticElement.hh"

class MacQuinticSegment:public MacQuinticElement {
  friend class MacQuinticBlend;

protected:
  // direction is an n-dof unit vector pointing from start to end
  JointPos direction;

  double jmax;  // maximum path jerk

  // distance is the straightline jointspace distance from start to end
  double distance;

  // velocities are the tangential jointspace velocities
  double start_vel, end_vel;

  // keep track of which "if" condition we used to build the segment
  int condition;

  double accel_rise_dist(double v, double amax) const;
  double accel_fall_dist(double v, double amax) const;
  double AP_dist(double v, double amax) const;
  double AP_max_delta_v(double v, double amax, double d);  // change back to const after removing "reason" assignments
  double current_path_vel, current_path_accel;
  void enforceSpeedLimits();
  void reverse_accel_pulses();
  void verify_end_conditions();

  std::vector<MacAccelElement *> accel_elements;

public:
  MacQuinticSegment( TrajPoint first_p,
                     TrajPoint second_p,
		     JointPos max_joint_vel,
		     JointPos max_joint_accel,
		     double max_jerk);

  // functions required by the base class
  void setStartVelocity(double v);
  void setEndVelocity(double v);
  void BuildProfile();
  
  double StartVelocity() const;
  double EndVelocity() const;
  
  void evaluate(double *y, double *yd, double *ydd, double t);
  double calc_time(JointPos value) const;
  double PathVelocity() const;
  double PathAcceleration() const;

  // extra functions specific to a segment
  void setVelocity(double v1, double v2);  // both start and end at once
  JointPos Direction() const;

  void dump();
};


#endif // MACQUINTICSEGMENT_H

