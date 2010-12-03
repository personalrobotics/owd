/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Corporation *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

// Implementation of jerk-limited concatenation of quintics, based
// on the 2001 thesis of Sonja Macfarlane, University of British Columbia

#ifndef MACQUINTICSEGMENT_H
#define MACQUINTICSEGMENT_H

#define TRAJ_TOLERANCE 0.003f

#include "MacQuinticElement.hh"
#include "MacAccelPulse.hh"

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


  double current_path_vel, current_path_accel;
  void enforceSpeedLimits();
  void reverse_accel_pulses();
  void verify_end_conditions();
  bool search_for_decel_pulse(double dist,
			      MacAccelElement *ae1,
			      MacAccelElement *ae2);
  MacAccelElement *check_for_cruise(double distance,
				    MacAccelElement *ae1,
				    MacAccelElement *ae2);
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

  static double accel_rise_dist(double v, double amax, double jmax);
  static double accel_fall_dist(double v, double amax, double jmax);
  static double AP_dist(double v, double amax, double jmax);
  static double AP_max_delta_v(double d, double v,
			       double vmax, double amax, double jmax);
  static double cubic_solve(double A, double B, double C);
  static double safe_pow(double val, double exp);
  static double safe_sqrt(double val);

  void dump();
};


#endif // MACQUINTICSEGMENT_H

