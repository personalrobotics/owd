/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Corporation *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

#ifndef MACQUINTICELEMENT_H
#define MACQUINTICELEMENT_H

#include "MacAccelPulse.hh"
#include "TrajType.hh"
#include <string.h>

class MacQuinticElement {
protected:

  // positions are n-dof vectors of joint angles
  JointPos start_pos, end_pos;

  // times are with respect to the start of the full trajectory
  double start_time, duration;

  double max_path_velocity, max_path_acceleration;
public:

  static const double VEL_MAX = -1; // flag to specify max velocity
  static const double PI = 3.141592654;

  char *reason;

  virtual void setStartVelocity(double v) = 0;
  virtual void setEndVelocity(double v) = 0;
  inline virtual void setStartTime(double t) {start_time = t;}
  virtual void BuildProfile() = 0;
  
  virtual double StartVelocity() const =0;
  virtual double EndVelocity() const =0;
  virtual double PathVelocity() const=0;
  virtual double PathAcceleration() const=0;
  inline virtual double MaxPathVelocity() const {return max_path_velocity;}
  inline virtual double MaxPathAcceleration() const {return max_path_acceleration;}

  inline virtual double StartTime() const {return start_time;}
  inline virtual double EndTime() const {
    if (duration<0) {
      throw "Must BuildProfile() before getting time";
    } else {
      return start_time+duration;
    }
  }
  virtual void evaluate(double *y, double *yd, double *ydd, double t)=0;
  virtual double calc_time(JointPos value) const =0;

  MacQuinticElement(JointPos start, JointPos end)
    : start_pos(start), end_pos(end), start_time(0), duration(-1) {
    reason = strdup("initial limits");
  }

  virtual void dump() {
    //    printf("  start_pos=%2.3f  end_pos=%2.3f\n",start_pos,end_pos);
    printf("  start_time=%2.3f  dur=%2.3f  end_time=%2.3f\n",
	   start_time,duration,start_time+duration);
  }

  virtual ~MacQuinticElement() {}
};

#endif // MACQUINTICELEMENT_H
