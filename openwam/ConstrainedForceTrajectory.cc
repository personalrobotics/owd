/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Corporation *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

#include "ConstrainedForceTrajectory.hh"
#include "Joint.hh"
#include <stdlib.h>

extern "C" {
  void dgemv_(char *trans, int *m, int *n, double *alpha, double *a, int *lda,
	      double *x, int *incx, double *beta, double *y, int *incy);
}

ConstrainedForceTrajectory::ConstrainedForceTrajectory(
      const JointPos &startpos,
      const JointPos &starting_force_vector,
      const EndCondition end_condition,
      Link wam_links[],
      double max_velocity,
      int trajid) :
  starting_force(starting_force_vector),
  end_cond(end_condition),
  links(wam_links),
  distance_moved(0)
{
  // initialize the base-class members
  start_position=startpos;
  id=trajid;
  WaitForStart=false;
  HoldOnStall=false;
  
  DOF=start_position.size();
  old_y = (double *) malloc (DOF*sizeof(double));

}

void ConstrainedForceTrajectory::evaluate(double y[], double yd[], double ydd[], double dt) {
  // shift the pointers to be 1-based (only if they're non-NULL)
  if (y) y++;
  if (yd) yd++;
  if (ydd) ydd++;
  
  if (!y) {
    return; // can't do anything
  }

  // compute the joint movement since last call
  static double *y_diff=(double *)malloc(DOF*sizeof(double));
  for (unsigned int i=0; i<DOF; ++i) {
    y_diff[i]=y[i]-old_y[i];
    old_y[i]=y[i]; // save the y for next time
  }

  // compute the current Jacobian
  static double J[Joint::Jn][6];
  JacobianDB(J, links);

  // compute the workspace movement of the hand
  static double *ws_diff = (double *)malloc(6*sizeof(double));
  // calculate ws_diff = J*y_vel
  char TRANSN = 'N';  int NEQS = 6; int LDJ = 6; int INC = 1;
  double ALPHA =  1.0;  double BETA  =  0.0;
  dgemv_(&TRANSN, &NEQS,&DOF, &ALPHA,
	   &J[0][0],        &LDJ,
	   y_diff,    &INC, &BETA,
	   ws_diff, &INC);
  JointPos WSdiff;
  WSdiff.SetFromArray(DOF,ws_diff);
  

  // compare against velocity limits???



  // check for ending condition
  distance_moved += sqrt(pow(ws_diff[0],2)
			 +pow(ws_diff[1],2)
			 +pow(ws_diff[2],2));
  double total_angle = acos((starting_force*WSdiff)
			    / starting_force.length()
			    / WSdiff.length());
  if (((end_cond.type == EndCondition::ANGLE)
       && (total_angle > end_cond.value)) 
      || ((end_cond.type == EndCondition::DISTANCE)
	  && (distance_moved > end_cond.value))) {
    stop();
    // don't need to change y[]; just leave it as is.
    for (unsigned int i=0; i<DOF; ++i) {
      if (yd) {
	yd[i]=0;
      }
      if (ydd) {
	ydd[i]=0;
      }
    }
    return;
  }

  // compute how big a WS step to take (based on velocity, distance)

  // compute the new joint angles for the step

}

void ConstrainedForceTrajectory::update_torques(double t[]) {
}

ConstrainedForceTrajectory::~ConstrainedForceTrajectory() {
  free(old_y);
}
