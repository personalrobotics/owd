/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Labs Pittsburgh *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

#include "StepTraj.hh"


StepTraj::StepTraj(int trajid, int dof, unsigned int joint,
		   double *start_pos, double step_size)
  : nDOF(dof)
{
  id=trajid;
  start_position.SetFromArray(nDOF,start_pos);
  end_position = start_position;
  if ((joint<1) || (joint>7)) {
    throw "Joint must be between 1 and 7";
  }
  end_position[joint-1] += step_size;
}

StepTraj::~StepTraj() {
}
    
void StepTraj::evaluate(double y[], double yd[], double ydd[], double dt) {
  time += dt;
  if (time < 1.0) {
    for (unsigned int i = 0; i<nDOF; ++i) {
      y[i+1]=start_position[i];
      yd[i+1]=ydd[i+1]=0;
    }
  } else {
    for (unsigned int i = 0; i<nDOF; ++i) {
      y[i+1]=end_position[i];
      yd[i+1]=ydd[i+1]=0;
    }
  }
  if (time>3.0) {
    runstate = DONE;
  }

}
