/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Labs Pittsburgh *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

#include <iostream>
#include <math.h>
#include "Trajectory.hh"
#include <vector>

#ifndef __STEPTRAJ_HH__
#define __STEPTRAJ_HH__

class StepTraj : public Trajectory {
private:
  int nDOF;

public:

  StepTraj(int trajid, int DOF, unsigned int joint, double *start_pos, double step_size);
  virtual ~StepTraj();

  // mandatory functions inherited from Trajectory
  void evaluate(double y[], double yd[], double ydd[], double dt);
};

#endif
