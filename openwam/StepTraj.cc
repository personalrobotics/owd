/***********************************************************************

  Copyright 2007-2010 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

  This file is part of owd.

  owd is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  owd is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
