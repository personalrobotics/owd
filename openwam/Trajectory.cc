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

#include "Trajectory.hh"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

namespace OWD {

  bool Trajectory::log(const char *trajname) {
    if ((runstate != Trajectory::STOP) && (runstate != Trajectory::DONE)) {
      // can't log a running trajectory; it would mess up the times
      return false;
    }
    double oldtime=time;

    char *simfname = (char *) malloc(strlen(trajname) +9);
    // start with the logfilename
    sprintf(simfname,"%s_sim.csv",trajname);
    FILE *csv = fopen(simfname,"w");
    if (csv) {
      Trajectory::TrajControl tc(end_position.size());
      runstate=LOG;
      time=0.0;
      unsigned int DOF = end_position.size();
      while (runstate == LOG) {
	evaluate(tc,0.01);
	fprintf(csv,"%3.8f, ",time);
	for (unsigned int j=0; j<DOF; ++j) {
	  fprintf(csv,"%2.8f, ",tc.q[j]);
	}
	for (unsigned int j=0; j<DOF; ++j) {
	  fprintf(csv,"%2.8f, ",tc.qd[j]);
	}
	for (unsigned int j=0; j<DOF; ++j) {
	  fprintf(csv,"%2.8f, ",tc.qdd[j]);
	}
	for (unsigned int j=0; j<DOF; ++j) {
	  fprintf(csv,"%2.8f, ",tc.t[j]);
	}
      }
      fclose(csv);
    }
    reset(oldtime);
    runstate=STOP;
    free(simfname);
    return true;
  }

  void Trajectory::ForceFeedback(double ft[]) {
    memcpy(forcetorque,ft,6*sizeof(double));
    valid_ft=true;
    // simplistic check to see if greater than 3N of force towards palm
    static int forcecount(0);
    if ((runstate==RUN) &&
	CancelOnForceInput &&
	(forcetorque[2] < -6.0)) {
      if (++forcecount == 8) {
	// only stop if we have eight cycles in a row that exceed the threshold
	runstate=ABORT;
      }
    } else {
      forcecount=0;
    }
  }

  Trajectory::TrajControl::TrajControl(unsigned int nDOF) :
    q(nDOF), qd(nDOF), qdd(nDOF), t(nDOF)
  {
  }

}; // namespace OWD
