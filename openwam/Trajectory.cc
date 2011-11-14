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
    forcetorque = R6(ft[0],ft[1],ft[2],
		     ft[3],ft[4],ft[5]);
    valid_ft=true;
    static int forcecount(0);
    static int torquecount(0);
   
    // check force threshold
    if ((runstate==RUN) &&
	CancelOnForceInput &&
	((forcetorque.v * forcetorque_threshold_direction) 
	 > forcetorque_threshold)) {
      //      if (++forcecount == 8) {
	// only stop if we have eight cycles in a row that exceed the threshold
	runstate=ABORT;
	//      }
	//    } else {
	//      forcecount=0;
    }
    // check torque threshold
    if ((runstate==RUN) &&
	CancelOnForceInput &&
	((forcetorque.w * forcetorque_torque_threshold_direction) 
	 > forcetorque_torque_threshold)) {
      //      if (++torquecount == 8) {
	// only stop if we have eight cycles in a row that exceed the threshold
      	runstate=ABORT;
      //      }
	//    } else {
	//      torquecount=0;
    }
  }

  void Trajectory::TactileFeedback(float tactile[], int repetitions) {
    // check whether we should stop the trajectory based on
    // tactile sensor values.
    // args:
    //   tactile: array of 96 readings
    //   repetitions: number of times that this function will be called
    //       before every  cell has been updated.  OWD gets just a partial
    //       update of the tactile data in each CAN message, so the
    //       repetitions argument allows this function to take that
    //       into account.
    // preset values:
    //   tactile_pad: the pad number (0 through 3) that we are watching
    //   tactile_threshold: the reading that is considered "pressed"
    //   tactile_minimum_cells: the number of cells that have to meet
    //      or exceed the threshold in a single cycle for the pad to be
    //      considered "pressed"
    //   tactile_minimum_readings: the number of consecutive times the
    //      cell should be "pressed" to stop the trajectory
    static int tactilecount(0);
    if ((runstate==RUN) &&
	CancelOnTactileInput) {
      int cellcount(0);
      for (int i=0; i<24; ++i) {
	if (tactile[i + tactile_pad*24] > tactile_threshold) {
	  ++cellcount; 
	}
      }
      if (cellcount > tactile_minimum_cells) {
	++tactilecount;
	if (tactilecount > tactile_minimum_readings * repetitions) {
	  runstate = ABORT;
	}
      } else {
	tactilecount=0;
      }
    }
  }


  Trajectory::TrajControl::TrajControl(unsigned int nDOF) :
    q(nDOF), qd(nDOF), qdd(nDOF), t(nDOF)
  {
  }

  R3 Trajectory::forcetorque_threshold_direction(0,0,-1);  // negative Z (towards palm)
  double Trajectory::forcetorque_threshold(6.0);    // 6 newtons
  R3 Trajectory::forcetorque_torque_threshold_direction(1,0,0);
  double Trajectory::forcetorque_torque_threshold(999); // disabled at start

  int Trajectory::tactile_pad(0);
  float Trajectory::tactile_threshold(1.0);
  int Trajectory::tactile_minimum_cells(8);
  int Trajectory::tactile_minimum_readings(6);
  std::vector<double> Trajectory::tactile_debug_data;
}; // namespace OWD

