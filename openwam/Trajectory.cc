/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Labs Pittsburgh *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

#include "Trajectory.hh"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
        double y[8],yd[8],ydd[8];
        runstate=LOG;
        time=0.0;
	unsigned int DOF = end_position.size();
        while (runstate == LOG) {
            evaluate(y,yd,ydd,0.01);
            fprintf(csv,"%3.8f, ",time);
            for (unsigned int j=1; j<=DOF; ++j) {
                fprintf(csv,"%2.8f, ",y[j]);
            }
            for (unsigned int j=1; j<=DOF; ++j) {
                fprintf(csv,"%2.8f, ",yd[j]);
            }
            for (unsigned int j=1; j<=DOF; ++j) {
                fprintf(csv,"%2.8f, ",ydd[j]);
            }
            fprintf(csv,"0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0\n"); // torq
        }
        fclose(csv);
    }
    reset(oldtime);
    runstate=STOP;
    free(simfname);
    return true;
}
