#include "MacJointTraj.hh"
#include <syslog.h>
#include "MacQuinticBlend.hh"


MacJointTraj::~MacJointTraj() {
  for (unsigned int i=0; i<macpieces.size(); ++i) {
    delete macpieces[i];
  }
}


MacJointTraj::MacJointTraj(TrajType &vtraj, 
			   const JointPos &mjv, 
			   const JointPos &mja,
			   double max_jerk,
			   bool bWaitForStart,
			   bool bHoldOnStall,
			   int trajid) :
  max_joint_vel(mjv), max_joint_accel(mja)
{
  id = trajid;
  WaitForStart=bWaitForStart;
  HoldOnStall=bHoldOnStall;
  runstate=STOP;
  time=0.0;
  // unlike previous trajectory code, this package assumes that
  // extraneous co-linear points have already been removed, so that
  // every remaining point is a bend or inflection.

    //    pthread_mutex_init(&mutex, NULL);
    if (vtraj.size() < 2) {
        throw "Trajectories must have 2 or more points";
    }

    DOF = vtraj[0].size();

    int numpoints = vtraj.size();
    start_position = vtraj.front();
    
    MacQuinticSegment *openseg; // this will hold the seg we're working on

    // make sure the trajectory specifies no blends at the ends
    vtraj[0].blend_radius = 0;
    vtraj.back().blend_radius = 0;

    printf("======== Segment 1 ===============\n");
    if (numpoints == 2) {
      // if there are only 2 points, we'll just make a single segment
      // with no blends at either end
      openseg = new MacQuinticSegment(vtraj[0],vtraj[1],
				      max_joint_vel,
				      max_joint_accel,
				      max_jerk);
      openseg->setVelocity(0,0);

    } else {

      // initialize the first segment with zero starting velocity
      openseg = new MacQuinticSegment(vtraj[0],vtraj[1],
				      max_joint_vel,
				      max_joint_accel,
				      max_jerk);
      openseg->setVelocity(0,MacQuinticElement::VEL_MAX);

    }

    // now, go through the points and build the remaining segments
    // and blends

    for (int i = 2; i < numpoints; ++i) {        

      printf("======== Segment %d ==============\n",i);
      // make the next segment
      MacQuinticSegment *nextseg;
      if (i==numpoints-1) {
	// the last segment doesn't get a final blend
	nextseg = new MacQuinticSegment(vtraj[i-1],vtraj[i],
					max_joint_vel,
					max_joint_accel,
					max_jerk);
	// set to come to a stop
	nextseg->setVelocity(openseg->EndVelocity(),0);
      } else {
	// all other segments expect blends at both ends (default)
	nextseg = new MacQuinticSegment(vtraj[i-1],vtraj[i],
					max_joint_vel,
					max_joint_accel,
					max_jerk);
	// set to increasing velocity (if possible)
	nextseg->setVelocity(openseg->EndVelocity(),
			     MacQuinticElement::VEL_MAX);
      }

      // add the current segment to the trajectory
      macpieces.push_back(openseg);

      if (vtraj[i-1].blend_radius > 0) {
	printf("======== Blend %d:%d ======================\n",i-1,i);
	// make the blend between the current and next
	MacQuinticBlend *blend = new MacQuinticBlend(openseg, nextseg,
						     vtraj[i-1].blend_radius,
						     max_joint_vel,
						     max_joint_accel,
						     max_jerk);

	// calculate the max blend velocity, based on previous segment
	// and dynamic limits
	blend->setStartVelocity(openseg->EndVelocity());
	
	// propogate the blend velocity to the two matching segment ends
	// (this may reduce the ending velocity of the openseg)
	printf("======== Blend seg %d ======================\n",i-1);
	openseg->setEndVelocity(blend->StartVelocity());
	printf("======== Blend seg %d ======================\n",i);
	nextseg->setStartVelocity(blend->EndVelocity());
      
	// add the blend to the trajectory
	macpieces.push_back(blend);
      } else {
	// must come to a stop before abruptly changing direction
	printf("======== Blend seg %d ======================\n",i-1);
	openseg->setEndVelocity(0);
	printf("======== Blend seg %d ======================\n",i);
	nextseg->setStartVelocity(0);
      }

      // step forward
      openseg = nextseg;

    }

    // add the final segment
    macpieces.push_back(openseg);

    // now we have a collection of segments and blends in which we always
    // tried to accelerate as much as possible, from beginning to end.  It's
    // time to go backwards from the end to make sure we can decelerate
    // in time for the final condition (v=0) and each of the blend limits.
    // We'll do this by using the base class interface so that we don't have 
    // to care if we're looking at a segment or a blend.

    for (unsigned int i = macpieces.size() -1; i>0; --i) {
      printf("==== Backwards pass, piece %d ====\n",i);
      if (macpieces[i-1]->EndVelocity() > macpieces[i]->StartVelocity()) {
	printf("WARNING: reducing speed of macpiece %d from %2.3f to %2.3f\n",
	       i-1,macpieces[i-1]->EndVelocity(),macpieces[i]->StartVelocity());
	macpieces[i-1]->setEndVelocity(macpieces[i]->StartVelocity());
      }
      if (macpieces[i-1]->EndVelocity() < macpieces[i]->StartVelocity()) {
	free(macpieces[i]->reason);
	macpieces[i]->reason = strdup("limited by previous segment ending velocity");
	macpieces[i]->setStartVelocity(macpieces[i-1]->EndVelocity());
      }
    }

    // finally, calculate all the curve coefficients
    for (unsigned int i=0; i<macpieces.size(); ++i) {
      printf("==== BuildProfile, piece %d ====\n",i);
      macpieces[i]->BuildProfile();
      if (i<macpieces.size()-1) {
	// propogate the time forward to the next element
	macpieces[i+1]->setStartTime(macpieces[i]->EndTime());
      }
    }

    traj_duration = macpieces.back()->EndTime();
    end_position = vtraj.back();

    current_piece =macpieces.begin();
    return;
}

void MacJointTraj::get_path_values(double *path_vel, double *path_accel) const {
  *path_vel = (*current_piece)->PathVelocity();
  *path_accel = (*current_piece)->PathAcceleration();
}

void MacJointTraj::get_limits(double *max_path_vel, double *max_path_accel) const {
  *max_path_vel = (*current_piece)->MaxPathVelocity();
  *max_path_accel = (*current_piece)->MaxPathAcceleration();
}

void MacJointTraj::evaluate(double y[], double yd[], double ydd[], double dt) {
  // shift the pointers to be 1-based (only if they're non-NULL)
  if (y) y++;
  if (yd) yd++;
  if (ydd) ydd++;

  // if we're running, then increment the time.  Otherwise, stay where we are
  if ((runstate == RUN) || (runstate == LOG)) {
    time += dt;
  }
  
  while ((current_piece != macpieces.end()) &&
	 (time > (*current_piece)->EndTime())) {
           
    // keep skipping forward until we find the one that includes this time
    ++current_piece;
  }
  if (current_piece == macpieces.end()) {
    // even though time is past the end, the piece will return the ending
    // values
    current_piece--;
    (*current_piece)->evaluate(y,yd,ydd,time);
    runstate = DONE;
  } else {
    (*current_piece)->evaluate(y,yd,ydd,time);
  }
  if (runstate == STOP) {
    // if we're supposed to be stationary, keep the position values we
    // calculated, but zero out the vel and accel
    for (int j=0; j < DOF; j++) {
      yd[j]=ydd[j]=0.0;
    }
  }
  return;
}

void MacJointTraj::run() {
    if (runstate==DONE || runstate==RUN) {
        return;
    }
    if (time > 0.0f) {
      syslog(LOG_ERR,"ERROR: Attempted to restart a trajectory in the middle");
      return;
    }
    runstate=RUN;
    return;
}

void MacJointTraj::stop() {
    runstate = STOP;
}

int MacJointTraj::state() {
    return runstate;
}

void MacJointTraj::log(char *trajname) {
    if ((runstate != STOP) && (runstate != DONE)) {
      throw "can't log a running trajectory";
    }
    double oldtime=time;
    current_piece = macpieces.begin();
    char *simfname = (char *) malloc(strlen(trajname) +9);
    // start with the logfilename
    sprintf(simfname,"%s_sim.csv",trajname);
    FILE *csv = fopen(simfname,"w");
    if (csv) {
        double y[8],yd[8],ydd[8];
        runstate=LOG;
        time=0.0;
        while (runstate == LOG) {
            evaluate(y,yd,ydd,0.01);
            fprintf(csv,"%3.8f, ",time);
            for (int j=1; j<=DOF; ++j) {
                fprintf(csv,"%2.8f, ",y[j]);
            }
            for (int j=1; j<=DOF; ++j) {
                fprintf(csv,"%2.8f, ",yd[j]);
            }
            for (int j=1; j<=DOF; ++j) {
                fprintf(csv,"%2.8f, ",ydd[j]);
            }
            fprintf(csv,"0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0\n"); // torq
        }
        fclose(csv);
    }
    reset(oldtime);
    runstate=STOP;
    free(simfname);
    return;
}

void MacJointTraj::dump() {
  printf("MacJointTraj: %d pieces, %2.3fs total duration\n",
	 macpieces.size(), traj_duration);
  for (unsigned int i=0; i<macpieces.size(); ++i) {
    macpieces[i]->dump();
  }
}

void MacJointTraj::reset(double t) {
  time=t;
  current_piece=macpieces.begin();
}
