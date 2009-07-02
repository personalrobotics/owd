#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <assert.h>
#include "MacQuinticSegment.hh"

MacQuinticSegment::MacQuinticSegment( TrajPoint first_p,
				      TrajPoint second_p,
				      JointPos max_joint_vel,
				      JointPos max_joint_accel,
				      double max_jerk):
  MacQuinticElement(first_p,second_p), // sets start_pos, end_pos
  jmax(max_jerk) 
{

  // calculate the distance and unit direction vector
  direction = second_p - first_p;
  distance = direction.length();
  direction /= distance;

  // adjust the endpoints based on anticipated blends
  if (first_p.blend_radius > 0) {
    start_pos += direction * fabs(first_p.blend_radius);
    distance -= first_p.blend_radius;
  }
  if (second_p.blend_radius > 0) {
    end_pos -= direction * fabs(second_p.blend_radius);
    distance -= second_p.blend_radius;
  }
  if (distance <0) {
    throw "MacQuinticSegment: Segment is shorter than the distance needed for blend(s)";
  }

  // find the max vel and accel we can apply in the calculated direction
  // without violating any per-joint limits

  max_path_velocity = -1.0;
  max_path_acceleration = -1.0;

  // go through the joints one-by-one; if we haven't yet set our values
  // and this joint is moving, try using it to set our speed.  If we're
  // already exceeding this joint's limit, then use this joint to recalc.
  for (unsigned int i = 0; i<direction.size(); ++i) {
    // velocity checks
    if (max_path_velocity < 0) {
      if (direction[i] != 0) {
	max_path_velocity = max_joint_vel[i] / fabs(direction[i]);
      }
    } else if ((max_path_velocity * fabs(direction[i])) > max_joint_vel[i]) {
      // this joint is going to exceed its vel limit, so recalc
      max_path_velocity = max_joint_vel[i] / fabs(direction[i]);
    }

    // repeat for acceleration
    if (max_path_acceleration < 0) {
      if (direction[i] != 0) {
	max_path_acceleration = max_joint_accel[i] / fabs(direction[i]);
      }
    } else if ((max_path_acceleration * fabs(direction[i])) > max_joint_accel[i]) {
      // this joint is going to exceed its accel limit, so recalc
      max_path_acceleration = max_joint_accel[i] / fabs(direction[i]);
    }
  }
  if ((max_path_velocity <= 0) || (max_path_acceleration <= 0)) {
    throw "ERROR: could not calculate a valid max velocity or acceleration (perhaps direction vector was all zero?)";
  }

}

// accel_rise_dist: returns how much distance is traversed by a single
// acceleration rise from zero to amax, starting at velocity v and 
// subject to jerk limit jmax
inline double MacQuinticSegment::accel_rise_dist(double v, double amax) const {
  // distance required for a single accel rise
  // Equation 3.7 / 3.8 of MacFarlane thesis
  double atime = amax/jmax * PI/2;
  return(fabs(v) * atime + amax*atime*atime*(.25 - 1/PI/PI));
}

// accel_fall_dist: distance covered by a single accel fall from amax to 0,
// starting at velocity v
inline double MacQuinticSegment::accel_fall_dist(double v, double amax) const {
  double atime = amax/jmax * PI/2;
  return(fabs(v) * atime + amax*atime*atime*(.25 + 1/PI/PI));
}

inline double MacQuinticSegment::AP_dist(double v, double amax) const {
  // total distance is an acceleration rise starting at velocity v followed
  //  by an acceleration fall starting at v+amax*dtmax/2 (midpoint v).
  double atime = amax/jmax * PI/2;
  return (accel_rise_dist(v,amax) 
	  + accel_fall_dist(v + amax*atime/2, amax));
}

// AP_max_delta_v: given limited distance, what's the max velocity
// change we can achieve with an acceleration pulse?
// (pulse can be either positive or negative - delta v is the same)
inline double MacQuinticSegment::AP_max_delta_v(double v, double amax, double d) {
  d = fabs(d); v=fabs(v); amax=fabs(amax);
  double atime = amax/jmax * PI/2;

  // make sure we can reach max accel and still have room for 
  //   a sustained portion
  if (d <= AP_dist(v,amax)) {
    // there's no room for a sustained portion, so calculate the max pulse
    // we can do
    if (d <= 2 * v * atime) {
      // at this speed, we don't even have room to change acceleration,
      // so we have to leave the speed the way it is
      //      printf("AP_max_delta_v # 1: dist of %2.3f required for a pulse, only %2.3f available\n",2*v*atime,d);
      free(reason);
      reason=strdup("distance limit (too short for any accel)");
      return 0;
    }
    // we'll just do an accel pulse to a lower accel value
    // set the equation in AP_dist equal to d and solve for the new amax
    //    printf("Segment accel pulse reduced from a=%2.3f to %2.3f (distance limit)\n",
    //	   amax,(d-2*v*atime)/atime/atime);
    amax=(d-2*v*atime)/atime/atime;
    if (amax<0) {
      throw "AP_max_delta_v: calculated a negative acceleration";
    }
    
    // now use the new (lower) amax to calculate the new delta v
    //    printf("New max delta_v=%2.3f\n",atime*amax);
    free(reason);
    reason=strdup("distance limit w/ lower amax");
    return atime * amax;
  }
  // we're rising to amax, holding as long as we can, then falling back to 0
  // first, subtract out the distance consumed by the first rise
  d -= accel_rise_dist(v,amax);
  //  printf("AP_max_delta_v: distance remaining = %2.3f\n",d);
  // next, calculate the time that the sustain portion can last
  // uses quadratic formula based on calculating remaining distance
  // d=d_sustain + d_fall  (a function of t_sustain^2 and t_sustain)
  double t_sustain = (-v -1.5*amax*atime
		+ sqrt(v*v + 3*v*amax*atime + 2.25*amax*amax*atime*atime
		       -2*amax*(atime*v+amax*atime*atime*(1/PI/PI+.75)-d)))
    /amax;  // updated 10/4/08
  if (t_sustain < 0) {
    throw "ERROR: negative sustain time in this segment";
  }
  
  // finally, compute the total velocity change
  // we're still just working with positive velocities (directions can be neg)
  double delta_v = amax*atime + t_sustain*amax;
  if ((v + delta_v) > max_path_velocity) {
    // should stay below max_path_velocity
    //    printf("AP_max_delta_v condition 3: limiting max delta_v of %2.3f to only %2.3f to stay below %2.3f\n",delta_v,max_path_velocity-v,max_path_velocity);
    free(reason);
    reason=strdup("velocity limit");
    return max_path_velocity - v;
  } else {
    //    printf("AP_max_delta_v condition 4: max delta_v of %2.3f\n",delta_v);
    //    printf("t_sustain=%2.3f, amax=%2.3f\n",t_sustain,amax);
    free(reason);
    reason=strdup("distance limit with sustain");
    return delta_v;
  }
}

void MacQuinticSegment::setVelocity(double v1, double v2) {
  start_vel = (v1 == VEL_MAX) ? max_path_velocity : fabs(v1);
  end_vel = (v2 == VEL_MAX) ? max_path_velocity : fabs(v2);
  enforceSpeedLimits();
  return;
}

void MacQuinticSegment::setStartVelocity(double v) {
  start_vel = (v==VEL_MAX) ? max_path_velocity : fabs(v);
  enforceSpeedLimits();
  return;
}

void MacQuinticSegment::setEndVelocity(double v) {
  end_vel = (v==VEL_MAX) ? max_path_velocity : fabs(v);
  enforceSpeedLimits();
  return;
}

double MacQuinticSegment::StartVelocity() const {
  return start_vel;
}

double MacQuinticSegment::EndVelocity() const {
  return end_vel;
}

void MacQuinticSegment::enforceSpeedLimits() {
  if (start_vel > max_path_velocity) {
    start_vel = max_path_velocity;
  }
  if (end_vel > max_path_velocity) {
    end_vel = max_path_velocity;
  }
  
  double delta_v = end_vel - start_vel;
  double max_delta;
  if (delta_v >= 0) {
    max_delta = AP_max_delta_v(start_vel, max_path_acceleration, distance);
  } else {
    // if start_vel > end_vel then we'll later swap them, so compute based
    // on end_vel instead of start_vel
    max_delta = AP_max_delta_v(end_vel, max_path_acceleration, distance);
  }
  if (delta_v > max_delta) {
    // speedup is too great: reduce the ending velocity
    //    printf("Reducing the ending velocity from %2.3f to %2.3f due to accel pulse limits\n",
    //	   end_vel, start_vel+max_delta);
    end_vel = start_vel + max_delta;
  } else if (delta_v < - (double) max_delta) {
    // slowdown is too great: reduce the starting velocity
    //    printf("Reducing the starting velocity from %2.3f to %2.3f due to decel pulse limits\n",
    //	   start_vel, end_vel+max_delta);
    start_vel = end_vel + max_delta;
  }
  return;
}

void MacQuinticSegment::verify_end_conditions() {
  if (accel_elements.size() == 0) {
    throw "MacQuinticSegment: no accel elements defined";
  }
  MacAccelElement *ap = accel_elements.front();
  if (fabs(ap->start_pos())>.005) {
    throw "MacQuinticSegment: generated pulses don't have correct starting position";
  }
  if (fabs(ap->start_vel()-start_vel)>.005) {
    throw "MacQuinticSegment: generated pulses don't have correct starting velocity";
  }
  ap = accel_elements.back();
  if (fabs(ap->end_pos()-distance)>.005) {
    throw "MacQuinticSegment: generated pulses don't have correct ending position";
  }
  if (fabs(ap->end_vel()-end_vel)>.005) {
    throw "MacQuinticSegment: generated pulses don't have correct ending velocity";
  }
  return;
}

void MacQuinticSegment::BuildProfile() {
  bool reverse=false;
  if (end_vel < start_vel) {
    // for a slowdown, compute it as if it were a speedup, but
    // remember to reverse things at the end
    reverse = true;
    double tmp_vel = end_vel;
    end_vel = start_vel;
    start_vel = tmp_vel;
  }

  MacAccelElement *ap;
  if (distance == 0) {
    throw "MacQuinticSegment::BuildProfile: distance was zero";
  }
  
  double cruise_start=0;
  double cruise_start_time=start_time;
  duration=0;

  double delta_v = end_vel - start_vel;
  
  // ============================================================
  // Test #1: Max Velocity?
  //
  // check to see whether we have room to reach max velocity
  // during this segment
  // ============================================================
  MacAccelElement *ap1 = NULL;
  MacAccelElement *ap2 = NULL;
  double distance_remaining = distance;
  if (start_vel < max_path_velocity) {
    ap1 = new MacAccelPulse(0,
			    start_vel,
			    (max_path_velocity-start_vel),
			    max_path_acceleration,
			    jmax,
			    start_time);
    distance_remaining -= maxrise->distance();
    cruise_start = maxrise->end_pos();
    cruise_start_time = maxrise->end_time();
  }
  if (end_vel < max_path_velocity) {
    ap2 = new MacAccelPulse(cruise_start;
			    max_path_velocity,
			    (end_vel - max_path_velocity),
			    -max_path_acceleration,
			    -jmax,
			    cruise_start_time);
    distance_remaining -= maxfall->distance();
  }
  if (fabs(distance_remaining) < 0.005) {
    // in the unlikely even that we matched the distance, we
    // can wrap things up with what we've already used for testing
    if (ap1) {
      duration += ap1->duration();
      accel_elements.push_back(ap1);
    }
    if (ap2) {
      duration += ap2->duration();
      accel_elements.push_back(ap2);
    }
    if (reverse) {
      reverse_accel_pulses();
    }
    verify_end_conditions();
    return;
  }

  if (distance_remaining > 0) {
    // we have confirmed that we can accel up to max vel and back down,
    // and still have room left over for a velocity cruise.  finishing up
    // is just a matter of creating the cruise.
    if (ap1) {
      duration += ap1->duration();
      accel_elements.push_back(ap1);
    }
    double cruise_end = cruise_start + distance_remaining;
    ap1 = new MacZeroAccel(cruise_start,
			   max_path_velocity,
			   cruise_end,
			   cruise_start_time);
    duration += ap1->duration();
    accel_elements.push_back(ap1);
    if (ap2) {
      // modify ap2 to start at the cruise ending position and time
      ap2->reset(cruise_end,
		 max_path_velocity,
		 -max_path_acceleration,
		 -jmax,
		 cruise_start_time + ap1->duration());
      duration += ap2->duration();
      accel_elements.push_back(ap2);
    }
    if (reverse) {
      reverse_accel_pulses();
    }
    verify_end_conditions();
    return;
  }

  // ok, we've established that we can't reach max_velocity.  throw
  // away our test pulses.
  if (ap1) {
    delete ap1;
    ap1=NULL;
  }
  if (ap2) {
    delete ap2;
    ap2=NULL;
  }
			   
  // ============================================================
  // TEST #2: Pure Acceleration
  //
  // check to see whether accelerating from start_vel to end_vel
  // exactly uses up the distance available
  // ============================================================

  // first, construct the accel pulse that satisfies the velocity change
  if (end_vel != start_vel) {
    ap1 = new MacAccelPulse(0,
			   start_vel,
			   (end_vel-start_vel),
			   max_path_acceleration,
			   jmax,
			   start_time);
    cruise_start=ap1->distance();
    cruise_start_time=ap1->end_time();
    distance_remaining = distance - ap1->distance();
    if (distance_remaining < -0.005) {
      // something's wrong; we don't even have the available distance
      // to reach end_vel
      printf("distance available is %2.8f, pulse requires %2.8f\n",
	     distance,ap1->distance());
      ap1->dump();
      delete ap1;
      throw "Error: velocity change cannot be met with motion limits in available distance";
    }
    if (distance_remaining < 0.005) {
      // we used up all the distance just making the velocity change,
      // so we're done.
      duration = ap1->duration();
      accel_elements.push_back(ap1);
      if (reverse) {
	reverse_accel_pulses();
      }
      condition=1;
      verify_end_conditions();
      return;
    }      
  }

  // ============================================================
  // Peak Velocity Tests
  //
  // we've established that there's extra distance available for
  // a velocity peak, and that peak is less than max_path_velocity.
  // now we'll go through some checks to solve for the peak velocity.
  //
  // since start_vel <= end_vel, we will end up with one of the
  // following cases (corresponding to the subsequent tests):
  //    #3: Both accel pulse and decel pulse reach max_accel
  //    #4: Accel pulse reaches max_accel but decel pulse doesn't
  //    #5: Neither accel pulse or decel pulse reach max_accel
  //
  // ============================================================
  // TEST #3: Max Deceleration
  //
  // see whether there's room to get in an Accel Pulse of amplitude
  // max_path_acceleration when decelerating from Vpeak to end_vel.
  // if so, we know that both accel and decel pulses will reach
  // max_path_acceleration, and it will just be a matter of computing
  // the appropriate sustain durations for each.
    
  double vpeak = end_vel + 
    max_path_acceleration * max_path_acceleration / jmax
    * PI / 2;
  if (ap1) {
    delete ap1;
  }
  ap1 = new MacAccelPulse(0,
			  start_vel,
			  (vpeak - start_vel),
			  max_path_acceleration,
			  jmax,
			  start_time);
  ap2 = new MacAccelPulse(ap1->end_pos(),
			  vpeak,
			  end_vel - vpeak,
			  -max_path_acceleration,
			  -jmax,
			  ap1->end_time());
  distance_remaining = distance - ap1->distance() - ap2->distance();
  if (fabs(distance_remaining) < 0.005) {
    // we got lucky; it fits!
    accel_elements.push_back(ap1);
    accel_elements.push_back(ap2);
    duration = ap1->duration() + ap2->duration();
    if (reverse) {
      reverse_accel_pulses();
    }
    verify_end_conditions();
    return;
  } else if (distance_remaining > 0) {
    // we can make it fit by adding some sustain portions to both
    // accel and decel pulses.  the integrated area added to each
    // pulse must be equal in order for the end_velocity to come
    // out right, which means that the two sustain durations will
    // be equal (since the amplitude is at max_accel).
    // also, each sustain portion will use exactly half of
    // distance_remaining.

    // compute velocity at end of ap1 sustain
    double ap1_sustain_v = ap1->end_vel() 
      - max_path_acceleration * max_path_acceleration / jmax * PI / 2;

    // compute length of sustain extension
    // solve for t using quadratic formula, based on
    //   d = 1/2 * a * t*t  + v * t  (where d = distance_remaining/2)
    double extra_sustain_t =
      (sqrt(ap1_sustain_v * ap1_sustain_v
	    + max_path_acceleration*distance_remaining)
       - ap1_sustain_v)
      / max_path_acceleration;
    if (extra_sustain_t <0) {
      // something went wrong
      delete ap1;
      delete ap2;
      throw "Error: bad calculation when extending pulses";
    }
    ap1->extend_sustain_time_by(extra_sustain_t);
    ap2->extend_sustain_time_by(extra_sustain_t);
    if ((ap1->distance() + ap2->distance()) > distance+0.005) {
      delete ap1;
      delete ap2;
      throw "Error: overall distance doesn't add up after extending pulses";
    }
    accel_elements.push_back(ap1);
    accel_elements.push_back(ap2);
    duration = ap1->duration() + ap2->duration();
    if (reverse) {
      reverse_accel_pulses();
    }
    verify_end_conditions();
    return;
  }
  delete ap1;
  delete ap2;

  // ============================================================
  // TEST #4: Max acceleration
  //
  // If we can reach max acceleration during the first pulse and still
  // return to end_vel with distance remaining, then we just have to 
  // add a sustain portion to the accel pulse and scale up the decel
  // pulse to match.
  
  if (end_vel < start_vel) {
    // no need to check unless the velocities are different; otherwise
    // it amounts to the same thing we tested in the previous case.
    // this case is really for the situation where just going from start_vel
    // to end_vel uses most of the distance, and there's just a little room
    // to squeeze in a bit more accel and decel.
    vpeak = start_vel + 
      max_path_acceleration * max_path_acceleration / jmax
      * PI / 2;
    ap1 = new MacAccelPulse(0,
			    start_vel,
			    (vpeak - start_vel),
			    max_path_acceleration,
			    jmax,
			    start_time);
    ap2 = new MacAccelPulse(ap1->end_pos(),
			    vpeak,
			    end_vel - vpeak,
			    -max_path_acceleration,
			    -jmax,
			    ap1->end_time());
    distance_remaining = distance - ap1->distance() - ap2->distance();
    if (fabs(distance_remaining) < 0.005) {
      // we're done
      accel_elements.push_back(ap1);
      accel_elements.push_back(ap2);
      duration = ap1->duration() + ap2->duration();
      if (reverse) {
	reverse_accel_pulses();
      }
      verify_end_conditions();
      return;
    } else if (distance_remaining > 0) {
      // calculate 
    











  // make up the remaining distance with a velocity cruise
  ap = new MacZeroAccel(cruise_start,end_vel,
			cruise_start+distance_remaining,
			cruise_start_time);
  duration += ap->duration();
  accel_elements.push_back(ap);
  if (reverse) {
    reverse_accel_pulses();
  }
  condition=13;
  verify_end_conditions();
  return;

  // FUTURE CODE (too many bugs to work out right now - MVW 10/4/2008)

  if (end_vel != start_vel) {

  } else if (start_vel == max_path_velocity) {
    // if both start and end are already at max, then just hold
    // the velocity constant
    MacAccelElement *ap = new MacZeroAccel(0,start_vel,distance,start_time);
    duration=ap->duration();
    accel_elements.push_back(ap);
    // no need to reverse?
    condition=2;
    verify_end_conditions();
    return;
  } else {
    // if start and end velocities are equal and less than max, we
    // have it easy: just put in two matching pulses, one pos and one neg

    // calculate the max velocity boost we can accomodate in the distance.
    // this will already be limited to max_path_velocity
    double delta_v = AP_max_delta_v(start_vel,max_path_acceleration,
				    distance_remaining/2);
    if (delta_v > 0) {
      ap = new MacAccelPulse(0,start_vel,delta_v,
			     max_path_acceleration, jmax, start_time);
      printf("condition 3 actual delta_v = %2.3f, first dist=%2.3f\n",
	     delta_v,ap->distance());
      
      accel_elements.push_back(ap);
    } else {
      // not enough space to boost velocity at all
      ap = new MacZeroAccel(0,start_vel,distance,start_time);
      accel_elements.push_back(ap);
      duration = ap->duration();
      if (reverse) {
	reverse_accel_pulses();
      }
      condition=12;
      verify_end_conditions();
      return;
    }
    // it's possible that we were velocity limited, and we need to cruise
    // at max velocity before starting our decel pulse
    duration = ap->duration();
    if (ap->distance() < distance_remaining/2) {
      double extra_d = distance_remaining - 2*ap->distance();
      // build the cruise element
      ap= new MacZeroAccel(ap->end_pos(),ap->end_vel(),
			   ap->end_pos()+extra_d,
			   ap->end_time());
      // insert it
      accel_elements.push_back(ap);
      duration += ap->duration();
    }
    // now build the final element
    ap = new MacAccelPulse(ap->end_pos(),ap->end_vel(),-delta_v,
			   -max_path_acceleration, -jmax, ap->end_time());
    
    accel_elements.push_back(ap);
    duration+=ap->duration();
    if (reverse) {
      reverse_accel_pulses();
    }
    condition=3;
    verify_end_conditions();
    return;
  }

  if (accel_elements.back()->end_vel() > 0.999*max_path_velocity) {
    // we essentially already reached max velocity, so there's nothing else
    // to do to speed things up.  Just add the constant section to go the
    // remaining distance and we'll be done
    ap = accel_elements.back();
    printf("creating velocity cruise, from p=%2.3f dist=%2.3f, v=%2.3f\n",
	   ap->end_pos(), distance-ap->end_pos(),ap->end_vel());
    ap = new MacZeroAccel(ap->end_pos(), ap->end_vel(),
			  ap->end_pos() + distance - ap->distance(),
			  ap->end_time());
    duration += ap->duration();
    accel_elements.push_back(ap);
    if (reverse) {
      reverse_accel_pulses();
    }
    condition=4;
    verify_end_conditions();
    return;
  }

  // if we've gotten to this point, then we still have some empty distance
  //   we can use to temporarily boost our speed, so try to accelerate
  //   more at beginning

  if (accel_elements.back()->accel() < max_path_acceleration) {
      // we can still increase the accel pulse amplitude
      
      // make sure we don't exceed max_path_velocity when boosting the accel
      double atime = max_path_acceleration/jmax * PI/2;
      double delta_v = max_path_acceleration * atime;  // a full accel pulse
      if (start_vel + delta_v > max_path_velocity) {
	delta_v = max_path_velocity - start_vel;
      }
      // new starting pulse with greater acceleration
      ap = new MacAccelPulse(0,start_vel,
			     delta_v,
			     max_path_acceleration,
			     jmax,
			     start_time);
      
      duration = ap->duration();
      delete accel_elements.back();
      accel_elements.pop_back();
      accel_elements.push_back(ap);

// REPLACEMENT STARTING PULSE IN PLACE

      // calculate the theoretical ending pulse needed to bring the
      // velocity back down to end_vel;
      double vel_boost = ap->end_vel() - accel_elements.back()->end_vel();
      double end_accel = vel_boost /atime;
      double delta_d = AP_dist(end_vel,end_accel); // distance traversed by this ending pulse
      if (ap->distance() + delta_d > distance) {
	// we don't have enough distance to boost the initial accel amplitude
	//   to max, so recalculate the pulse
	// this equation was derived by solving for two pulses whose
	//   overall distance equals our dist and whose meeting velocities
 	//   are equal
	double accel1 = (distance - atime*(start_vel*(3-2/PI/PI) +end_vel*(1+2/PI/PI)))
	  / (atime*atime*(2-4/PI/PI));
	double accel2 = (end_vel - start_vel) / atime - accel1;
	printf("condition 5: ap->dist=%2.3f, delta_d=%2.3f, distance=%2.3f\n",
	       ap->distance(), delta_d, distance);
	printf("accel1=%2.3f, accel2=%2.3f\n",accel1, accel2);

	delete ap;
	accel_elements.pop_back();
	// accel pulse
	ap = new MacAccelPulse(0,start_vel,
			       accel1*atime,
			       accel1,
			       jmax,
			       start_time);
	duration = ap->duration();
	accel_elements.push_back(ap);
	// decel pulse
	ap = new MacAccelPulse(ap->end_pos(),ap->end_vel(),
			       -accel2*atime,
			       -accel2, -jmax, ap->end_time());
	duration += ap->duration();
	accel_elements.push_back(ap);
	if (reverse) {
	  reverse_accel_pulses();
	}
	condition=5;
	verify_end_conditions();
	return;
      }
      if (ap->distance() + delta_d == distance) {
	// we have exactly enough distance
	ap=new MacAccelPulse(ap->end_pos(),ap->end_vel(),
			     -end_accel*atime,
			     -end_accel,-jmax, ap->end_time());
	duration += ap->duration();
	accel_elements.push_back(ap);
	if (reverse) {
	  reverse_accel_pulses();
	}
	condition=6;
	verify_end_conditions();
	return;
      }
  }
  
  // if we're here, then we've already boosted the amplitude of the initial
  // accel pulse to max, and we still have more empty distance.
  // the next thing to try is to start boosting the decel pulse up to
  // max_path_acceleration while adding a sustain portion to the accel pulse


  double end_accel = max_path_acceleration;
  double atime = max_path_acceleration/jmax * PI/2;

  if (end_vel + end_accel*atime > max_path_velocity) {
    // trying to put a max decel at the end would require the
    // velocity to be exceeded beforehand, so live with less
    end_accel = (max_path_velocity - end_vel) / atime;
  }
  double delta_d = AP_dist(end_vel,end_accel); // bigger distance
  delete accel_elements.back();
  accel_elements.pop_back();
  // remake the starting pulse so that it increases velocity to the point
  // that a single decel pulse at the end will get us back to end_vel
  ap = new MacAccelPulse(0,start_vel,
			 end_vel + end_accel*atime - start_vel,
			 max_path_acceleration,
			 jmax,
			 start_time);
  
  duration = ap->duration();
  accel_elements.push_back(ap);
  ap = accel_elements.front();

  if (ap->distance() + delta_d > distance) {
    // too far, so decrease the amplitude of the ending pulse, and
    // decrease the length of the starting pulse to match.
    // once again, we solve for a pair of pulses whose total distance
    // matches the segment distance and whose velocities match at the
    // meeting point
    end_accel = (distance - (start_vel + end_vel)*2*atime
		 -max_path_acceleration*atime*atime*(1-1/PI/PI)
		 -(start_vel + 0.5*max_path_acceleration*atime) *
		 (end_vel - start_vel - max_path_acceleration*atime)/
		 max_path_acceleration)
      / (atime*atime*(1-1/PI/PI)
	 + (start_vel + 0.5*max_path_acceleration*atime)*atime/max_path_acceleration);
    // recompute the smaller sustain time for the accel pulse
    double const_accel_time = (end_vel - start_vel
			       + (end_accel-max_path_acceleration)*atime)
      / max_path_acceleration;

    // adjust the accel pulse
    ap = accel_elements.back();
    ap->set_constant_accel(const_accel_time);
    duration = ap->duration();

    // build a new ending pulse
    ap = new MacAccelPulse(ap->end_pos(),ap->end_vel(),
			   end_vel - ap->end_vel(),
			   -end_accel,
			   -jmax,
			   ap->end_time());
    duration += ap->duration();
    accel_elements.push_back(ap);
    if (reverse) {
      reverse_accel_pulses();
    }
    condition=7;
    verify_end_conditions();
    return;
  }
  
  if (ap->distance() + delta_d == distance) {
    // perfect
    // build the ending pulse
    ap = new MacAccelPulse(ap->end_pos(), ap->end_vel(),
			   end_vel - ap->end_vel(),
			   -end_accel,
			   -jmax,
			   ap->end_time());
    duration += ap->duration();
    accel_elements.push_back(ap);
    if (reverse) {
      reverse_accel_pulses();
    }
    condition=8;
    verify_end_conditions();
    return;
  }
   
  // make sure we aren't already hitting velocity limit.
  // if we are, there's nothing else we can do, so just add a const
  // section to the middle and wrap it up
  if (end_vel + end_accel*atime > 0.999* max_path_velocity) {
    // build constant section
    double extra_dist = distance - ap->distance() - delta_d;
    ap = new MacZeroAccel(ap->end_pos(),ap->end_vel(),
			  ap->end_pos()+extra_dist,
			  ap->end_time());
    duration += ap->duration();
    accel_elements.push_back(ap);
    // build end
    ap = new MacAccelPulse(ap->end_pos(),ap->end_vel(),
			   end_vel - ap->end_vel(),
			   end_accel,
			   jmax,
			   ap->end_time());
    duration += ap->duration();
    accel_elements.push_back(ap);
    if (reverse) {
      reverse_accel_pulses();
    }
    condition=9;
    verify_end_conditions();
    return;
  }

  // final adjustment
  // at this point, we have a sustained max-acceleration pulse at the
  // beginning that doesn't reach max_path_velocity, and we have planned for an
  // unsustained max-deceleration pulse at the end, with empty distance
  // in between.  increase the sustain portion of both until we either
  // hit the velocity limit or run out of distance

  // double vel_headroom = max_path_velocity - accel_elements.front()->end_vel();
  distance_remaining = distance - accel_elements.front()->distance()
    - AP_dist(accel_elements.front()->end_vel(), max_path_acceleration);
  double a1 = accel_elements.front()->accel();
  double a2 = -max_path_acceleration;
  if (a1 != - a2) {
    throw "Error: expected both accel pulses to be at max accel";
  }
  double sustain_v1 = accel_elements.front()->end_vel() - a1 * atime;
  double sustain_v2 = accel_elements.front()->end_vel() + a2 * atime;

  // first try to fill the distance
  // calculate the sustain time we want to add on to each pulse
  double extension_t = distance_remaining/(sustain_v1 + sustain_v2);

  double transition_vel = accel_elements.front()->end_vel() + a1*extension_t;
  if (transition_vel <= max_path_velocity) {
    // great, it worked.  modify the accel pulse.
    assert(accel_elements.size() == 1);
    ap=accel_elements.front(); // get a pointer to the first pulse
    ap->extend_sustain_time_by(extension_t); // change it
    duration = ap->duration();
    // build the second pulse
    ap=new MacAccelPulse(ap->end_pos(), ap->end_vel(),
			 end_vel - ap->end_vel(),
			 a2, jmax, ap->end_time());  // remake the 2nd
    accel_elements.push_back(ap);
    duration += ap->duration();
    if (reverse) {
      reverse_accel_pulses();
    }
    condition=10;
    verify_end_conditions();
    return;
  }
  // otherwise, we need to recalculate the extension time to stay within
  // velocity limit
  extension_t -= (transition_vel - max_path_velocity) / a1;

  // modify the first pulse
  ap=accel_elements.front();
  ap->extend_sustain_time_by(extension_t);
  duration = ap->duration();

  // add a velocity cruise segment
  distance_remaining -= extension_t * (sustain_v1 + sustain_v2
				       + 0.5 * (a1-a2) * extension_t);
  ap = new MacZeroAccel(ap->end_pos(),ap->end_vel(),
			ap->end_pos() + distance_remaining,
			ap->end_time());
  duration += ap->duration();
  accel_elements.push_back(ap);

  // rebuild the final decel pulse
  ap=new MacAccelPulse(ap->end_pos(),ap->end_vel(),
		       end_vel - ap->end_vel(),
		       a2, jmax, ap->end_time());
  duration += ap->duration();
  accel_elements.push_back(ap);
  
  // all done!!!
  if (reverse) {
    reverse_accel_pulses();
  }
  condition=11;
  verify_end_conditions();
  return;
}

void MacQuinticSegment::reverse_accel_pulses() {
  //  printf("reversing pulses.  Before:\n");
  //  dump();
  // swap the start and end vels back to what they were
  double tmp_vel = end_vel;
  end_vel = start_vel;
  start_vel = tmp_vel;
  
  // reverse the order of the accel elements
  std::vector<MacAccelElement *> temp;
  for (int i=accel_elements.size()-1; i>=0; --i) {
    temp.push_back(accel_elements[(unsigned int)i]);
  }
  accel_elements.clear();
  accel_elements=temp;

  // run through them and reassign the proper values
  double t(start_time);
  double pos(0);
  double vel(start_vel);
  for (unsigned int i=0; i<accel_elements.size(); ++i) {
    double accel = accel_elements[i]->accel();
    accel_elements[i]->reset(pos,vel,-accel,-jmax,t);

    // save ending values to use for start of next segment
    t=accel_elements[i]->end_time();
    pos=accel_elements[i]->end_pos();
    vel=accel_elements[i]->end_vel();
  }
  //  printf("\nAfter:\n");
  //  dump();

  verify_end_conditions();
  return;
}

double MacQuinticSegment::PathVelocity() const {
  return current_path_vel;
}

double MacQuinticSegment::PathAcceleration() const {
  return current_path_accel;
}

void MacQuinticSegment::evaluate(double *y, double *yd, double *ydd, double t) {
  if (duration <0) {
    throw "MacQuinticSegment: BuildProfile() was never called";
  }
  if (t<start_time) {
    t = start_time;
  }
  if (t > start_time + duration) {
    t= start_time + duration;
  }
  unsigned int i=0;
  while (i<accel_elements.size() 
	 && (t-accel_elements[i]->end_time())>.001) {
    ++i;
  }
  if (i == accel_elements.size()) {
    printf("time of t=%2.8f exceeds last element ending time of %2.8f\n",
	   t, accel_elements.back()->end_time());
    throw "MacQuinticSegment: accel elements don't match overall time";
  }
  // first, get the 1-D values (relative position, path vel, path accel),
  double path_pos;
  accel_elements[i]->eval(&path_pos,&current_path_vel,&current_path_accel,t);
  // now, compute the n-D values
  JointPos pos, vel, accel;
  pos = start_pos + direction * path_pos;
  vel = direction * current_path_vel;
  accel = direction * current_path_accel;

  // copy the values into the output pointers
  if (y) pos.cpy(y);
  if (yd) vel.cpy(yd);
  if (ydd) accel.cpy(ydd);
  return;
}

double MacQuinticSegment::calc_time(JointPos value) const {
  return 0;
}

void MacQuinticSegment::dump() {
  printf("MacQuinticSegment [limited by %s, condition %d]:\n",reason,condition);
  MacQuinticElement::dump();
  printf("  start_pos: ");  start_pos.dump();
  printf("  end_pos: "); end_pos.dump();

  printf("  eval at t=%2.3f: ",start_time+duration);
  double *y = (double *) malloc (sizeof(double) * direction.size());
  evaluate(y,0,0,start_time+duration);
  JointPos jpy;
  jpy.resize(direction.size());
  memcpy(&(jpy[0]),y,sizeof(double)*direction.size());
  jpy.dump();
  free(y);

  printf("  start_vel=%2.3f  end_vel=%2.3f\n",start_vel,end_vel);
  printf("  distance=%2.3f\n",distance);
  printf("  accel elements:\n");
  for (unsigned int i=0; i<accel_elements.size(); ++i) {
    accel_elements[i]->dump();
  }
}
