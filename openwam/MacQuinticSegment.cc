#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <assert.h>
#include "MacQuinticSegment.hh"

#define DTMAX MacAccelElement::DTMAX

MacQuinticSegment::MacQuinticSegment( TrajPoint first_p,
				      TrajPoint second_p,
				      JointPos max_joint_vel,
				      JointPos max_joint_accel):
  MacQuinticElement(first_p,second_p) // sets start_pos, end_pos
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
// acceleration rise from zero to amax, starting at velocity v
// (change can be either positive or negative - distance is the same)
inline double MacQuinticSegment::accel_rise_dist(double v, double amax) const {
  // distance required for a single accel rise
  // Equation 3.7 / 3.8 of MacFarlane thesis
  return(fabs(v) * DTMAX + amax*DTMAX*DTMAX*(.25 - 1/PI/PI));
}

// accel_fall_dist: distance covered by a single accel fall from amax to 0,
// starting at velocity v
inline double MacQuinticSegment::accel_fall_dist(double v, double amax) const {
  return(fabs(v) * DTMAX + amax*DTMAX*DTMAX*(.25 + 1/PI/PI));
}

inline double MacQuinticSegment::AP_dist(double v, double amax) const {
  // total distance is an acceleration rise starting at velocity v followed
  //  by an acceleration fall starting at v+amax*dtmax/2 (midpoint v).
  //  return (accel_rise_dist(v,amax) 
  //	  + accel_fall_dist(fabs(v)+fabs(amax)*DTMAX/2,amax));
  return (accel_rise_dist(v,amax) 
	  + accel_fall_dist(v + amax*DTMAX/2, amax));
}

// AP_max_delta_v: given limited distance, what's the max velocity
// change we can achieve with an acceleration pulse?
// (pulse can be either positive or negative - delta v is the same)
inline double MacQuinticSegment::AP_max_delta_v(double v, double amax, double d) {
  d = fabs(d); v=fabs(v); amax=fabs(amax);

  // make sure we can reach max accel and still have room for 
  //   a sustained portion
  if (d <= AP_dist(v,amax)) {
    // there's no room for a sustained portion, so calculate the max pulse
    // we can do
    if (d <= 2 * v * DTMAX) {
      // at this speed, we don't even have room to change acceleration,
      // so we have to leave the speed the way it is
      //      printf("AP_max_delta_v # 1: dist of %2.3f required for a pulse, only %2.3f available\n",2*v*DTMAX,d);
      free(reason);
      reason=strdup("distance limit (too short for any accel)");
      return 0;
    }
    // we'll just do an accel pulse to a lower accel value
    // set the equation in AP_dist equal to d and solve for the new amax
    //    printf("Segment accel pulse reduced from a=%2.3f to %2.3f (distance limit)\n",
    //	   amax,(d-2*v*DTMAX)/DTMAX/DTMAX);
    amax=(d-2*v*DTMAX)/DTMAX/DTMAX;
    if (amax<0) {
      throw "AP_max_delta_v: calculated a negative acceleration";
    }
    
    // now use the new (lower) amax to calculate the new delta v
    //    printf("New max delta_v=%2.3f\n",DTMAX*amax);
    free(reason);
    reason=strdup("distance limit w/ lower amax");
    return DTMAX * amax;
  }
  // we're rising to amax, holding as long as we can, then falling back to 0
  // first, subtract out the distance consumed by the first rise
  d -= accel_rise_dist(v,amax);
  //  printf("AP_max_delta_v: distance remaining = %2.3f\n",d);
  // next, calculate the time that the sustain portion can last
  // uses quadratic formula based on calculating remaining distance
  // d=d_sustain + d_fall  (a function of t_sustain^2 and t_sustain)
  double t_sustain = (-v -1.5*amax*DTMAX
		+ sqrt(v*v + 3*v*amax*DTMAX + 2.25*amax*amax*DTMAX*DTMAX
		       -2*amax*(DTMAX*v+amax*DTMAX*DTMAX*(1/PI/PI+.75)-d)))
    /amax;  // updated 10/4/08
  if (t_sustain < 0) {
    throw "ERROR: negative sustain time in this segment";
  }
  
  // finally, compute the total velocity change
  // we're still just working with positive velocities (directions can be neg)
  double delta_v = amax*DTMAX + t_sustain*amax;
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
  double distance_remaining = distance;
  if (distance == 0) {
    throw "MacQuinticSegment::BuildProfile: distance was zero";
  }
  
  double cruise_start=0;
  double cruise_start_time=start_time;
  duration=0;

  // first, construct the accel pulse that satisfies the velocity change
  if (end_vel != start_vel) {
    ap = new MacAccelPulse(0, start_vel,
			   (end_vel-start_vel),
			   max_path_acceleration,
			   start_time);
    cruise_start=ap->distance();
    cruise_start_time=ap->end_time();
    distance_remaining -= ap->distance();
    if (distance_remaining < -0.005) {
      printf("initial distance was %2.8f, pulse requires %2.8f\n",
	     distance,ap->distance());
      printf("distance_remaining is %2.3f\n",distance_remaining);
      ap->dump();
      throw "Error: velocity change cannot be met with accel limit in available distance";
      
    }
    duration = ap->duration();
    accel_elements.push_back(ap);
    if (distance_remaining ==0) {
      // we exactly met the distance
      if (reverse) {
	reverse_accel_pulses();
      }
      condition=1;
      verify_end_conditions();
      return;
    }
  }
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
			     max_path_acceleration, start_time);
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
			   -max_path_acceleration, ap->end_time());
    
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
      double delta_v = max_path_acceleration * DTMAX;  // a full accel pulse
      if (start_vel + delta_v > max_path_velocity) {
	delta_v = max_path_velocity - start_vel;
      }
      // new starting pulse with greater acceleration
      ap = new MacAccelPulse(0,start_vel,
			     delta_v,
			     max_path_acceleration,
			     start_time);
      
      duration = ap->duration();
      delete accel_elements.back();
      accel_elements.pop_back();
      accel_elements.push_back(ap);

// REPLACEMENT STARTING PULSE IN PLACE

      // calculate the theoretical ending pulse needed to bring the
      // velocity back down to end_vel;
      double vel_boost = ap->end_vel() - accel_elements.back()->end_vel();
      double end_accel = vel_boost /DTMAX;
      double delta_d = AP_dist(end_vel,end_accel); // distance traversed by this ending pulse
      if (ap->distance() + delta_d > distance) {
	// we don't have enough distance to boost the initial accel amplitude
	//   to max, so recalculate the pulse
	// this equation was derived by solving for two pulses whose
	//   overall distance equals our dist and whose meeting velocities
 	//   are equal
	double accel1 = (distance - DTMAX*(start_vel*(3-2/PI/PI) +end_vel*(1+2/PI/PI)))
	  / (DTMAX*DTMAX*(2-4/PI/PI));
	double accel2 = (end_vel - start_vel) / DTMAX - accel1;
	printf("condition 5: ap->dist=%2.3f, delta_d=%2.3f, distance=%2.3f\n",
	       ap->distance(), delta_d, distance);
	printf("accel1=%2.3f, accel2=%2.3f\n",accel1, accel2);

	delete ap;
	accel_elements.pop_back();
	// accel pulse
	ap = new MacAccelPulse(0,start_vel,
			       accel1*DTMAX,
			       accel1, start_time);
	duration = ap->duration();
	accel_elements.push_back(ap);
	// decel pulse
	ap = new MacAccelPulse(ap->end_pos(),ap->end_vel(),
			       -accel2*DTMAX,
			       -accel2, ap->end_time());
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
			     -end_accel*DTMAX,
			     -end_accel,ap->end_time());
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
  if (end_vel + end_accel*DTMAX > max_path_velocity) {
    // trying to put a max decel at the end would require the
    // velocity to be exceeded beforehand, so live with less
    end_accel = (max_path_velocity - end_vel) / DTMAX;
  }
  double delta_d = AP_dist(end_vel,end_accel); // bigger distance
  delete accel_elements.back();
  accel_elements.pop_back();
  // remake the starting pulse so that it increases velocity to the point
  // that a single decel pulse at the end will get us back to end_vel
  ap = new MacAccelPulse(0,start_vel,
			 end_vel + end_accel*DTMAX - start_vel,
			 max_path_acceleration,
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
    end_accel = (distance - (start_vel + end_vel)*2*DTMAX
		 -max_path_acceleration*DTMAX*DTMAX*(1-1/PI/PI)
		 -(start_vel + 0.5*max_path_acceleration*DTMAX) *
		 (end_vel - start_vel - max_path_acceleration*DTMAX)/
		 max_path_acceleration)
      / (DTMAX*DTMAX*(1-1/PI/PI)
	 + (start_vel + 0.5*max_path_acceleration*DTMAX)*DTMAX/max_path_acceleration);
    // recompute the smaller sustain time for the accel pulse
    double const_accel_time = (end_vel - start_vel
			       + (end_accel-max_path_acceleration)*DTMAX)
      / max_path_acceleration;

    // adjust the accel pulse
    ap = accel_elements.back();
    ap->set_constant_accel(const_accel_time);
    duration = ap->duration();

    // build a new ending pulse
    ap = new MacAccelPulse(ap->end_pos(),ap->end_vel(),
			   end_vel - ap->end_vel(),
			   -end_accel,
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
  if (end_vel + end_accel*DTMAX > 0.999* max_path_velocity) {
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
  double sustain_v1 = accel_elements.front()->end_vel() - a1 * DTMAX;
  double sustain_v2 = accel_elements.front()->end_vel() + a2 * DTMAX;

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
			 a2, ap->end_time());  // remake the 2nd
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
		       a2, ap->end_time());
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
    accel_elements[i]->reset(pos,vel,-accel,t);

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
