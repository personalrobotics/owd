#include "WSTraj.h"
#include <math.h>
#include <LinearMath/btQuaternion.h>
#define WRIST
#include <openwam/Link.hh>
#include <openwam/Kinematics.hh>
#define PEAK_CAN
#include <openwamdriver.h>

bool WSTraj::AddWSTraj(gfe_owd_plugin::AddWSTraj::Request &req,
		       gfe_owd_plugin::AddWSTraj::Response &res) {
  // compute a new workspace trajectory
  try {
    WSTraj *newtraj = new WSTraj(req);
    newtraj->CancelOnStall=(req.options & pr_msgs::JointTraj::opt_CancelOnStall);
    newtraj->CancelOnForceInput=(req.options & pr_msgs::JointTraj::opt_CancelOnForceInput);
    newtraj->CancelOnTactileInput=(req.options & pr_msgs::JointTraj::opt_CancelOnTactileInput);

    // send it to the arm
    res.id = OWD::Plugin::AddTrajectory(newtraj,res.reason);
    if (res.id > 0) {
      res.ok=true;
      res.reason="";
    } else {
      delete newtraj;
      res.ok=false;
    }
  } catch (const char *err) {
    res.ok=false;
    res.reason=err;
    res.id=0;
  }

  // always return true for the service call so that the client knows that
  // the call was processed, as opposed to there being a
  // network communication error.
  // the client will examine the "ok" field to see if the command was
  // actually successful
  return true;
}

bool WSTraj::ForceFeedbackNextTrajSrv(pr_msgs::Reset::Request &req,
					     pr_msgs::Reset::Response &res) {
  ForceFeedbackNextTraj=true;
  res.ok=true;
  res.reason="";
  return true;
}


WSTraj::WSTraj(gfe_owd_plugin::AddWSTraj::Request &wst) 
  : OWD::Trajectory("WS Traj"),
    driving_force_direction(wst.wrench.force.x,
			    wst.wrench.force.y,
			    wst.wrench.force.z,
			    wst.wrench.torque.x,
			    wst.wrench.torque.y,
			    wst.wrench.torque.z),
    endpoint_translation(wst.endpoint_change.position.x,
			 wst.endpoint_change.position.y,
			 wst.endpoint_change.position.z),
    movement_direction(endpoint_translation),
    max_linear_vel(wst.max_linear_velocity),
    max_linear_accel(max_linear_vel/wst.min_accel_time),
    max_angular_vel(wst.max_angular_velocity),
    max_angular_accel(max_angular_vel/wst.min_accel_time),
    parseg(0,0,0,1),
    force_scale(0.1),
    AFTraj(NULL),
    ForceFeedback(ForceFeedbackNextTraj)
{
  // reset our flag before the next trajectory
  ForceFeedbackNextTraj=false;
  driving_forcetorque = driving_force_direction.normalize();
  if (driving_forcetorque > 0) {
    ROS_INFO("WSTraj will use a feedforward force of x=%2.2f y=%2.2f z=%2.2f",
	     driving_forcetorque*driving_force_direction.v[0],
	     driving_forcetorque*driving_force_direction.v[1],
	     driving_forcetorque*driving_force_direction.v[2]);
    ROS_INFO("WSTraj will use a feedforward torque of x=%2.2f y=%2.2f z=%2.2f",
	     driving_forcetorque*driving_force_direction.w[0],
	     driving_forcetorque*driving_force_direction.w[1],
	     driving_forcetorque*driving_force_direction.w[2]);
  }
  movement_direction.normalize();

  start_position = OWD::JointPos(wst.starting_config);
  OWD::JointPos current_pos(gfeplug->target_arm_position);
  if (start_position.closeto(current_pos)) {
    start_position = current_pos;
  }
  end_position = OWD::JointPos(wst.ending_config);
  joint_change = OWD::JointPos(wst.ending_config) - start_position;

  //  if (gfeplug->target_arm_position.size() != OWD::Link::Ln) {
  //    ROS_ERROR("Warning: number of joints doesn't match number of links");
  //  }

  OWD::Link mylinks[gfeplug->target_arm_position.size()];
  for (unsigned int i=0; i<=gfeplug->target_arm_position.size(); ++i) {
    // copy the link geometry from the WAM class
    mylinks[i]=OWD::WamDriver::owam->links[i];
  }
  for (unsigned int i=0; i<gfeplug->target_arm_position.size(); ++i) {
    // update it to use our starting config
    mylinks[i+1].theta(start_position[i]);
  }
  // use our own links to compute the target endpoint pose
  start_endpoint=OWD::Kinematics::forward_kinematics(mylinks);
  
  // validate our start_endpoint
  //  so3 ep_rot_diff = (so3)((SO3)start_endpoint * (! (SO3)gfeplug->endpoint));
  //  R3 ep_trans_diff = (R3)gfeplug->endpoint - (R3)start_endpoint;
  //  ROS_INFO("Start reference endpoint differs from actual by x=%1.3f y=%1.3f z=%1.3f theta=%1.3f",
  //	   ep_trans_diff[0],
  //	   ep_trans_diff[1],
  //	   ep_trans_diff[2],
  //	   ep_rot_diff.t());
  
  // translate the desired endpoint rotation by extracting the
  // quanternion and converting it to axis/angle format
  btQuaternion ep_qrot(wst.endpoint_change.orientation.x,
		       wst.endpoint_change.orientation.y,
		       wst.endpoint_change.orientation.z,
		       wst.endpoint_change.orientation.w);
  endpoint_rotation = so3(R3(ep_qrot.getAxis().x(),
			     ep_qrot.getAxis().y(),
			     ep_qrot.getAxis().z()),
			  ep_qrot.getAngle());

  if (wst.ApplyForce) {
    AFTraj = new ApplyForceTraj(R3(wst.af_x,
				   wst.af_y,
				   wst.af_z),
				wst.af_f);
    if (wst.af_rotational_compliance) {
      AFTraj->rotational_leeway = 10.0 / 180.0 * 3.14159; // 10 degrees
    }
    if (wst.Vibrate) {
      AFTraj->SetVibration(wst.vibrate_hand_x,
			   wst.vibrate_hand_y,
			   wst.vibrate_hand_z,
			   wst.vibrate_amplitude_m,
			   wst.vibrate_frequency_hz);
    }
  }

  // #define SIMULATION
#ifdef SIMULATION
  ROS_INFO_NAMED("wstraj","Moving in direction [%1.3f, %1.3f, %1.3f]",
		 endpoint_translation[0],
		 endpoint_translation[1],
		 endpoint_translation[2]);
#endif // SIMULATION

  // figure out whether our speed will be governed by the
  // linear velocity limit or the angular velocity limit.
  // The ParabolicSegments can be created with zero distance,
  // so we'll go ahead and create one for the translation and
  // a second one for the rotation, then compare them to see
  // which one to use.

  // We want to end up with a "unit trajectory" that goes from
  // zero to one, so we'll scale down the velocity and accel
  // to match.  Then the resulting unit trajectory can be used
  // to gauge the progress of both the rotation and translation.

  if ((endpoint_translation.norm() == 0) &&
      (endpoint_rotation.t() == 0)) {
    throw "No motion defined by endpoint_change field";
  }

  OWD::ParabolicSegment trans_seg(0,0,0,1);
  if (endpoint_translation.norm() > 0) {
    double distance_factor=1.0/endpoint_translation.norm();
    trans_seg.fit_curve(wst.max_linear_velocity * distance_factor,
			wst.max_linear_velocity * distance_factor
			/ wst.min_accel_time);
    ROS_DEBUG_NAMED("wstraj","Building WSTraj: trans distance is %1.3f, vel %2.3f, accel %2.3f, time %2.3f",
		    endpoint_translation.norm(),
		    wst.max_linear_velocity * distance_factor,
		    wst.max_linear_velocity * distance_factor
		    / wst.min_accel_time,
		    trans_seg.end_time);
  } else {
    trans_seg.end_time=0;
  }

  OWD::ParabolicSegment rot_seg(0,0,0,1);
  if (endpoint_rotation.t() != 0) {
    double distance_factor=1.0/endpoint_rotation.t();
    rot_seg.fit_curve(wst.max_angular_velocity * distance_factor,
		      wst.max_angular_velocity * distance_factor
		      / wst.min_accel_time);
    ROS_DEBUG_NAMED("wstraj","Building WSTraj: rot distance is %1.3f, vel %2.3f, accel %2.3f, time %2.3f",
		    endpoint_rotation.t(),
		    wst.max_angular_velocity * distance_factor,
		    wst.max_angular_velocity * distance_factor
		    / wst.min_accel_time,
		    rot_seg.end_time);
  } else {
    rot_seg.end_time=0;
  }

  if (trans_seg.end_time > rot_seg.end_time) {
    parseg = trans_seg;
    vel_factor = 1.0*wst.max_linear_velocity;
    ROS_DEBUG_NAMED("wstraj","Using linear velocity; vel_factor is %2.3f",
		    vel_factor);
  } else {
    parseg = rot_seg;
    vel_factor = 1.0*wst.max_angular_velocity;
    ROS_DEBUG_NAMED("wstraj","Using angular velocity; vel_factor is %2.3f",
		    vel_factor);
  }
  parseg.dump();

  gfeplug->recorder->reset();
  // log values:
  //    0: how far ahead (behind) we are from computed time
  //    1: force_scale
  //    2: length of ft_error
  //    3: magnitude of total feedback torque vector
  //    4: X force error
  //    5: Y force error
  //    6: Z force error
  //    7: desired Z force
  //    8: actual Z force
  //    9: derivative of Z force
  //   10: stapling success
  //   11-16: desired feedforward force/torque
  gfeplug->net_force.data.resize(17);
  gfeplug->net_force.data[10]=0;
  gfeplug->current_traj=this;
}

WSTraj::~WSTraj() {
  gfeplug->flush_recorder_data = true;
  gfeplug->current_traj=NULL;
}

void WSTraj::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
  if (ForceFeedback) {
    // use our older version as requested
    force_feedback_evaluate(tc,dt);
    return;
  }
 
  if (driving_forcetorque > 0) {
    // When we come into this function, we will ideally be exactly at
    // the desired point on the trajectory.  We will have two kinds of
    // errors: lateral error (normal to the driving force direction),
    // and longitudinal error (as a result of traveling too fast or
    // too slow).  We will correct all of the lateral error as a joint
    // position adjustment, and we will correct the speed error by
    // modulating the force we are applying.


    // increment our time by dt, to figure out where we want to be
    // based on how much time elapsed since the last call
    time += dt;

    // check for ending condition
    if (time >= parseg.end_time) {
      runstate = DONE;
      return;
    }

    // calculate our trajectory values
    double dist, vel, accel;
    parseg.evaluate(dist, vel, accel, time);
    // we'll use the segment velocity (relative to the max vel) to scale
    // the feedforward force, so that we get smooth starts and stops
    double force_percent = vel / (parseg.dir * parseg.accel * parseg.time_a);

    R3 target_position = (R3)start_endpoint + endpoint_translation * dist;
    // calculate the endpoint position error
    R3 position_error = target_position - (R3)gfeplug->endpoint;

    // calculate our trajectory rotation
    so3 relative_rotation(endpoint_rotation.w(), endpoint_rotation.t() * dist);
    SO3 target_rotation = (SO3)relative_rotation * (SO3)start_endpoint;

    // Compute the endpoint rotation error by taking the net rotation from
    // the current orientation to the target orientation and converting
    // it to axis-angle format
    so3 rotation_error = (so3)(target_rotation * (! (SO3)gfeplug->endpoint));
    // represent as rotation about each of the axes
    R3 rotation_correction = rotation_error.t() * rotation_error.w();

    R3 position_correction = position_error;

    // calculate our endpoint correction
    R6 endpos_correction(position_correction,rotation_correction);
    OWD::JointPos joint_correction(tc.q.size());
    try {
      joint_correction = 
	gfeplug->JacobianPseudoInverse_times_vector(endpos_correction);
      for (unsigned int j=0; j<joint_correction.size(); ++j) {
	if (isnan(joint_correction[j])) {
	  joint_correction[j]=0;
	}
      }
    } catch (const char *err) {
      // no valid Jacobian, for whatever reason, so abort the trajectory
      // and leave the joint values unchanged
      ROS_WARN_NAMED("wstraj","JacobianPseudoInverse failed when correcting endpoint");
      runstate = ABORT;
      return;
    }
    
    // calculate the nullspace joint correction to keep the rest of
    // the arm out of trouble
    OWD::JointPos target_jointpos = start_position + joint_change * dist;
    OWD::JointPos jointpos_error = target_jointpos - tc.q;
    try {
      OWD::JointPos configuration_correction
	= gfeplug->Nullspace_projection(jointpos_error);
      for (unsigned int j=0; j<configuration_correction.size(); ++j) {
	if (isnan(configuration_correction[j])) {
	  configuration_correction[j]=0;
	}
      }
      joint_correction += configuration_correction;
    } catch (const char *err) {
      // abort the trajectory if we can't correct the configuration
      ROS_WARN_NAMED("wstraj","Nullspace projection failed when correcting configuration");
      runstate = ABORT;
      return;
    }

    // Calculate the feedforward torques to produce our driving force
    R6 desired_ft = force_percent*driving_forcetorque*driving_force_direction;
    gfeplug->net_force.data[11]=desired_ft.v[0];
    gfeplug->net_force.data[12]=desired_ft.v[1];
    gfeplug->net_force.data[13]=desired_ft.v[2];
    gfeplug->net_force.data[14]=desired_ft.w[0];
    gfeplug->net_force.data[15]=desired_ft.w[1];
    gfeplug->net_force.data[16]=desired_ft.w[2];
    OWD::JointPos force_feedforward_torques = gfeplug->JacobianTranspose_times_vector(desired_ft);

    // return the new joint positions and the force
    tc.q += joint_correction;
    tc.t = force_feedforward_torques;

  } else {
    // movement is purely time-based

    static R3 total_endpoint_movement;
    // advance our time by dt
    time += dt;
    if (time > parseg.end_time) {
      time = parseg.end_time;
      runstate = DONE;
    }

    // calculate where we should be.
    // our trajectory is a "unit trajectory" with a position that goes from
    // zero to one, so we can use it as a fraction to scale both our
    // position and rotation progress.
    // the velocity and accel returned by the trajectory have actual units,
    // so we have to normalize them by vel_factor (which we recorded in our
    // constructor) before scaling them.
    double dist, vel, accel;
    parseg.evaluate(dist, vel, accel, time);

#ifdef SIMULATION
    static R3 last_endpoint = (R3) gfeplug->endpoint;
    R3 ep_movement = (R3) gfeplug->endpoint - last_endpoint;
    total_endpoint_movement+= ep_movement;
    ROS_DEBUG_STREAM_NAMED("wstraj","Endpoint movement since last time: " << ep_movement);
    last_endpoint = (R3) gfeplug->endpoint;

    ROS_DEBUG_NAMED("wstraj","WSTraj eval: dist=%2.3f, vel=%2.3f, accel=%2.3f, time=%2.3f, dt=%0.3f",
		    dist,vel,accel,time,dt);
#endif // SIMULATION

    /**********************************************
     *   calculate the endpoint pose correction   *
     **********************************************/
    R3 target_position = (R3)start_endpoint + endpoint_translation * dist;
    // calculate the endpoint position correction
    R3 position_correction = target_position - (R3)gfeplug->endpoint;

    // calculate our trajectory rotation
    so3 relative_rotation(endpoint_rotation.w(), endpoint_rotation.t() * dist);
    SO3 target_rotation = (SO3)relative_rotation * (SO3)start_endpoint;

    // Compute the endpoint rotation error by taking the net rotation from
    // the current orientation to the target orientation and converting
    // it to axis-angle format
    so3 rotation_error = (so3)(target_rotation * (! (SO3)gfeplug->endpoint));
    // represent as rotation about each of the axes
    R3 rotation_correction = rotation_error.t() * rotation_error.w();
    
    // calculate the joint change required to correct the endpoint pose
    R6 endpos_correction(position_correction,rotation_correction);
    OWD::JointPos joint_correction(tc.q.size());
    try {
      joint_correction = 
	gfeplug->JacobianPseudoInverse_times_vector(endpos_correction);
      for (unsigned int j=0; j<joint_correction.size(); ++j) {
	if (isnan(joint_correction[j])) {
	  joint_correction[j]=0;
	}
      }
    } catch (const char *err) {
      // no valid Jacobian, for whatever reason, so stop the trajectory
      // and leave the joint values unchanged
      ROS_WARN_NAMED("wstraj","JacobianPseudoInverse failed when correcting endpoint");
      runstate = STOP;
      return;
    }
#ifdef SIMULATION
    ROS_INFO_NAMED("wstraj","Taking a WS step of [%1.4f, %1.4f, %1.4f]",
		   dist*endpoint_translation[0],
		   dist*endpoint_translation[1],
		   dist*endpoint_translation[2]);
    ROS_INFO_NAMED("wstraj","Position correction is [%1.4f, %1.4f, %1.4f]",
		   endpos_correction[0],
		   endpos_correction[1],
		   endpos_correction[2]);
    ROS_DEBUG_STREAM_NAMED("wstraj","Position correction is " << position_correction);
    ROS_DEBUG_STREAM_NAMED("wstraj","target rot" << std::endl << (so3)target_rotation);
    ROS_DEBUG_STREAM_NAMED("wstraj","rotation error" << std::endl << rotation_error);
    ROS_DEBUG_STREAM_NAMED("wstraj","Rotation correction is " << rotation_correction);
    ROS_DEBUG_NAMED("wstraj","Joint correction is %s", joint_correction.sdump());
#endif
    
    // OWD needs to know our current joint velocities and accelerations
    // in order to calculate the feedforward torque values from the
    // dynamics model.  These are calculated for the ideal case, as
    // if we had no position error.  We'll get them by creating an
    // endpoint velocity (acceleration) vector and multiplying it
    // by the Jacobian Pseudo Inverse.
    double current_linear_vel=vel * vel_factor * max_linear_vel;
    double current_linear_accel=accel * vel_factor * max_linear_accel;
    double current_angular_vel=vel * vel_factor * max_angular_vel;
    // double current_angular_accel=accel * vel_factor * max_angular_accel;
    R3 endpos_trans_vel(0,0,0); // default is zero vel
    if (endpoint_translation.norm() > 0) { // avoid div by zero
      endpos_trans_vel = endpoint_translation * current_linear_vel
	/ endpoint_translation.norm();
    }
    so3 endpos_rot_vel(R3(1,0,0),0); // default is zero vel
    if (endpoint_rotation.t() != 0) { // avoid div by zero
      endpos_rot_vel = endpoint_rotation * (current_angular_vel
					    / endpoint_rotation.t());
    }
    R6 endpos_vel(endpos_trans_vel,R3(endpos_rot_vel.t() * endpos_rot_vel.w()));
    OWD::JointPos joint_vel(tc.q.size());
    try {
      joint_vel = gfeplug->JacobianPseudoInverse_times_vector(endpos_vel);
    } catch (const char *err) {
      // no valid Jacobian, for whatever reason, so stop the trajectory
      // and leave the joint values unchanged
      ROS_WARN_NAMED("wstraj","JacobianPseudoInverse failed when calculating joint vel/accel");
      runstate = STOP;
      return;
    }

    /**********************************************
     *  calculate the nullspace joint correction  *
     **********************************************/
    // first, figure out where we are supposed to be
    OWD::JointPos target_jointpos = start_position + joint_change * dist;

    if (!AFTraj) {
      // if we are using ApplyForce at the same time, it will do its
      // own nullspace correction as long as we tell it the config we want
      OWD::JointPos jointpos_error = target_jointpos - tc.q;
      try {
	OWD::JointPos configuration_correction
	  = gfeplug->Nullspace_projection(jointpos_error);
	for (unsigned int j=0; j<configuration_correction.size(); ++j) {
	  if (isnan(configuration_correction[j])) {
	    configuration_correction[j]=0;
	  }
	}
	joint_correction += configuration_correction;
#ifdef SIMULATION
	ROS_DEBUG_NAMED("wstraj","Configuration corrected by %s",
			configuration_correction.sdump());
#endif
      } catch (const char *err) {
	// stop the trajectory if we can't correct the configuration
	ROS_WARN_NAMED("wstraj","Nullspace projection failed when correcting configuration");
	runstate = STOP;
	return;
      }
    }

    /**********************************************
     *  set our new values                        *
     **********************************************/
    if (AFTraj) {

      // modify the AFTraj target position and target configuration to
      // match our current values
      AFTraj->start_position = target_jointpos;
      AFTraj->endpoint_target = SE3(target_rotation,
				    target_position);

      // Let AFTraj compute the positions and torques needed to reach the
      // desired position while also maintaining the force
      AFTraj->evaluate(tc, dt);
  
    } else {

      // apply the pure position corrections
      for (unsigned int j=0; (j<joint_correction.size()) 
	     && (j<tc.q.size()); ++j) {
	tc.q[j] += joint_correction[j];
      }
    }


    // apply the joint vels and use them to calculate the accels, too.
    for (unsigned int j=0; (j<joint_vel.size()) 
	   && (j<tc.qd.size())
	   && (j<tc.qdd.size()); ++j) {
      tc.qd[j]=joint_vel[j];
      // since the ratio of lin_accel to lin_vel is the same as the ratio
      // of ang_accel to ang_vel, we can just scale all of the resulting
      // joint vels to get joint accels without having to push an accel
      // vector through the Jacobian Pseudo-Inverse again
      tc.qdd[j]=joint_vel[j] / current_linear_vel * current_linear_accel;
    }
  }
  
  // keep the end position tracking our current position (OWD will use this
  // to hold the point when we're done, and we have no guarrantee that we will
  // get to the requested config, so it's better to use the config we are in)
  end_position = tc.q;

  gfeplug->log_data(gfeplug->net_force.data);
  return;
}

// older version that modulates the force based on whether we are ahead of
// or behind our trajectory time.
// this one works better for stapling, so it's still here
void WSTraj::force_feedback_evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
 
  if (driving_forcetorque > 0) {
    // When we come into this function, we will ideally be exactly at
    // the desired point on the trajectory.  We will have two kinds of
    // errors: lateral error (normal to the driving force direction),
    // and longitudinal error (as a result of traveling too fast or
    // too slow).  We will correct all of the lateral error as a joint
    // position adjustment, and we will correct the speed error by
    // modulating the force we are applying.


    // increment our time by dt, to figure out where we want to be
    // based on how much time elapsed since the last call
    time += dt;

    // map our current position to the closest point
    // on the trajectory.  this does not move us forward or
    // backward in time.
    double traj_progress = (((R3)gfeplug->endpoint - (R3)start_endpoint) * 
			    movement_direction);

    bool ignore_longitudinal_error(true);
    double traj_time;
    if (traj_progress < 0) {
      // some external force pushed us back behind the start point,
      // so instead of doing our force-based control we will correct
      // the entire position error to try to pull us back to the start.
      traj_time = 0;
      time = 0;
      ignore_longitudinal_error=false;
    } else {
      R3 traj_progress_vector = traj_progress * movement_direction;
      double traj_percent = traj_progress_vector.norm() / endpoint_translation.norm();
      if ((traj_percent < 0) || (traj_percent > 1)) {
	// Bad calculation of trajectory progress
	runstate=ABORT;
	return;
      }
      traj_time = parseg.calc_time(traj_percent);
    }

    // check for ending condition
    if (traj_time >= parseg.end_time) {
      time = parseg.end_time;
      runstate = DONE;
      return;
    }

    // calculate our trajectory values
    double dist, vel, accel;
    parseg.evaluate(dist, vel, accel, traj_time);

    R3 target_position = (R3)start_endpoint + endpoint_translation * dist;
    // calculate the endpoint position error
    R3 position_error = target_position - (R3)gfeplug->endpoint;

    // calculate our trajectory rotation
    so3 relative_rotation(endpoint_rotation.w(), endpoint_rotation.t() * dist);
    SO3 target_rotation = (SO3)relative_rotation * (SO3)start_endpoint;

    // Compute the endpoint rotation error by taking the net rotation from
    // the current orientation to the target orientation and converting
    // it to axis-angle format
    so3 rotation_error = (so3)(target_rotation * (! (SO3)gfeplug->endpoint));
    // represent as rotation about each of the axes
    R3 rotation_correction = rotation_error.t() * rotation_error.w();

    R3 position_correction;
    if (ignore_longitudinal_error) {
      // break down the position error into lateral and longitudinal components
      R3 longitudinal_error = (position_error * movement_direction) 
	* movement_direction;
      R3 lateral_error = position_error - longitudinal_error;
      position_correction = lateral_error;
    } else {
      position_correction = position_error;
    }

    // calculate our endpoint correction to keep us on course by doing the
    // full rotation correction but only correcting the lateral error.  We
    // will rely on the force controller to move us forward.
    R6 endpos_correction(position_correction,rotation_correction);
    OWD::JointPos joint_correction(tc.q.size());
    try {
      joint_correction = 
	gfeplug->JacobianPseudoInverse_times_vector(endpos_correction);
      for (unsigned int j=0; j<joint_correction.size(); ++j) {
	if (isnan(joint_correction[j])) {
	  joint_correction[j]=0;
	}
      }
    } catch (const char *err) {
      // no valid Jacobian, for whatever reason, so abort the trajectory
      // and leave the joint values unchanged
      ROS_WARN_NAMED("wstraj","JacobianPseudoInverse failed when correcting endpoint");
      runstate = ABORT;
      return;
    }
    
    // calculate the nullspace joint correction to keep the rest of
    // the arm out of trouble
    OWD::JointPos target_jointpos = start_position + joint_change * dist;
    OWD::JointPos jointpos_error = target_jointpos - tc.q;
    try {
      OWD::JointPos configuration_correction
	= gfeplug->Nullspace_projection(jointpos_error);
      for (unsigned int j=0; j<configuration_correction.size(); ++j) {
	if (isnan(configuration_correction[j])) {
	  configuration_correction[j]=0;
	}
      }
      joint_correction += configuration_correction;
    } catch (const char *err) {
      // abort the trajectory if we can't correct the configuration
      ROS_WARN_NAMED("wstraj","Nullspace projection failed when correcting configuration");
      runstate = ABORT;
      return;
    }

    // Figure out if we are ahead of or behind where we are supposed
    // to be, and adjust our force scale accordingly.  If we are behind,
    // increment our force scale by an appropriate amount, and decrement
    // time back to the actual position so that we take the right-sized
    // step for the next iteration.  If we are ahead, decrement our
    // force scale.
    double ahead = traj_time - time;
    if (ahead < 0) {
      // we are behind schedule, so apply the full force
      force_scale =1;
      if (traj_time + 0.1 < time) {
	time = traj_time + 0.1; // don't fall behind by more than 0.1
      }
    } else {
      if (ahead > 0.1) {
	force_scale = 0;  // zero force when 0.1 sec ahead
      } else {
	force_scale = (0.1 - ahead) / 0.1;  // linear ratio
      }
    }
    gfeplug->net_force.data[0] = traj_time - time;
    gfeplug->net_force.data[1] = force_scale;

    // Calculate the force to apply based on the max force, the force
    // scale, and the trajectory's relative velocity.  By using the
    // trajectory's velocity we will get a soft start at the beginning
    // and a soft stop at the end.  But we have to bump vel up a little
    // at the beginning and end since it starts and ends at zero.
    if (vel/parseg.max_vel < 0.05) {
      vel = parseg.max_vel * 0.05;
    }
    R6 desired_ft = force_scale * driving_forcetorque * 
      driving_force_direction;
    OWD::JointPos force_feedforward_torques = gfeplug->JacobianTranspose_times_vector(desired_ft);

    // project the actual FT onto the direction we care about
    R6 actual_ft = gfeplug->workspace_forcetorque() * driving_force_direction
      * driving_force_direction;
    R6 ft_error = desired_ft - actual_ft;
    gfeplug->net_force.data[2] = ft_error.norm();
    gfeplug->net_force.data[4] = ft_error.v[0];
    gfeplug->net_force.data[5] = ft_error.v[1];
    gfeplug->net_force.data[6] = ft_error.v[2];
    gfeplug->net_force.data[7] = desired_ft.v[2];
    gfeplug->net_force.data[8] = actual_ft.v[2];
    static double last_z_force( actual_ft.v[0]);
    double Z_diff = actual_ft.v[0] - last_z_force;
    last_z_force=actual_ft.v[0];
    if (fabs(Z_diff) < 30) {
      gfeplug->net_force.data[9] = Z_diff;
      if (Z_diff < -0.7) {
	gfeplug->net_force.data[10]=1;
      }
    }
    
    OWD::JointPos force_feedback_torques(tc.q.size());
    force_feedback_torques = force_controller.control(ft_error);
    gfeplug->net_force.data[3] = force_feedback_torques.length();
    // return the new joint positions and the force
    tc.q += joint_correction;
    tc.t = force_feedforward_torques + force_feedback_torques;

  } else {
    // movement is purely time-based

    static R3 total_endpoint_movement;
    // advance our time by dt
    time += dt;
    if (time > parseg.end_time) {
      time = parseg.end_time;
      runstate = DONE;
    }

    // calculate where we should be.
    // our trajectory is a "unit trajectory" with a position that goes from
    // zero to one, so we can use it as a fraction to scale both our
    // position and rotation progress.
    // the velocity and accel returned by the trajectory have actual units,
    // so we have to normalize them by vel_factor (which we recorded in our
    // constructor) before scaling them.
    double dist, vel, accel;
    parseg.evaluate(dist, vel, accel, time);

#ifdef SIMULATION
    static R3 last_endpoint = (R3) gfeplug->endpoint;
    R3 ep_movement = (R3) gfeplug->endpoint - last_endpoint;
    total_endpoint_movement+= ep_movement;
    ROS_DEBUG_STREAM_NAMED("wstraj","Endpoint movement since last time: " << ep_movement);
    last_endpoint = (R3) gfeplug->endpoint;

    ROS_DEBUG_NAMED("wstraj","WSTraj eval: dist=%2.3f, vel=%2.3f, accel=%2.3f, time=%2.3f, dt=%0.3f",
		    dist,vel,accel,time,dt);
#endif // SIMULATION

    /**********************************************
     *   calculate the endpoint pose correction   *
     **********************************************/
    R3 target_position = (R3)start_endpoint + endpoint_translation * dist;
    // calculate the endpoint position correction
    R3 position_correction = target_position - (R3)gfeplug->endpoint;

    // calculate our trajectory rotation
    so3 relative_rotation(endpoint_rotation.w(), endpoint_rotation.t() * dist);
    SO3 target_rotation = (SO3)relative_rotation * (SO3)start_endpoint;

    // Compute the endpoint rotation error by taking the net rotation from
    // the current orientation to the target orientation and converting
    // it to axis-angle format
    so3 rotation_error = (so3)(target_rotation * (! (SO3)gfeplug->endpoint));
    // represent as rotation about each of the axes
    R3 rotation_correction = rotation_error.t() * rotation_error.w();
    
    // calculate the joint change required to correct the endpoint pose
    R6 endpos_correction(position_correction,rotation_correction);
    OWD::JointPos joint_correction(tc.q.size());
    try {
      joint_correction = 
	gfeplug->JacobianPseudoInverse_times_vector(endpos_correction);
      for (unsigned int j=0; j<joint_correction.size(); ++j) {
	if (isnan(joint_correction[j])) {
	  joint_correction[j]=0;
	}
      }
    } catch (const char *err) {
      // no valid Jacobian, for whatever reason, so stop the trajectory
      // and leave the joint values unchanged
      ROS_WARN_NAMED("wstraj","JacobianPseudoInverse failed when correcting endpoint");
      runstate = STOP;
      return;
    }
#ifdef SIMULATION
    ROS_INFO_NAMED("wstraj","Taking a WS step of [%1.4f, %1.4f, %1.4f]",
		   dist*endpoint_translation[0],
		   dist*endpoint_translation[1],
		   dist*endpoint_translation[2]);
    ROS_INFO_NAMED("wstraj","Position correction is [%1.4f, %1.4f, %1.4f]",
		   endpos_correction[0],
		   endpos_correction[1],
		   endpos_correction[2]);
    ROS_DEBUG_STREAM_NAMED("wstraj","Position correction is " << position_correction);
    ROS_DEBUG_STREAM_NAMED("wstraj","target rot" << std::endl << (so3)target_rotation);
    ROS_DEBUG_STREAM_NAMED("wstraj","rotation error" << std::endl << rotation_error);
    ROS_DEBUG_STREAM_NAMED("wstraj","Rotation correction is " << rotation_correction);
    ROS_DEBUG_NAMED("wstraj","Joint correction is %s", joint_correction.sdump());
#endif
    
    // OWD needs to know our current joint velocities and accelerations
    // in order to calculate the feedforward torque values from the
    // dynamics model.  These are calculated for the ideal case, as
    // if we had no position error.  We'll get them by creating an
    // endpoint velocity (acceleration) vector and multiplying it
    // by the Jacobian Pseudo Inverse.
    double current_linear_vel=vel * vel_factor * max_linear_vel;
    double current_linear_accel=accel * vel_factor * max_linear_accel;
    double current_angular_vel=vel * vel_factor * max_angular_vel;
    // double current_angular_accel=accel * vel_factor * max_angular_accel;
    R3 endpos_trans_vel(0,0,0); // default is zero vel
    if (endpoint_translation.norm() > 0) { // avoid div by zero
      endpos_trans_vel = endpoint_translation * current_linear_vel
	/ endpoint_translation.norm();
    }
    so3 endpos_rot_vel(R3(1,0,0),0); // default is zero vel
    if (endpoint_rotation.t() != 0) { // avoid div by zero
      endpos_rot_vel = endpoint_rotation * (current_angular_vel
					    / endpoint_rotation.t());
    }
    R6 endpos_vel(endpos_trans_vel,R3(endpos_rot_vel.t() * endpos_rot_vel.w()));
    OWD::JointPos joint_vel(tc.q.size());
    try {
      joint_vel = gfeplug->JacobianPseudoInverse_times_vector(endpos_vel);
    } catch (const char *err) {
      // no valid Jacobian, for whatever reason, so stop the trajectory
      // and leave the joint values unchanged
      ROS_WARN_NAMED("wstraj","JacobianPseudoInverse failed when calculating joint vel/accel");
      runstate = STOP;
      return;
    }

    /**********************************************
     *  calculate the nullspace joint correction  *
     **********************************************/
    // first, figure out where we are supposed to be
    OWD::JointPos target_jointpos = start_position + joint_change * dist;

    if (!AFTraj) {
      // if we are using ApplyForce at the same time, it will do its
      // own nullspace correction as long as we tell it the config we want
      OWD::JointPos jointpos_error = target_jointpos - tc.q;
      try {
	OWD::JointPos configuration_correction
	  = gfeplug->Nullspace_projection(jointpos_error);
	for (unsigned int j=0; j<configuration_correction.size(); ++j) {
	  if (isnan(configuration_correction[j])) {
	    configuration_correction[j]=0;
	  }
	}
	joint_correction += configuration_correction;
#ifdef SIMULATION
	ROS_DEBUG_NAMED("wstraj","Configuration corrected by %s",
			configuration_correction.sdump());
#endif
      } catch (const char *err) {
	// stop the trajectory if we can't correct the configuration
	ROS_WARN_NAMED("wstraj","Nullspace projection failed when correcting configuration");
	runstate = STOP;
	return;
      }
    }

    /**********************************************
     *  set our new values                        *
     **********************************************/
    if (AFTraj) {

      // modify the AFTraj target position and target configuration to
      // match our current values
      AFTraj->start_position = target_jointpos;
      AFTraj->endpoint_target = SE3(target_rotation,
				    target_position);

      // Let AFTraj compute the positions and torques needed to reach the
      // desired position while also maintaining the force
      AFTraj->evaluate(tc, dt);
  
    } else {

      // apply the pure position corrections
      for (unsigned int j=0; (j<joint_correction.size()) 
	     && (j<tc.q.size()); ++j) {
	tc.q[j] += joint_correction[j];
      }
    }


    // apply the joint vels and use them to calculate the accels, too.
    for (unsigned int j=0; (j<joint_vel.size()) 
	   && (j<tc.qd.size())
	   && (j<tc.qdd.size()); ++j) {
      tc.qd[j]=joint_vel[j];
      // since the ratio of lin_accel to lin_vel is the same as the ratio
      // of ang_accel to ang_vel, we can just scale all of the resulting
      // joint vels to get joint accels without having to push an accel
      // vector through the Jacobian Pseudo-Inverse again
      tc.qdd[j]=joint_vel[j] / current_linear_vel * current_linear_accel;
    }
  }
  
  // keep the end position tracking our current position (OWD will use this
  // to hold the point when we're done, and we have no guarrantee that we will
  // get to the requested config, so it's better to use the config we are in)
  end_position = tc.q;

  gfeplug->log_data(gfeplug->net_force.data);
  return;
}


bool WSTraj::Register() {
  ros::NodeHandle n("~");
  ss_AddWSTraj = n.advertiseService("AddWSTraj",&WSTraj::AddWSTraj);
  ss_ForceFeedbackNextTraj = n.advertiseService("ForceFeedbackNextWSTraj",
						&WSTraj::ForceFeedbackNextTrajSrv);
  return true;
}

void WSTraj::Shutdown() {
  ss_AddWSTraj.shutdown();
  ss_ForceFeedbackNextTraj.shutdown();
}

ros::ServiceServer WSTraj::ss_AddWSTraj, WSTraj::ss_ForceFeedbackNextTraj;

bool WSTraj::ForceFeedbackNextTraj(false);
