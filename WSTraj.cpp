#include "WSTraj.h"
#include <math.h>
#include <LinearMath/btQuaternion.h>

bool WSTraj::AddWSTraj(gfe_owd_plugin::AddWSTraj::Request &req,
		       gfe_owd_plugin::AddWSTraj::Response &res) {
  // compute a new workspace trajectory
  try {
    WSTraj *newtraj = new WSTraj(req);

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

WSTraj::WSTraj(gfe_owd_plugin::AddWSTraj::Request &wst) 
  : OWD::Trajectory("WS Traj"),
    driving_force(wst.wrench.force.x,
		  wst.wrench.force.y,
		  wst.wrench.force.z,
		  wst.wrench.torque.x,
		  wst.wrench.torque.y,
		  wst.wrench.torque.z),
    start_endpoint(gfeplug->endpoint),
    endpoint_translation(wst.endpoint_change.position.x,
			 wst.endpoint_change.position.y,
			 wst.endpoint_change.position.z),
    max_linear_vel(wst.max_linear_velocity),
    max_linear_accel(max_linear_vel/wst.min_accel_time),
    max_angular_vel(wst.max_angular_velocity),
    max_angular_accel(max_angular_vel/wst.min_accel_time),
    parseg(0,0,0,1)
{
  start_position = OWD::JointPos(wst.starting_config);
  end_position = OWD::JointPos(wst.ending_config);
  joint_change = OWD::JointPos(wst.ending_config) - start_position;

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
}

WSTraj::~WSTraj() {
}

void WSTraj::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
 
  if (driving_force.norm() > 0) {

    // increment our time by dt. 

    // map our current position to the closest point
    // on the trajectory.  this does not move us forward or
    // backward in time.

    // calculate our endpoint correction

    // calculate the nullspace joint correction

    // look up the "feedforward" amount of force we should be
    // applying based on the trajectory position.  this will give
    // us a soft start at the beginning and a soft stop at the end

    // correct our velocity by computing a "feedback" force term.
    // this will reduce the overall force if the velocity is too high,
    // but will never increase the force over the force limit.

    // our position in the reference trajectory at the current time
    // is the farthest we should be at this point.  it's ok if we aren't
    // as far along yet, since we're letting the force working against
    // the environment dictate how fast we move

    // return the new joint positions and the feedforward force

  } else {
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

    so3 relative_rotation(endpoint_rotation.w(), endpoint_rotation.t() * dist);
    SO3 target_rotation = (SO3)relative_rotation * (SO3)start_endpoint;
    // calculate the endpoint position correction
    R3 position_correction = target_position - (R3)gfeplug->endpoint;

    // Compute the endpoint rotation error by taking the net rotation from
    // the current orientation to the target orientation and converting
    // it to axis-angle format
    
    so3 rotation_error = (so3)(target_rotation * (! (SO3)gfeplug->endpoint));

    // represent as correction in world frame
    R3 rotation_correction = rotation_error.t() * rotation_error.w();
    
    // calculate the joint change required to correct the endpoint pose
    R6 endpos_correction(position_correction,rotation_correction);
    OWD::JointPos joint_correction;
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
    double current_angular_accel=accel * vel_factor * max_angular_accel;
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
    OWD::JointPos joint_vel;
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

    /**********************************************
     *  set our new values                        *
     **********************************************/
    // apply the position corrections
    for (unsigned int j=0; (j<joint_correction.size()) 
	   && (j<tc.q.size()); ++j) {
      tc.q[j] += joint_correction[j];
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
    
  return;
}


bool WSTraj::Register() {
  ros::NodeHandle n("~");
  ss_AddWSTraj = n.advertiseService("AddWSTraj",&WSTraj::AddWSTraj);
  return true;
}

void WSTraj::Shutdown() {
  ss_AddWSTraj.shutdown();
}

ros::ServiceServer WSTraj::ss_AddWSTraj;
