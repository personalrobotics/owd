/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "ApplyForceTraj.h"
#include <openwam/Kinematics.hh>
#include <openwam/ControlLoop.hh>

// Process our ApplyForce service calls from ROS
bool ApplyForceTraj::ApplyForce(gfe_owd_plugin::ApplyForce::Request &req,
				gfe_owd_plugin::ApplyForce::Response &res) {
  // compute a new trajectory
  try {
    ApplyForceTraj *newtraj = new ApplyForceTraj(R3(req.x,req.y,req.z),req.f);
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

ApplyForceTraj::ApplyForceTraj(R3 _force_direction, double force_magnitude):
  OWD::Trajectory("GFE Apply Force"),
  correction_motion_sum(0),
  time_sum(0), 
  last_force_error(0), stopforce(false)
{
  if (gfeplug) {

    // make sure that the F/T sensor has already been tared
    const double max_tared_force=1.0;
    const double max_tared_torque=0.1;
    if ((gfeplug->ft_force.size() < 3) ||
	(gfeplug->ft_torque.size() < 3)) {
      throw "ApplyForce requires that the Force/Torque sensor is installed and configured";
    }
    for (int i=0; i<3; ++i) {
      if ((gfeplug->ft_force[i] > max_tared_force) || 
	  (gfeplug->ft_torque[i] > max_tared_torque)) {
	throw "F/T sensor must be tared in the current configuration before calling ApplyForce";
      }
    }

    if (OWD::Kinematics::max_condition > 15) {
      throw "Arm is too close to a singularity for accurate force control; please move it to a different configuration and try again";
    }

    // Set the start position from the current position
    start_position=gfeplug->target_arm_position;
    end_position = start_position;

    // Save the current Workspace endpoint position
    endpoint_target = gfeplug->endpoint;

    // Calculate the force/torque to apply at endpoint
    _force_direction.normalize();
    force_direction = _force_direction; // save for evaluate function
    _force_direction *= force_magnitude;
    R3 torques;  // initializes to zero
    forcetorque_vector = R6(_force_direction,torques);

    gfeplug->net_force.data.resize(11);
    // values used for debugging during development
    // not all of these are filled in right now, but here's
    // the general idea:
    //
    // 0: time factor (1=realtime)
    // 1: endpoint motion factor (1=100% of limit)
    // 2: joint motion factor (1=100% of limit)
    // 3: max condition number
    // 4: force error
    // 5: position/rotation correction
    // 6: nullspace correction
    // 7: force correction
    // 8: force_x_avg
    // 9: force_y_avg
    // 10: force_z_avg

  } else {
    throw "Could not get current WAM values from GfePlugin";
  }

  // create the service that the client can use to stop the force
  ros::NodeHandle n("~");
  ss_StopForce = n.advertiseService("StopForce",&ApplyForceTraj::StopForce, this);

}

void ApplyForceTraj::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
  time += dt;
  times.push(dt);  time_sum += dt;
  
  if (stopforce) {
    runstate=OWD::Trajectory::DONE;
    return;
  }

  // calculate the endpoint position error
  R3 position_err = (R3)endpoint_target - (R3)gfeplug->endpoint;

  // calculate the component of the error in the direction of the
  // desired force and subtract it out
  double dot_prod = position_err * force_direction;
  R3 position_correction = position_err - dot_prod * force_direction;

  if (position_correction.norm() > .015) {
    // limit the step size each cycle
    position_correction *= .015 / position_correction.norm();
  }

  // keep track of the total movement in the force direction so that
  // we can actively brake if it exceeds our limit
  //  axial_movement += dot_prod;
  //  axial_movements.push_back(dot_prod);

  // Compute the rotation error by taking the net rotation from the
  // current orientation to the original orientation and converting
  // it to axis-angle format
  SO3 rotation_error_SO3 = (SO3)endpoint_target * (! (SO3)gfeplug->endpoint);
  so3 rotation_error = (so3) rotation_error_SO3;

  // error in world frame
  R3 rotation_correction = rotation_error.t() * rotation_error.w();

  if (rotation_correction.norm() * 0.18 > .045) {
    // limit the correction this cycle.  0.18m is approximately the distance
    // from the link7 origin to the fingertips
    rotation_correction *= .045 / 0.18 / rotation_correction.norm();
  }

  // ok, we've corrected the position error that's normal to the
  // force vector.  now we need to decide how much we need to adjust
  // the position along our force vector in order to maintain the
  // desired force at the f/t sensor

  // take the dot product of the WS force with our force direction
  double net_force = workspace_force() * force_direction;

  // calculate the error
  double force_target = forcetorque_vector.v().norm();
  double force_error = force_target -  net_force;
  double force_limit = xforce * force_target;
  gfeplug->net_force.data[4]=force_error;
  if (fabs(force_error) > force_limit) {
    // threshold our force error
    if (force_error > 0) {
      force_error = force_limit;
    } else {
      force_error = -force_limit;
    }
  }
  double force_error_delta = force_error - last_force_error;
  last_force_error = force_error;

  // multiply it times our gain
  double correction_distance = force_error * force_gain_kp
    + force_error_delta * force_gain_kd;

  // limit the amount of motion due to the force error
  correction_distance = limit_force_correction_movement(correction_distance);

  // multiply the distance times the force direction vector and
  // add the resulting vector to our position error
  position_correction += correction_distance * force_direction;

  R6 endpos_correction(position_correction,rotation_correction);
  OWD::JointPos joint_correction;
  try {
    joint_correction = 
      gfeplug->JacobianPseudoInverse_times_vector(endpos_correction);
  } catch (const char *err) {
    // no valid Jacobian, for whatever reason, so just leave the joint
    // values unchanged.  Maybe someone will move us back to a more
    // favorable configuration
    return;
  }

  // calculate a move in the nullspace that keeps the arm as close to its
  // original configuration as possible
  OWD::JointPos configuration_error = start_position - tc.q;
  try {
    OWD::JointPos configuration_correction
      = gfeplug->Nullspace_projection(configuration_error);
    joint_correction += 0.5 * configuration_correction;
  } catch (const char *err) {
    // don't worry about it
  }

  // Check the requested motion of each joint over the past 0.25 seconds to 
  // make sure it doesn't exceed 90deg/sec (0.3927 rad in 0.25 sec)

  if (time_sum > time_window) {
    OWD::JointPos joint_motion = tc.q - jointpositions.front()
      + joint_correction;
    gfeplug->net_force.data[2]=joint_motion.length() / joint_vel_limit / time_sum;
  /*  joint speed limits weren't working too well the last time I tried,
      so I've commented out this section until I can test it further

    for (unsigned int i=0; i<joint_motion.size(); ++i) {
      if (fabs(joint_motion[i]) > joint_vel_limit * time_sum) {
	if ((fabs(joint_motion[i]-joint_correction[i])
	    > joint_vel_limit * time_sum)
	    && (fabs(joint_motion[i]) > fabs(joint_motion[i]-joint_correction[i]))) {
	  // we were exceeding the joint motion limits even without the
	  // correction, and the correction is only making it worse, so
	  // don't do any correction this round
      	  joint_motion=tc.q - jointpositions.front();
	  break;
	} else {
	  // the correction at least helps a little, so
	  // leave it unchanged
	}
      } else {
	// bring this joint's correction back to the limit.
	// yes, this will change the direction of the overall joint
	// correction vector, but it will bring us as close as we can
	// get to the desired target.
	joint_motion[i] *= joint_vel_limit * time_sum / fabs(joint_motion[i]);
      }
    }
    // recalculate our joint correction
    joint_correction = jointpositions.front()+ joint_motion - tc.q;
  */
  }

  // Normally the evaluate function will be called with dt equal to
  // the control period.  But if OWD is detecting stall conditions due to
  // too-high PID torques, it will reduce the timescale and we'll see
  // smaller values for dt, meaning we should take a smaller "step" along
  // the trajectory path.  We'll achieve this by scaling down our 
  // ultimate joint correction.
  joint_correction *= dt / OWD::ControlLoop::PERIOD;
  gfeplug->net_force.data[0]=dt/OWD::ControlLoop::PERIOD;
  
  // add the correction to the joint values
  for (unsigned int i=0; i<tc.q.size(); ++i) {
    tc.q[i] += isnan(joint_correction[i])? 0 : joint_correction[i];
  }
  jointpositions.push(tc.q);  // remember the requested positions

  // specify the feedforward torques that would ideally produce the
  // desired endpoint force.  this makes life easier for the force
  // controller.
  OWD::JointPos joint_torques = 
    gfeplug->JacobianTranspose_times_vector(forcetorque_vector);
  for (unsigned int i=0; i<tc.t.size(); ++i) {
    tc.t[i] = isnan(joint_torques[i])? 0 : joint_torques[i];
  }

  // update the values to be published
  gfeplug->net_force.data[3]=OWD::Kinematics::max_condition;

  while (time_sum > time_window) {
    // update our averaging windows
    time_sum -= times.front();  times.pop();
    correction_motion_sum -= correction_distances.front();
    correction_distances.pop();
    jointpositions.pop();
  }

  end_position =tc.q;  // keep tracking the current position
  return;
}

R3 ApplyForceTraj::workspace_force() {
  // get the FT force and smooth it
  // I make this a little more efficient by always keeping track
  // of the sum of all the elements in the queue, so no matter how
  // many elements we are averaging we can quickly compute the new
  // average by just adjusting the sum and dividing by the count.
  static std::queue<double> force_x, force_y, force_z;
  static double force_x_sum=0;
  static double force_y_sum=0;
  static double force_z_sum=0;
  force_x.push(gfeplug->ft_force[0]);
  force_y.push(gfeplug->ft_force[1]);
  force_z.push(gfeplug->ft_force[2]);
  force_x_sum += gfeplug->ft_force[0];
  force_y_sum += gfeplug->ft_force[1];
  force_z_sum += gfeplug->ft_force[2];
  if (force_x.size() > FT_window_size) {
    force_x_sum -= force_x.front();
    force_y_sum -= force_y.front();
    force_z_sum -= force_z.front();
    force_x.pop();
    force_y.pop();
    force_z.pop();
  }
  double force_x_avg = force_x_sum / force_x.size();
  double force_y_avg = force_y_sum / force_y.size();
  double force_z_avg = force_z_sum / force_z.size();
  
  // we want to make sure we're applying forcetorque_vector of force
  // in workspace coordinates, so first calculate our actual force
  // in that direction using the values from the f/t sensor.
  // we negate each of the sensor readings because what we want is a
  // measure of what the arm is producing, which is the opposite of
  // what the F/T sensor feels.
  R3 handframe_force(-force_x_avg,
		     -force_y_avg,
		     -force_z_avg);
  // rotate into workspace coordinates
  R3 ws_force = (SO3) gfeplug->endpoint * handframe_force;
  gfeplug->net_force.data[8]=ws_force[0];
  gfeplug->net_force.data[9]=ws_force[1];
  gfeplug->net_force.data[10]=ws_force[2];

  return ws_force;
}

double ApplyForceTraj::limit_force_correction_movement(double correction_distance) {
  // Check the total force-based position correction we have made
  // in the past 0.25 seconds to make sure it doesn't exceed the
  // cartesian velocity limit

#ifdef WINDOW
  if (time_sum > time_window) {
    double correction_motion = correction_motion_sum + correction_distance;
    if (fabs(correction_motion) > (cartesian_vel_limit * time_sum)) {
      // The full correction will exceed our overall velocity limit.
      if (fabs(correction_motion_sum) > (cartesian_vel_limit * time_sum)) {
	if (fabs(correction_motion) > fabs(correction_motion_sum)) {
	  // we were already exceeding the limit for this timestep, and
	  // our correction only makes it worse, so don't do any correction
	  // this time around
	  correction_distance = 0;
	} else {
	  // the correction seems to be helping, so we will just stick
	  // with the full correction
	}
      } else {
	// reduce the correction to be within the velocity limit
	if (correction_motion_sum > 0) {
	  correction_distance = cartesian_vel_limit * time_sum - 
	    correction_motion_sum;
	} else {
	  correction_distance = - cartesian_vel_limit * time_sum - 
	    correction_motion_sum;
	}
      }
    }
  }

  correction_motion_sum += correction_distance;

  // publish for debugging
  gfeplug->net_force.data[1] 
    =correction_motion_sum / cartesian_vel_limit / time_sum;

#else

#ifdef NONE
  gfeplug->net_force.data[1] 
    =correction_distance / cartesian_vel_limit / time_sum;
  if (fabs(correction_distance) > (cartesian_vel_limit*.002)) {
    if (correction_distance > 0) {
      correction_distance = cartesian_vel_limit*.002;
    } else {
      correction_distance = - cartesian_vel_limit*.002;
    }
  }
#endif // NONE

#endif // WINDOW

  correction_distances.push(correction_distance);
  return correction_distance;
}


// Stop the trajectory when asked by a client
bool ApplyForceTraj::StopForce(gfe_owd_plugin::StopForce::Request &req,
			       gfe_owd_plugin::StopForce::Response &res) {
  stopforce=true;
  return true;
}

bool ApplyForceTraj::SetForcePGain(pr_msgs::SetStiffness::Request &req,
				  pr_msgs::SetStiffness::Response &res) {
  if (req.stiffness > .008) {
    res.reason = "Proportional gain appears too high (above .008, which was unstable).  If you really want this gain you have to change the code).";
    res.ok=false;
  } else {
    force_gain_kp = req.stiffness;
    res.ok=true;
  }
  return true;
}

bool ApplyForceTraj::SetForceDGain(pr_msgs::SetStiffness::Request &req,
				  pr_msgs::SetStiffness::Response &res) {
  if (req.stiffness > .001) {
    res.reason = "Derivative gain appears too high (above .001, which was unstable).  If you really want this gain you have to change the code).";
    res.ok=false;
  } else {
    force_gain_kd = req.stiffness;
    res.ok=true;
  }
  return true;
}

bool ApplyForceTraj::SetForceFactor(pr_msgs::SetStiffness::Request &req,
				    pr_msgs::SetStiffness::Response &res) {
  if (req.stiffness > 10) {
    res.reason = "10 is too high.  If you really want this gain you have to change the code).";
    res.ok=false;
  } else {
    xforce = req.stiffness;
    res.ok=true;
  }
  return true;
}

double ApplyForceTraj::force_gain_kp = 1.6e-3;

double ApplyForceTraj::force_gain_kd = 0;

double ApplyForceTraj::xforce = 1.0;

double ApplyForceTraj::cartesian_vel_limit=0.5;

ros::ServiceServer ApplyForceTraj::ss_ApplyForce,
  ApplyForceTraj::ss_SetForcePGain, 
  ApplyForceTraj::ss_SetForceDGain,
  ApplyForceTraj::ss_SetForceFactor;
  

// Set up our ROS service for receiving trajectory requests.
bool ApplyForceTraj::Register() {
  ros::NodeHandle n("~");
  ss_ApplyForce = n.advertiseService("ApplyForce",&ApplyForceTraj::ApplyForce);
  ss_SetForcePGain = n.advertiseService("SetForcePGain",&ApplyForceTraj::SetForcePGain);
  ss_SetForceDGain = n.advertiseService("SetForceDGain",&ApplyForceTraj::SetForceDGain);
  ss_SetForceFactor = n.advertiseService("SetForceFactor",&ApplyForceTraj::SetForceFactor);

  return true;
}

// Shut down our service so that it's no longer listed by the ROS master
void ApplyForceTraj::Shutdown() {
  ss_ApplyForce.shutdown();
  ss_SetForcePGain.shutdown();
  ss_SetForceDGain.shutdown();
  ss_SetForceFactor.shutdown();
}

ApplyForceTraj::~ApplyForceTraj() {
  ss_StopForce.shutdown();
}

