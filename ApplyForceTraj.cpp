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
    if (req.vibrate_amplitude_m > 0) {
      newtraj->SetVibration(req.vibrate_hand_x,
			     req.vibrate_hand_y,
			     req.vibrate_hand_z,
			     req.vibrate_amplitude_m,
			     req.vibrate_frequency_hz);
    }
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

  // reset the errors in our static force controller
  force_controller.reset();

  // always return true for the service call so that the client knows that
  // the call was processed, as opposed to there being a
  // network communication error.
  // the client will examine the "ok" field to see if the command was
  // actually successful
  return true;
}

ApplyForceTraj::ApplyForceTraj(R3 _force_direction, double force_magnitude,
			       double dist_limit):
  OWD::Trajectory("GFE Apply Force"),
  time_sum(0), 
  last_force_error(0), stopforce(false),
  distance_limit(dist_limit),
  last_travel(0),
  ft_filter(2,10.0),
  velocity_filter(2,10.0),
  vibration(NULL)
{
  if (gfeplug) {

    if ((gfeplug->ft_force.size() < 3) ||
	(gfeplug->ft_torque.size() < 3)) {
      throw "ApplyForce requires that the Force/Torque sensor is installed and configured";
    }

    /*  This check was removed so that we can call ApplyForce while we're
	already in contact with something.
    // make sure that the F/T sensor has already been tared
    const double max_tared_force=1.0;
    const double max_tared_torque=0.1;
    for (int i=0; i<3; ++i) {
      if ((gfeplug->ft_force[i] > max_tared_force) || 
	  (gfeplug->ft_torque[i] > max_tared_torque)) {
	throw "F/T sensor must be tared in the current configuration before calling ApplyForce";
      }
   }
    */

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

    gfeplug->net_force.data.resize(60);
    // values used for debugging during development
    // not all of these are filled in right now, but here's
    // the general idea:
    //
    // 0: time factor (1=realtime)
    // 1: endpoint motion factor (1=100% of limit)
    // 2: joint motion factor (1=100% of limit)
    // 3: max condition number
    // 4: force controller Ferr
    // 5: force controller dFerr
    // 6: force controller IFerr
    // 7-13: joint torques
    // 14-16: WS position X,Y,Z error
    // 17-19: WS rotation X,Y,Z error
    // 20-26: joint position correction
    // 27-33: incoming joint values
    // 34-40: PID torques from previous timestep
    // 41-47: Joint position changes since last timestep
    // 48-50: smoothed X,Y,Z force readings from force sensor
    // 51-53: raw X,Y,Z force readings
    // 54:  1-dim force controller correction force
    // 55:  1-dim travel in force direction from start point
    // 56:  excursion return force
    // 57:  1-dim velocity in force direction
    // 58:  velocity damping force
    // 59:  GfePlugin::recorder->count

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
  gfeplug->net_force.data[0 ]=dt/OWD::ControlLoop::PERIOD;
  
  if (stopforce) {
    runstate=OWD::Trajectory::DONE;
    return;
  }
  // record the joint positions
  gfeplug->net_force.data[27]=tc.q[0];
  gfeplug->net_force.data[28]=tc.q[1];
  gfeplug->net_force.data[29]=tc.q[2];
  gfeplug->net_force.data[30]=tc.q[3];
  gfeplug->net_force.data[31]=tc.q[4];
  gfeplug->net_force.data[32]=tc.q[5];
  gfeplug->net_force.data[33]=tc.q[6];

  // record the PID torques from the previous timestep
  gfeplug->net_force.data[34]=gfeplug->pid_torque[0];
  gfeplug->net_force.data[35]=gfeplug->pid_torque[1];
  gfeplug->net_force.data[36]=gfeplug->pid_torque[2];
  gfeplug->net_force.data[37]=gfeplug->pid_torque[3];
  gfeplug->net_force.data[38]=gfeplug->pid_torque[4];
  gfeplug->net_force.data[39]=gfeplug->pid_torque[5];
  gfeplug->net_force.data[40]=gfeplug->pid_torque[6];
  
  // record the force readings
  gfeplug->net_force.data[51]=gfeplug->ft_force[0];
  gfeplug->net_force.data[52]=gfeplug->ft_force[1];
  gfeplug->net_force.data[53]=gfeplug->ft_force[2];

  // calculate the endpoint position error
  R3 position_goal = (R3)endpoint_target;
  if (vibration) {
    position_goal += vibration->eval(dt);
  }

  R3 position_err = position_goal - gfeplug->endpoint;

  // calculate the component of the error in the direction of the
  // desired force and subtract it out
  double dot_prod = position_err * force_direction;

  // limit the amount of distance we can travel from the
  // starting position and the EE velocity by applying counter-torques
  // as necessary.  we'll pass in the amount of travel in the force direction
  // that's occurred, which is just the opposite of dot_prod.
  OWD::JointPos damping_torques = limit_excursion_and_velocity(-dot_prod);

  R3 position_correction = position_err - dot_prod * force_direction;
  gfeplug->net_force.data[14]=position_correction[0];
  gfeplug->net_force.data[15]=position_correction[1];
  gfeplug->net_force.data[16]=position_correction[2];

  // Compute the rotation error by taking the net rotation from the
  // current orientation to the original orientation and converting
  // it to axis-angle format
  SO3 rotation_error_SO3 = (SO3)endpoint_target * (! (SO3)gfeplug->endpoint);
  so3 rotation_error = (so3) rotation_error_SO3;

  // error in world frame
  R3 rotation_correction = rotation_error.t() * rotation_error.w();
  gfeplug->net_force.data[17]=rotation_correction[0];
  gfeplug->net_force.data[18]=rotation_correction[1];
  gfeplug->net_force.data[19]=rotation_correction[2];

#ifdef NDEF
  if (rotation_correction.norm() * 0.18 > .045) {
    // limit the correction this cycle.  0.18m is approximately the distance
    // from the link7 origin to the fingertips
    rotation_correction *= .045 / 0.18 / rotation_correction.norm();
  }
#endif

  // we now have position corrections to bring the endpoint back
  // into the desired configuration along the force direction.  Now
  // we'll use the ForceController to calculate the torques to give
  // us the right endpoint force.

  // get the current smoothed sensor force/torque in WS coordinates
  R6 current_force_torque = workspace_forcetorque();
  gfeplug->net_force.data[48]=current_force_torque.v[0];
  gfeplug->net_force.data[49]=current_force_torque.v[1];
  gfeplug->net_force.data[50]=current_force_torque.v[2];

  // take the dot product of the WS force with our force direction, so
  // that we just correct the on-axis forces.
  // we'll throw out the torques and set them to zero.
  R6 net_force_torque((current_force_torque.v * force_direction)
                      * force_direction, // project onto direction
                      R3()); // zero

  // overall force/torque error in WS coordinates
  R6 workspace_forcetorque_error =
    forcetorque_vector - net_force_torque;

  // Pass the F/T error to the force controller to get
  // the joint torques to apply
  OWD::JointPos correction_torques(tc.q.size());
  correction_torques = 
    force_controller.control(workspace_forcetorque_error);

  // log the 1-dimensional values from force controller
  gfeplug->net_force.data[4] = force_controller.bounded_ft_error.v 
    * force_direction;
  gfeplug->net_force.data[5] = force_controller.ft_error_delta.v
    * force_direction;
  gfeplug->net_force.data[6] = force_controller.ft_error_integral.v
    * force_direction / 3.0;
  gfeplug->net_force.data[54] = force_controller.ft_correction.v
    * force_direction;

  gfeplug->net_force.data[7] = correction_torques[0];
  gfeplug->net_force.data[8] = correction_torques[1];
  gfeplug->net_force.data[9] = correction_torques[2];
  gfeplug->net_force.data[10] = correction_torques[3];
  gfeplug->net_force.data[11] = correction_torques[4];
  gfeplug->net_force.data[12] = correction_torques[5];
  gfeplug->net_force.data[13] = correction_torques[6];


  // specify the feedforward torques that would ideally produce the
  // desired endpoint force.  this makes life easier for the force
  // controller.
  OWD::JointPos ideal_torques = 
    gfeplug->JacobianTranspose_times_vector(forcetorque_vector);

  // sum the correction and feedforward torques
  for (unsigned int i=0; i<tc.t.size(); ++i) {
    tc.t[i] = correction_torques[i]
      + ideal_torques[i]
      + damping_torques[i];
  }

  R6 endpos_correction(position_correction,rotation_correction);
  R3 desired_endpoint = (R3)gfeplug->endpoint + endpos_correction.v;
  endpositions.push(desired_endpoint);

  OWD::JointPos joint_correction;
  try {
    joint_correction = 
      gfeplug->JacobianPseudoInverse_times_vector(endpos_correction);
  } catch (const char *err) {
    // no valid Jacobian, for whatever reason, so give up.
    runstate=OWD::Trajectory::ABORT;
    return;
  }
  gfeplug->net_force.data[20]=joint_correction[0];
  gfeplug->net_force.data[21]=joint_correction[1];
  gfeplug->net_force.data[22]=joint_correction[2];
  gfeplug->net_force.data[23]=joint_correction[3];
  gfeplug->net_force.data[24]=joint_correction[4];
  gfeplug->net_force.data[25]=joint_correction[5];
  gfeplug->net_force.data[26]=joint_correction[6];

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

  // add the correction to the joint values
  for (unsigned int i=0; i<tc.q.size(); ++i) {
    tc.q[i] += isnan(joint_correction[i])? 0 : joint_correction[i];
  }
  OWD::JointPos joint_change(7);
  if (jointpositions.size() > 0) {
    joint_change = (tc.q - jointpositions.back());
  }
  gfeplug->net_force.data[41]=joint_change[0];
  gfeplug->net_force.data[42]=joint_change[1];
  gfeplug->net_force.data[43]=joint_change[2];
  gfeplug->net_force.data[44]=joint_change[3];
  gfeplug->net_force.data[45]=joint_change[4];
  gfeplug->net_force.data[46]=joint_change[5];
  gfeplug->net_force.data[47]=joint_change[6];

  jointpositions.push(tc.q);  // remember the requested positions
  // update the values to be published
  gfeplug->net_force.data[3]=OWD::Kinematics::max_condition;

  while (time_sum > time_window) {
    // update our averaging windows
    time_sum -= times.front();  times.pop();
    jointpositions.pop();
    endpositions.pop();
  }

  // record our data for logging
  gfeplug->log_data(gfeplug->net_force.data);
  gfeplug->net_force.data[59]=gfeplug->recorder->count;

  end_position =tc.q;  // keep tracking the current position
  return;
}

R6 ApplyForceTraj::workspace_forcetorque() {
  // get the FT force+torque and smooth it
  // I make this a little more efficient by always keeping track
  // of the sum of all the elements in the queue, so no matter how
  // many elements we are averaging we can quickly compute the new
  // average by just adjusting the sum and dividing by the count.

  R6 current_ft(gfeplug->ft_force[0],
		gfeplug->ft_force[1],
		gfeplug->ft_force[2],
		gfeplug->ft_torque[0],
		gfeplug->ft_torque[1],
		gfeplug->ft_torque[2]);

  /*
  static std::queue<R6> force_torque;
  static R6 force_torque_sum;
  force_torque.push(current_ft);
  force_torque_sum += current_ft;

  if (force_torque.size() > FT_window_size) {
    force_torque_sum -= force_torque.front();
    force_torque.pop();
  }
  R6 force_torque_avg = force_torque_sum / force_torque.size();
*/
  R6 force_torque_avg = ft_filter.eval(current_ft);
  
  // rotate the force and torque into workspace coordinates
  // we negate each of the sensor readings because what we want is a
  // measure of what the arm is producing, which is the opposite of
  // what the F/T sensor feels.
  R6 ws_force_torque((SO3)gfeplug->endpoint * (-1 * force_torque_avg.v),
                     (SO3)gfeplug->endpoint * (-1 * force_torque_avg.w));


  return ws_force_torque;
}


// Stop the trajectory when asked by a client
bool ApplyForceTraj::StopForce(gfe_owd_plugin::StopForce::Request &req,
			       gfe_owd_plugin::StopForce::Response &res) {
  stopforce=true;
  return true;
}

bool ApplyForceTraj::SetForceGains(pr_msgs::SetJointOffsets::Request &req,
                                   pr_msgs::SetJointOffsets::Response &res) {
  if (req.offset.size() != 6) {
    res.reason = "Need 6 gains: force kP, kD, kI, torque kP, kD, kI";
    res.ok=false;
  } else {
    force_controller.fkp=req.offset[0];
    force_controller.fkd=req.offset[1];
    force_controller.fki=req.offset[2];
    force_controller.tkp=req.offset[3];
    force_controller.tkd=req.offset[4];
    force_controller.tki=req.offset[5];
    res.ok=true;
  }
  return true;
}

OWD::JointPos ApplyForceTraj::limit_excursion_and_velocity(double travel) {
  // The travel argument is the distance we've moved in the force direction
  // from the original starting point.

  // We limit the excursion by calculating a restoring force proportional
  // to the cube of the distance beyond 90% of the travel limit.  By the
  // time we get to 100% of the travel limit, the restoring force will be
  // equal to the excursion_gain.
  double excursion_return_force = 0;
  static double excursion_gain = 10; // 10N of force if we reach the limit
  if (travel > 0.9 * distance_limit) {
    excursion_return_force = -pow((travel - 0.9*distance_limit) / (0.1*distance_limit),3)
      * excursion_gain;
  } else if (travel < -0.9 * distance_limit) {
    excursion_return_force = -pow((travel + 0.9*distance_limit) / (0.1*distance_limit),3)
      * excursion_gain;
  }
  gfeplug->net_force.data[55] = travel * 1000.0;
  gfeplug->net_force.data[56] = excursion_return_force;
    
  // We limit the velocity by creating a virtual dashpot that applies
  // a counter force proportional to the velocity.
  const double velocity_gain = 0.5;
  double travel_delta = travel - last_travel;
  last_travel = travel;
  double velocity = velocity_filter.eval(travel_delta) / OWD::ControlLoop::PERIOD;
  double velocity_return_force=0;
  gfeplug->net_force.data[1] = travel / cartesian_vel_limit;
  velocity_return_force = 
    - pow(velocity / cartesian_vel_limit, 3)
    * velocity_gain;
  gfeplug->net_force.data[57] = velocity * 1000.0;
  gfeplug->net_force.data[58] = velocity_return_force;

  // compute the corresponding joint torques
  R3 correction_torque;
  R6 correction_ft 
    = R6((excursion_return_force+velocity_return_force)* force_direction,
	 correction_torque);

  return OWD::Plugin::JacobianTranspose_times_vector(correction_ft);
}

void ApplyForceTraj::SetVibration(double hand_x, double hand_y, double hand_z,
				  double amplitude, double frequency) {
  vibration = new Vibration(R3(hand_x, hand_y, hand_z),
			    amplitude,
			    frequency);
}


double ApplyForceTraj::force_gain_kp = 1.6e-3;

double ApplyForceTraj::force_gain_kd = 0;

double ApplyForceTraj::xforce = 1.0;

double ApplyForceTraj::cartesian_vel_limit=0.1;

ForceController ApplyForceTraj::force_controller;

ros::ServiceServer ApplyForceTraj::ss_ApplyForce,
  ApplyForceTraj::ss_SetForceGains;
  

// Set up our ROS service for receiving trajectory requests.
bool ApplyForceTraj::Register() {
  ros::NodeHandle n("~");
  ss_ApplyForce = n.advertiseService("ApplyForce",&ApplyForceTraj::ApplyForce);
  ss_SetForceGains = n.advertiseService("SetForceGains",&ApplyForceTraj::SetForceGains);

  return true;
}

// Shut down our service so that it's no longer listed by the ROS master
void ApplyForceTraj::Shutdown() {
  ss_ApplyForce.shutdown();
  ss_SetForceGains.shutdown();
}

ApplyForceTraj::~ApplyForceTraj() {
  gfeplug->flush_recorder_data = true;
  ss_StopForce.shutdown();
  if (vibration) {
    delete vibration;
  }
}

