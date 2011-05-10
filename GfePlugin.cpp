/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "GfePlugin.hh"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include "openwam/Kinematics.hh"

GfePlugin::GfePlugin() {
  // ROS has already been initialized by OWD, so we can just
  // create our own NodeHandle in the same namespace
  ros::NodeHandle n("~");

  // Let our Trajectory class register itself
  if (!ApplyForceTraj::Register()) {
    throw "ApplyForceTraj trajectory failed to register";
  }

#ifdef SIMULATION
  // create our Jacobian publisher
  pub_jacobian = n.advertise<gfe_owd_plugin::Jacobian>("jacobian",40);
  jacobian_msg.jacobianEE_row1.resize(7);
  jacobian_msg.jacobianEE_row2.resize(7);
  jacobian_msg.jacobianEE_row3.resize(7);
  jacobian_msg.jacobian0_row1.resize(7);
  jacobian_msg.jacobian0_row2.resize(7);
  jacobian_msg.jacobian0_row3.resize(7);
  jacobian_msg.endpoint_row1.resize(4);
  jacobian_msg.endpoint_row2.resize(4);
  jacobian_msg.endpoint_row3.resize(4);
  jacobian_msg.rotation_row1.resize(4);
  jacobian_msg.rotation_row2.resize(4);
  jacobian_msg.rotation_row3.resize(4);
#endif // SIMULATION
}

GfePlugin::~GfePlugin() {
  // Also tell our Trajectory class to clean up.
  ApplyForceTraj::Shutdown();
}

void GfePlugin::Publish() {
#ifdef SIMULATION
  for (int i=0; i<7; ++i) {
    jacobian_msg.jacobianEE_row1[i] = OWD::Kinematics::JacobianEE[i][0];
    jacobian_msg.jacobianEE_row2[i] = OWD::Kinematics::JacobianEE[i][1];
    jacobian_msg.jacobianEE_row3[i] = OWD::Kinematics::JacobianEE[i][2];

    jacobian_msg.jacobian0_row1[i] = OWD::Kinematics::Jacobian0[i][0];
    jacobian_msg.jacobian0_row2[i] = OWD::Kinematics::Jacobian0[i][1];
    jacobian_msg.jacobian0_row3[i] = OWD::Kinematics::Jacobian0[i][2];
  }
  for (int i=0; i<4; ++i) {
    jacobian_msg.endpoint_row1[i] = endpoint[i];
    jacobian_msg.endpoint_row2[i] = endpoint[4+i];
    jacobian_msg.endpoint_row3[i] = endpoint[8+i];

    jacobian_msg.rotation_row1[i] = U_matrix[i];
    jacobian_msg.rotation_row2[i] = U_matrix[4+i];
    jacobian_msg.rotation_row3[i] = U_matrix[8+i];
  }
  pub_jacobian.publish(jacobian_msg);
#endif // SIMULATION
}


// Allocation for the pointer that will hold the single instantiation
// of our plugin class.  We initialize it to NULL so that the register
// function can tell whether or not one has already been allocated.
GfePlugin *gfeplug = NULL;

// Process our ApplyForce service calls from ROS
bool ApplyForceTraj::ApplyForce(gfe_owd_plugin::ApplyForce::Request &req,
				gfe_owd_plugin::ApplyForce::Response &res) {
  ROS_INFO("GfePlugin: received ApplyForce service call");

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
  // the call was actually processed, as opposed to there being a
  // network communication error.
  // the client will examine the "ok" field to see if the command was
  // actually successful
  return true;
}

ApplyForceTraj::ApplyForceTraj(R3 _force_direction, double force_magnitude): OWD::Trajectory("GFE Apply Force"), stopforce(false) {
  if (gfeplug) {

    // Set the start position from the current position
    start_position=gfeplug->target_arm_position;
    end_position = start_position;

    // Save the current Cartesian endpoint position
    endpoint_target = gfeplug->endpoint;

    // Calculate the force/torque to apply at endpoint
    _force_direction.normalize();
    force_direction = _force_direction; // save for evaluate function

    R3 torques;  // initializes to zero
    _force_direction *= force_magnitude;
    forcetorque_vector = R6(_force_direction,torques);

#ifdef SIMULATION
    joints.resize(start_position.size());
    sub_setjoints = n.subscribe("setjoints",1,&ApplyForceTraj::setjoints_callback,this);
#endif // SIMULATION

  } else {
    throw "Could not get current WAM values from GfePlugin";
  }

  // create the service that the client can use to stop the force
  ros::NodeHandle n("~");
  ss_StopForce = n.advertiseService("StopForce",&ApplyForceTraj::StopForce, this);

}

void ApplyForceTraj::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
  time += dt;

  if (stopforce) {
    runstate=OWD::Trajectory::DONE;
    return;
  }

#ifdef SIMULATION
  // allow joint values to be set directly by ROS message
  for (unsigned int i=0; i<tc.q.size(); ++i) {
    tc.q[i]=joints[i];
  }

  // perturb the joints as if they drifted since last time
  for (unsigned int i=0; i<tc.q.size(); ++i) {
    double rand_joint_tweak=random();
    rand_joint_tweak = (rand_joint_tweak / RAND_MAX - 0.5) / 100.0;
    tc.q[i] += rand_joint_tweak;
  }
#endif // SIMULATION

  // calculate the endpoint position error
  R3 position_err = (R3)endpoint_target - (R3)gfeplug->endpoint;

#ifdef SIMULATION
  ROS_DEBUG_NAMED("applyforce","Workspace goal [%1.3f %1.3f %1.3f]",
		  ((R3)endpoint_target)[0],
		  ((R3)endpoint_target)[1],
		  ((R3)endpoint_target)[2]);
  ROS_DEBUG_NAMED("applyforce","Workspace position [%1.3f %1.3f %1.3f]",
		  ((R3)gfeplug->endpoint)[0],
		  ((R3)gfeplug->endpoint)[1],
		  ((R3)gfeplug->endpoint)[2]);
  ROS_DEBUG_NAMED("applyforce","Original position error [%1.2f %1.2f %1.2f]",
		  position_err[0] * 1000.0,
		  position_err[1] * 1000.0,
		  position_err[2] * 1000.0);
#endif // SIMULATION

  // calculate the component of the error in the direction of the
  // desired force and subtract it out
  double dot_prod = position_err * force_direction;
  position_err = position_err - dot_prod * force_direction;

#ifdef SIMULATION
  ROS_DEBUG_NAMED("applyforce","Remapped position error [%1.2f %1.2f %1.2f]",
		  position_err[0] * 1000.0,
		  position_err[1] * 1000.0,
		  position_err[2] * 1000.0);
#endif // SIMULATION

  // for now, ignore any rotation error
  R3 rotation_err;  // initializes to zero

  // combine position and rotation into a single vector
  R6 endpoint_error(position_err,rotation_err);

  // use the Jacobian Transpose to figure out what direction
  // the joints should move in to correct the position error
  JointPos joint_correction = 
    gfeplug->JacobianTranspose_times_vector(endpoint_error);

  // figure out how much to scale our correction by
  R6 workspace_movement = 
    gfeplug->Jacobian_times_vector(joint_correction);
  R3 workspace_position_movement = workspace_movement.v();
  // move 90% of the calculated ratio
  double scale = 0.90 * position_err.norm() 
    / workspace_position_movement.norm();
  joint_correction *= scale;

#ifdef SIMULATION
  // print our progress
  workspace_movement = 
    gfeplug->Jacobian_times_vector(joint_correction);
  workspace_position_movement = workspace_movement.v();
  R3 new_error = position_err - workspace_position_movement;
  ROS_DEBUG_NAMED("applyforce","Workspace error was %1.3fmm [%1.2f %1.2f %1.2f], now %1.3fmm [%1.2f %1.2f %1.2f]", position_err.norm() * 1000.0,
		  position_err[0] * 1000.0,
		  position_err[1] * 1000.0,
		  position_err[2] * 1000.0,
		  new_error.norm() * 1000.0,
		  new_error[0] * 1000.0,
		  new_error[1] * 1000.0,
		  new_error[2] * 1000.0);
  if (new_error.norm() > 
      position_err.norm()) {
    ROS_ERROR_NAMED("applyforce","WARNING: error grew");
  }
  ROS_DEBUG_NAMED("applyforce","Correcting joints by [%2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f]",
		  joint_correction[0],joint_correction[1],joint_correction[2],joint_correction[3],joint_correction[4],joint_correction[5],joint_correction[6]);
#endif // SIMULATION

  // add the correction to the joint values
  for (unsigned int i=0; i<tc.q.size(); ++i) {
    tc.q[i] += isnan(joint_correction[i])? 0 : joint_correction[i];
  }

  // calculate the additional torques for the endpoint force in the
  // specified direction
  JointPos joint_torques = 
    gfeplug->JacobianTranspose_times_vector(forcetorque_vector);

  // figure out how much to scale our correction by
  R6 workspace_forcetorques =
    gfeplug->Jacobian_times_vector(joint_torques);
  scale = forcetorque_vector.v().norm() 
    / workspace_forcetorques.v().norm();
  // apply the scale factor
  joint_torques *= scale;

#ifdef SIMULATION
  ROS_DEBUG_NAMED("applyforce","Adding torque of [%2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f]",
		  joint_torques[0],joint_torques[1],joint_torques[2],joint_torques[3],joint_torques[4],joint_torques[5],joint_torques[6]);
#endif // SIMULATION

  // set the joint torques
  for (unsigned int i=0; i<tc.t.size(); ++i) {
    tc.t[i] = isnan(joint_torques[i]) ? 0 : joint_torques[i];
  }

#ifdef SIMULATION
  // publish our values for debugging
  afdebug_msg.forcetorque_in.resize(6);
  afdebug_msg.torques_out.resize(7);
  afdebug_msg.forcetorque_out.resize(6);
  for (int i=0; i<6; ++i) {
    afdebug_msg.forcetorque_in[i]=forcetorque_vector[i];
  }
  for (int i=0; i<7; ++i) {
    afdebug_msg.torques_out[i]=joint_torques[i];
  }
  for (int i=0; i<6; ++i) {
    afdebug_msg.forcetorque_out[i]=workspace_forcetorques[i];
  }
  afdebug_msg.scale = scale;
  pub_AFDebug.publish(afdebug_msg);
#endif // SIMULATION

  end_position =tc.q;  // keep tracking the current position
  return;
}

// Stop the trajectory when asked by a client
bool ApplyForceTraj::StopForce(gfe_owd_plugin::StopForce::Request &req,
			       gfe_owd_plugin::StopForce::Response &res) {
  stopforce=true;
  return true;
}

#ifdef SIMULATION
void ApplyForceTraj::setjoints_callback(const boost::shared_ptr<const pr_msgs::Joints> &newjoints) {
  joints = newjoints->j;
}

ros::Publisher ApplyForceTraj::pub_AFDebug;

#endif // SIMULATION

// Set up our ROS service for receiving trajectory requests.
bool ApplyForceTraj::Register() {
  ros::NodeHandle n("~");
  ss_ApplyForce = n.advertiseService("ApplyForce",&ApplyForceTraj::ApplyForce);

#ifdef SIMULATION
  pub_AFDebug = n.advertise<gfe_owd_plugin::ApplyForceDebug>("afdebug",100);
#endif // SIMULATION

  return true;
}

// Shut down our service so that it's no longer listed by the ROS master
void ApplyForceTraj::Shutdown() {
  ss_ApplyForce.shutdown();
}

ApplyForceTraj::~ApplyForceTraj() {
  ss_StopForce.shutdown();

#ifdef SIMULATION
  sub_setjoints.shutdown();
#endif // SIMULATION
}

// Have to allocate storage for our ROS service handlers
ros::ServiceServer ApplyForceTraj::ss_ApplyForce;

// The register_owd_plugin() is the only function that OWD will call when
// the plugin is loaded.  It has to initialize our custom Plugin and 
// Trajectory classes.
bool register_owd_plugin() {
  if (gfeplug) {
    delete gfeplug; // free the previous one in case register was called twice
  }
  try {
    // create an instantiation of our custom Plugin class
    gfeplug = new GfePlugin();
  } catch (...) {
    gfeplug=NULL;
    return false;
  }
  return true;
}

void unregister_owd_plugin() {
  if (gfeplug) {
    // remove our Plugin class, which will let it shut down any ROS
    // communications it created
    delete gfeplug;
    gfeplug=NULL;
  }
  return;
}
