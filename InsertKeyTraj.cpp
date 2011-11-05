#include "InsertKeyTraj.h"

bool InsertKeyTraj::InsertKey(gfe_owd_plugin::InsertKey::Request &req,
			      gfe_owd_plugin::InsertKey::Response &res) {
  // compute a new trajectory
  try {
    InsertKeyTraj *newtraj = new InsertKeyTraj();

    // send it to the arm
    res.id = OWD::Plugin::AddTrajectory(newtraj,res.reason);
    if (res.id > 0) {
      current_traj = newtraj;
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

InsertKeyTraj::InsertKeyTraj() :
  ApplyForceTraj((SO3)gfeplug->endpoint * R3(0,0,1), 0.8, 0.02),
  current_step(NULL)
{
  current_step = new InsertKeyStep8();
  gfeplug->current_traj=this;
}

void InsertKeyTraj::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
  /* Procedure:
     0.  Tare the F/T sensor
     1.  Approach the face of the handle to the side of the cylinder and stop.
     2.  Move towards the cylinder and stop when we drop in.
     3.  Start executing an upwards CCW arc , always pressing radially
         outwards and stopping when we move horizontally left (at 12 o'clock).
     4.  Start a CW arc, stop when we move horizontally left (at 6 o'clock).
     5.  Estimate cylinder location and move directly to keyhole.
     6.  Verify we're in slot by trying to move gently up.
     7.  Verify we're in slot by trying to move gently down.
     8.  Push while keeping zero torque about Y axis (to keep key straight)
   */
  if (!current_step) {
    runstate=ABORT;
    return;
  }
  switch (current_step->stepname) {
  case STEP1_FIND_SURFACE:
    // run a regular forward trajectory with stop on force set to true.
    // stop when the trajectory aborts.
	break;
  case STEP2_FIND_CYLINDER:
    // run a sliding trajectory and stop when we see the z impulse or
    // when we hit some limit.  make sure z impulse was not too big (fell
    // off the edge)
	break;
  case STEP3_FIND_12OCLOCK:
    // run a CCW sliding trajectory and stop when we're moving horizontally to left.
	break;
  case STEP4_FIND_6OCLOCK:
    // run a CW sliding trajectory and stop when we're moving horizontally to left.
	break;
  case STEP5_MOVE_TO_KEYHOLE:
    // run a sliding trajectory and stop when we see force/torque or when we
    // hit the distance limit
	break;
  case STEP6_VERIFY_KEYHOLE_TOP:
    // slide up and stop on force/torque
	break;
  case STEP7_VERIFY_KEYHOLE_BOTTOM:
    // slide down and stop on force/torque
	break;
  case STEP8_INSERT:
    // force-driven AF while wiggling and vibrating
    current_step->evaluate(tc,dt);
    end_position=current_step->end_position;
    if ((current_step->runstate == DONE) ||
	(current_step->runstate == ABORT)) {
      runstate = current_step->runstate;
    }
    if ((runstate==DONE) || (runstate==ABORT)) {
      delete current_step;
      current_step=NULL;
    }
    break;
  }
}

InsertKeyTraj::InsertKeyStep::InsertKeyStep(INSERTION_STEP _stepname) :
  Trajectory("InsertKey"),stepname(_stepname) {
}

InsertKeyTraj::InsertKeyStep::~InsertKeyStep() {
}

InsertKeyTraj::InsertKeyStep8::InsertKeyStep8() :
  InsertKeyStep(STEP8_INSERT),
  total_shift(0)
{
  start_jointpos = gfeplug->target_arm_position;
  original_position = (R3) gfeplug->endpoint;
  original_rotation = (SO3) gfeplug->endpoint;
}

void InsertKeyTraj::InsertKeyStep8::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
  const double torque_servo_gain=0.00012;
  double y_torque = gfeplug->filtered_ft_torque[1];
  double x_shift = -y_torque * torque_servo_gain;

  // maintain our total amount of X correction so that we can limit it
  // in each direction
  total_shift += x_shift;
  if (total_shift > 0.020) {
    total_shift = 0.020;
  } else if (total_shift < -0.020) {
    total_shift = -0.020;
  }

  // change our correction to world coordinates
  R3 ws_shift = (SO3)gfeplug->endpoint * R3(total_shift,0,0);

  // update our target position
  R3 target_position = original_position + ws_shift;

  // calculate the endpoint position correction
  R3 position_correction = target_position - (R3)gfeplug->endpoint;
  
  // Compute the endpoint rotation error by taking the net rotation from
  // the current orientation to the target orientation and converting
  // it to axis-angle format
  so3 rotation_error = (so3)(original_rotation * (! (SO3)gfeplug->endpoint));
  // represent as rotation about each of the axes
  R3 rotation_correction = rotation_error.t() * rotation_error.w();

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
  OWD::JointPos jointpos_error = start_jointpos - tc.q;
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

  for (unsigned int j=0; (j<joint_correction.size()) 
	 && (j<tc.q.size()); ++j) {
    tc.q[j] += joint_correction[j];
  }

  end_position = tc.q;
  return;
}

InsertKeyTraj::InsertKeyStep8::~InsertKeyStep8() {
}

ros::ServiceServer InsertKeyTraj::ss_InsertKey;

// Set up our ROS service for receiving trajectory requests.
bool InsertKeyTraj::Register() {
  ros::NodeHandle n("~");
  ss_InsertKey = n.advertiseService("InsertKey",&InsertKeyTraj::InsertKey);
  return true;
}

// Shut down our service so that it's no longer listed by the ROS master
void InsertKeyTraj::Shutdown() {
  ss_InsertKey.shutdown();
}

InsertKeyTraj::~InsertKeyTraj() {
  gfeplug->flush_recorder_data = true;
  gfeplug->current_traj=NULL;
}




