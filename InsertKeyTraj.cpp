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
  insertion_step(STEP1_FIND_SURFACE)
{

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
     8.  Push while wiggling left/right.
   */
  switch (insertion_step) {
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
	break;
  }

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
}




