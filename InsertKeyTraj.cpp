#include "InsertKeyTraj.h"

bool InsertKeyTraj::InsertKey(owd_msgs::InsertKey::Request &req,
			      owd_msgs::InsertKey::Response &res) {

  //  double max_distance=0;
  //  for (int xstep=-1; xstep<2; ++xstep) {
    //    for (int ystep=-1; ystep<2; ++ystep) {
      // move to point
      //      WSMove()
      

  // compute a new trajectory
  try {
    InsertKeyTraj *newtraj = new InsertKeyTraj();

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

InsertKeyTraj::InsertKeyTraj() :
  OWD::Trajectory("InsertKeyTraj",OWD::Trajectory::random_id()),
  current_step(NULL)
{
  start_position=hybridplug->target_arm_position;
  end_position = start_position;
  current_step = new InsertKeyStep8();
  hybridplug->current_traj=this;
}

void InsertKeyTraj::evaluate_abs(OWD::Trajectory::TrajControl &tc, double t) {
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
    current_step->evaluate_abs(tc,t);
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
  Trajectory("InsertKey",OWD::Trajectory::random_id()),
  stepname(_stepname),
  last_time(-1)
 {
}

InsertKeyTraj::InsertKeyStep::~InsertKeyStep() {
}

InsertKeyTraj::InsertKeyStep8::InsertKeyStep8() :
  InsertKeyStep(STEP8_INSERT),
  AFTraj(NULL),
  total_shift(0),
  extra_vibe(NULL),
  vibe_count(0),
  vibe_multiplier(1),
  motion(1),
  motiontime(20),
  horiz_vibe((SO3)hybridplug->endpoint * R3(1,0,0), .004, 0.25),
  vert_vibe((SO3)hybridplug->endpoint * R3(0,1,0), .002, 0.20)
{
 
  start_position=hybridplug->target_arm_position;
  end_position = start_position;
  start_jointpos = hybridplug->target_arm_position;
  original_position = (R3) hybridplug->endpoint;
  original_rotation = (SO3) hybridplug->endpoint;
  AFTraj = new ApplyForceTraj((SO3)hybridplug->endpoint * R3(0,0,1), 5, 0.05);
  AFTraj->SetVibration(1,0,0,0.001,60);
}

void InsertKeyTraj::InsertKeyStep8::evaluate_abs(OWD::Trajectory::TrajControl &tc, double t) {
  
  if (last_time < 0) {
    last_time = t;
  }
  double dt = t - last_time;
  last_time = t;
  motiontime -= dt;
  if (motiontime < 0) {
    ++motion;
    motiontime=10;
  }
  R3 target_position(original_position);
  int i;
  switch (motion) {
  case 0:
    // Straight pushing (might get lucky first time)
    if (motiontime > 4) {
      motiontime=10;
    }
    break;
  case 1:
    i=0;
  case 2:
    i=0;
  case 3:
    i=0;
  case 4:
    i=0;
  case 5:
    i=0;
  case 6:
    i=0;
  case 7:
    // add our horiz. and vert. wiggles
    target_position += horiz_vibe.eval(dt)
      + vert_vibe.eval(dt);
    break;
  case 8:
    runstate=ABORT;
    return;
  }
  R3 travel_in_hand_frame = (!(SO3)hybridplug->endpoint) * 
    ((R3)hybridplug->endpoint - original_position);
  if (travel_in_hand_frame[2] > .05) {
    runstate=DONE; // yay!
    return;
  }





    /*
  case 1:
    // initialize for upper travel limit
    y_shift=0;
    motion=2;
    // fall_through
  case 2:
    // find upper travel limit
    if (hybridplug->filtered_ft_force[1] > 1) {
      upper_y=y_shift;
      motiontime=0;
    } else {
      y_shift += .001/500; // 1mm/sec
      if (y_shift > .020) {
	upper_y=.020;
	motiontime=0;
      }
    }
    break;
  case 3:
    // find lower travel limit
    if (hybridplug->filtered_ft_force[1] < -1) {
      lower_y = y_shift;
      center_y=(upper_y + lower_y)/2;
      if (fabs(center_y) > .005) {
	// we missed the slot, so try finding right edge
	motion=XX;
      }
      motiontime=0;
    } else {
      y_shift -= .001/500; // 1mm/sec
      if (y_shift < -0.020) {
	lower_y = -0.020;
	center_y=(upper_y + lower_y)/2;
	motiontime=0;
      }
    }
    break;
  case 4:
  }

  double y_torque = hybridplug->filtered_ft_torque[1];
  double x_shift(0);
  if (y_torque > 0.2) {
    x_shift -= .0025 / 500;  // at 500hz, this will move 2.5mm per second
  } else if (y_torque < -0.2) {
    x_shift += .0025 / 500;
  }

  // maintain our total amount of X correction so that we can limit it
  // in each direction
  total_shift += x_shift;
  if (total_shift > 0.020) {
    total_shift = 0.020;
  } else if (total_shift < -0.020) {
    total_shift = -0.020;
  }

  // change our correction to world coordinates
  R3 ws_shift = (SO3)hybridplug->endpoint * R3(total_shift,0,0);

  // update our target position
  R3 target_position = original_position; // + ws_shift;

  if (time/4 > vibe_count) {
    if (extra_vibe) {
      delete extra_vibe;
      extra_vibe=NULL;
    }
    switch (vibe_count % 2) {
    case 1:
      extra_vibe=new Vibration((SO3)hybridplug->endpoint *R3(0,1,0),
			       0.0005 * vibe_multiplier,
			       2);
      break;
    case 0:
      extra_vibe=new Vibration((SO3)hybridplug->endpoint *R3(1,0,0),
			       0.001 * vibe_multiplier,
			       1);
      vibe_multiplier *= 1.5;
      break;
    }
    vibe_count++;
  }
  if (extra_vibe) {
    target_position += extra_vibe->eval(dt);
  }

  */


  // update the position target in the ApplyForce trajectory
  AFTraj->endpoint_target = SE3(original_rotation,
				target_position);

  // let ApplyForce calculate the joint positions and torques
  AFTraj->evaluate_abs(tc,t);

  // track the updated joint values
  end_position = tc.q;

  return;
}

InsertKeyTraj::InsertKeyStep8::~InsertKeyStep8() {
  delete AFTraj;
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
  hybridplug->flush_recorder_data = true;
  hybridplug->current_traj=NULL;
}




