/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "JacobianTest.h"
#include "openwam/Kinematics.hh"

bool JacobianTestTraj::JacobianTest(pr_msgs::Reset::Request &req,
				    pr_msgs::Reset::Response &res) {
  ROS_INFO("GfePlugin: received JacobianTest service call");

  // compute a new trajectory
  try {
    JacobianTestTraj *newtraj = new JacobianTestTraj();
    // send it to the arm
    std::string reason;
    int id = OWD::Plugin::AddTrajectory(newtraj,reason);
    if (id > 0) {
      return true;
    } else {
      ROS_ERROR("JacobianTestTraj could not AddTrajectory: %s",
		reason.c_str());
      return false;
    }
  } catch (const char *err) {
    ROS_ERROR("Could not create JacobianTestTraj: %s",err);
    return false;
  }
}

JacobianTestTraj::JacobianTestTraj(): OWD::Trajectory("GFE Jacobian Test"),
				      stoptraj(false)
{
  if (gfeplug) {
    start_position=gfeplug->target_arm_position;
    end_position = start_position;

    // create the service that the client can use to stop the force
    ros::NodeHandle n("~");
    ss_StopTraj = n.advertiseService("StopTraj",&JacobianTestTraj::StopTraj, this);

  } else {
    throw "Cannot access Plugin instance for getting OWD values";
  }
}

// Stop the trajectory when asked by a client
bool JacobianTestTraj::StopTraj(pr_msgs::Reset::Request &req,
				pr_msgs::Reset::Response &res) {
  stoptraj=true;
  return true;
}

void JacobianTestTraj::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
  time += dt;

  if (stoptraj) {
    runstate=OWD::Trajectory::DONE;
    return;
  }

  // pick a small random movement in 6-DOF workspace
  R6 ws_tweak( ((double)random() / (double)RAND_MAX -0.5)/100.0, //   +/-5mm
	       ((double)random() / (double)RAND_MAX -0.5)/100.0,
	       ((double)random() / (double)RAND_MAX -0.5)/100.0,
	       ((double)random() / (double)RAND_MAX -0.5)/20.0, //    +/- .025 radians
	       ((double)random() / (double)RAND_MAX -0.5)/20.0,
	       ((double)random() / (double)RAND_MAX -0.5)/20.0);

  ROS_INFO_NAMED("applyforce","Workspace desired movement of [%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f]",
		 ws_tweak.v[0],ws_tweak.v[1],ws_tweak.v[2],
                 ws_tweak.w[0],ws_tweak.w[1],ws_tweak.w[2]);
  
  // get the correction from the Jacobian Pseudo-Inverse
  OWD::JointPos joint_correction =
    gfeplug->JacobianPseudoInverse_times_vector(ws_tweak);

  // estimate the actual workspace movement
  R6 workspace_movement;
  try {
    workspace_movement = gfeplug->Jacobian_times_vector(joint_correction);
  } catch (const char *err) {
    ROS_ERROR_NAMED("applyforce","Could not multiply Jacobian times correction");
    return;
  }

  ROS_INFO_NAMED("applyforce","Workspace actual movement of  [%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f]",
		 workspace_movement.v[0],workspace_movement.v[1],
                 workspace_movement.v[2],workspace_movement.w[0],
                 workspace_movement.w[1],workspace_movement.w[2]);

  // compare the original ws change to the calculated value
  for (int i=0; i<6; ++i) {
    double percent_error;
    if (i<3) {
      percent_error = (ws_tweak.v[i] - workspace_movement.v[i])
        / ws_tweak.v[i];
    } else {
      percent_error = (ws_tweak.w[i-3] - workspace_movement.w[i-3])
        / ws_tweak.w[i-3];
    }
    if (fabs(percent_error) > .01) {
      ROS_WARN_NAMED("applyforce","Dim %d: remaining error of %2f%%",
	       i+1,
	       percent_error * 100.0);
    }
  }
  if (OWD::Kinematics::thresholded_count > 0) {
    std::stringstream msg("There were ");
    msg << OWD::Kinematics::thresholded_count;
    msg << " thresholded singular values:";
    for (int i=0; i<OWD::Kinematics::thresholded_count; ++i) {
      msg << " ";
      msg << OWD::Kinematics::thresholded_values[i];
    }
    ROS_ERROR_NAMED("applyforce","%s",msg.str().c_str());
  }
  if (OWD::Kinematics::zero_count > 0) {
    ROS_ERROR_NAMED("applyforce","There were %d zero singular values",
	     OWD::Kinematics::zero_count);
  }

  ROS_INFO_NAMED("applyforce","Correction was [%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f]",
		 joint_correction[0],joint_correction[1],joint_correction[2],
		 joint_correction[3],joint_correction[4],joint_correction[5],
		 joint_correction[6]);

  // make a big change in the joints in order to try a new configuration
  tc.q[0] = random_joint_value( 0.52,5.76);
  tc.q[1] = random_joint_value(-1.96,1.96);
  tc.q[2] = random_joint_value(-2.73,2.73);
  tc.q[3] = random_joint_value(-0.86,3.13);
  tc.q[4] = random_joint_value(-4.79,1.30);
  tc.q[5] = random_joint_value(-1.56,1.56);
  tc.q[6] = random_joint_value(-2.99,2.99);
  
}

bool JacobianTestTraj::Register() {
  ros::NodeHandle n("~");
  ss_JacobianTest = n.advertiseService("JacobianTest",&JacobianTestTraj::JacobianTest);
  return true;
}

void JacobianTestTraj::Shutdown() {
  ss_JacobianTest.shutdown();
}

JacobianTestTraj::~JacobianTestTraj() {
  ss_StopTraj.shutdown();
}

ros::ServiceServer JacobianTestTraj::ss_JacobianTest;
