#include "bhd280.hh"

BHD280::BHD280(CANbus *cb) : node("bhd"), bus(cb) {
  AdvertiseAndSubscribe(node);
  bhstate.state = pr_msgs::BHState::state_uninitialized;
  bhstate.temperature=0.0f;
  bhstate.positions.resize(4, 0.0f);
  bhstate.strain.resize(4, 0.0f);
}

BHD280::~BHD280() {
  Unadvertise();
}

void BHD280::AdvertiseAndSubscribe(ros::NodeHandle &n) {
  pub_handstate = n.advertise<pr_msgs::BHState>("handstate", 10);
  ss_gethanddof = n.advertiseService("GetHandDOF",
					&BHD280::GetDOF,this);
  ss_movehand = n.advertiseService("MoveHand",
				      &BHD280::MoveHand,this);
  ss_resethand = n.advertiseService("ResetHand",
				       &BHD280::ResetHand,this);
  ss_relaxhand = n.advertiseService("RelaxHand",
				       &BHD280::RelaxHand,this);
}

void BHD280::Unadvertise() {
  pub_handstate.shutdown();
  ss_gethanddof.shutdown();
  ss_movehand.shutdown();
  ss_resethand.shutdown();
  ss_relaxhand.shutdown();
}

bool BHD280::Publish() {
  int32_t state;
  if (bus->hand_get_positions(bhstate.positions[0],
			      bhstate.positions[1],
			      bhstate.positions[2],
			      bhstate.positions[3]) != OW_SUCCESS) {
    ROS_WARN_NAMED("bhd280","Unable to get positions from CANbus object");
    return false;
  }
    
  bus->hand_get_state(state);
  if (state == CANbus::HANDSTATE_UNINIT) {
    bhstate.state = pr_msgs::BHState::state_uninitialized;
  } else if (state == CANbus::HANDSTATE_DONE) {
    bhstate.state = pr_msgs::BHState::state_done;
  } else if (state == CANbus::HANDSTATE_MOVING) {
    bhstate.state = pr_msgs::BHState::state_moving;
  }
  
  pub_handstate.publish(bhstate);
  return true;
}
  

// Handle requests for DOF information
bool BHD280::GetDOF(pr_msgs::GetDOF::Request &req,
			 pr_msgs::GetDOF::Response &res) {
  ROS_INFO_NAMED("service","Received GetDOF; returning 4");
  res.nDOF =4;
  return true;
}

// Relax command
bool BHD280::RelaxHand(pr_msgs::RelaxHand::Request &req,
			    pr_msgs::RelaxHand::Response &res) {
  if (bus->hand_relax() == OW_SUCCESS) {
    return true;
  } else {
    return false;
  }
}

// Reset command
bool BHD280::ResetHand(pr_msgs::ResetHand::Request &req,
			    pr_msgs::ResetHand::Response &res) {
  if (bus->hand_reset() == OW_SUCCESS) {
    return true;
  } else {
    return false;
  }
}

// Move command
bool BHD280::MoveHand(pr_msgs::MoveHand::Request &req,
			   pr_msgs::MoveHand::Response &res) {
  if (bhstate.state == pr_msgs::BHState::state_uninitialized) {
    ROS_WARN_NAMED("bhd280","Rejected MoveHand: hand is uninitialized");
    return false;
  }

  if (req.positions.size() != 4) {
    ROS_ERROR_NAMED("bhd280","Expected 4 joints for MoveHand; received %d",
	      req.positions.size());
    return false;
  }
  
  if (req.movetype == pr_msgs::MoveHand::Request::movetype_position) {
    bhstate.state = pr_msgs::BHState::state_moving;
    pub_handstate.publish(bhstate);  // ensure at least 1 moving msg
    if (bus->hand_move(req.positions[0],
		       req.positions[1],
		       req.positions[2],
		       req.positions[3]) != OW_SUCCESS) {
      return false;
    } else {
      return true;
    }
  } else if (req.movetype == pr_msgs::MoveHand::Request::movetype_velocity) {
    bhstate.state = pr_msgs::BHState::state_moving;
    pub_handstate.publish(bhstate);  // ensure at least 1 moving msg
    if (bus->hand_velocity(req.positions[0],
			   req.positions[1],
			   req.positions[2],
			   req.positions[3]) != OW_SUCCESS) {
      return false;
    } else {
      return true;
    }
  }
}
