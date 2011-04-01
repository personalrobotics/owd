/***********************************************************************

  Copyright 2010 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

  This file is part of owd.

  owd is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  owd is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***********************************************************************/

#include "bhd280.hh"

BHD_280::BHD_280(CANbus *cb) : node("bhd"), bus(cb) {
  AdvertiseAndSubscribe(node);
  GetParameters(node);
  tf_broadcaster = new tf::TransformBroadcaster();
  bhstate.state = pr_msgs::BHState::state_done;
  bhstate.temperature=0.0f;
  bhstate.positions.resize(4, 0.0f);
  bhstate.secondary_positions.resize(3, 0.0f);
  bhstate.strain.resize(3, 0.0f);
  bhstate.internal_state.resize(4);
  
  const double PI=3.14159;
  const double LINK2_OFFSET=2.46/180.0*PI;
  const double LINK3_OFFSET=39.56/180.0*PI;

  btQuaternion PI_YAW,ZERO;
  ZERO.setRPY(0,0,0);
  PI_YAW.setRPY(0,0,PI);

  // create transforms from WAM7 to origin of each finger's link1
  finger_link1_base[0]=btTransform(ZERO,btVector3(0,-0.025,0.135));
  finger_link1_base[1]=btTransform(ZERO,btVector3(0, 0.025,0.135));
  finger_link1_base[2]=btTransform(PI_YAW,btVector3(0,0,0.135));

  // create transforms from each link to the next
  btQuaternion HALFPI_ROLL_PLUS_LINK2_PITCH;
  HALFPI_ROLL_PLUS_LINK2_PITCH.setRPY(PI/2,LINK2_OFFSET,0);
  btQuaternion LINK3_YAW;
  LINK3_YAW.setRPY(0,0,LINK3_OFFSET);

  // from link1 to link2 origin
  finger_link2_base=btTransform(HALFPI_ROLL_PLUS_LINK2_PITCH,btVector3(.05,0,0));
  // from link2 to link3 origin
  finger_link3_base=btTransform(LINK3_YAW,btVector3(0.070,0,0));
}

BHD_280::~BHD_280() {
  Unadvertise();
}

void BHD_280::AdvertiseAndSubscribe(ros::NodeHandle &n) {
  pub_handstate = n.advertise<pr_msgs::BHState>("handstate", 10);
  ss_gethanddof = n.advertiseService("GetHandDOF",
					&BHD_280::GetDOF,this);
  ss_movehand = n.advertiseService("MoveHand",
				      &BHD_280::MoveHand,this);
  // DON'T ALLOW RESET HAND WITH THE 280 MODEL; IT SEEMS LIKE IT'S
  // UPSETTING THE SAFETY PUCK
  //  ss_resethand = n.advertiseService("ResetHand",
  //				       &BHD_280::ResetHand,this);
  ss_relaxhand = n.advertiseService("RelaxHand",
				       &BHD_280::RelaxHand,this);
  ss_gethandprop = n.advertiseService("SetProperty",
				       &BHD_280::SetHandProperty,this);
  ss_sethandprop = n.advertiseService("GetProperty",
				       &BHD_280::GetHandProperty,this);
}

void BHD_280::GetParameters(ros::NodeHandle &n) {
  n.param("max_velocity",max_velocity,6.0);
}
  
void BHD_280::Unadvertise() {
  pub_handstate.shutdown();
  ss_gethanddof.shutdown();
  ss_movehand.shutdown();
  ss_resethand.shutdown();
  ss_relaxhand.shutdown();
}

void BHD_280::Pump(ros::TimerEvent const& e) {
  // publish our state info
  this->Publish();

  // let the driver update
  // this->Update();
}

bool BHD_280::Publish() {
  int32_t state[4];

  bhstate.header.stamp = ros::Time::now();

  if (bus->hand_get_positions(bhstate.positions[0],
			      bhstate.positions[1],
			      bhstate.positions[2],
			      bhstate.positions[3]) != OW_SUCCESS) {
    ROS_WARN_NAMED("bhd280","Unable to get positions from CANbus object");
    return false;
  }
    
  bus->hand_get_strain(bhstate.strain[0],
		       bhstate.strain[1],
		       bhstate.strain[2]);

  bus->hand_get_distal_positions(bhstate.secondary_positions[0],
				 bhstate.secondary_positions[1],
				 bhstate.secondary_positions[2]);

  bus->hand_get_state(state);
  // combine all the individual states into one
  // UNINIT has the highest priority
  // MOVING has priority over STALLED
  // STALLED has priority over DONE

  // set the default case
  bhstate.state=pr_msgs::BHState::state_done;


  // the internal_state field will eventually become the regular
  // state field once herbcontroller is updated
  for (unsigned int i=0; i<4; ++i) {
    if (state[i] == CANbus::HANDSTATE_UNINIT) {
      bhstate.internal_state[i] = pr_msgs::BHState::state_uninitialized;
    } else if (state[i] == CANbus::HANDSTATE_MOVING) {
      bhstate.internal_state[i] = pr_msgs::BHState::state_moving;
    } else if (state[i] == CANbus::HANDSTATE_STALLED) {
      bhstate.internal_state[i] = pr_msgs::BHState::state_stalled;
    } else if (state[i] == CANbus::HANDSTATE_DONE) {
      bhstate.internal_state[i] = pr_msgs::BHState::state_done;
    } else {
      bhstate.internal_state[i] = 0;
    }
  }

  for (unsigned int i=0; i<4; ++i) {
    if (state[i] == CANbus::HANDSTATE_UNINIT) {
      bhstate.state = pr_msgs::BHState::state_uninitialized;
      break; // don't have to check any others
    } else if (state[i] == CANbus::HANDSTATE_MOVING) {
      // moving always overrides state_stalled or state_done
      bhstate.state = pr_msgs::BHState::state_moving;
      
      /* Until herbcontroller is updated to know about the stalled
	 state, we will ignore stalled and treat it as state_done
	 
	 // } else if ((state[i] == CANbus::HANDSTATE_STALLED) && 
	 // (bhstate.state != pr_msgs::BHState::state_moving)) {
	 //   // stalled cannot override moving
	 // bhstate.state = pr_msgs::BHState::state_stalled;
      */
    }
  }
  
  pub_handstate.publish(bhstate);

  for (int f=0; f<3; ++f) {
      char jref[50], jname[50];
      btQuaternion YAW;
      
      // LINK 1
      snprintf(jref,50,"wam7");
      snprintf(jname,50,"finger%d_0",f);
      if (f==0) {
	YAW.setRPY(0,0,-bhstate.positions[3]);
      } else if (f==1) {
	YAW.setRPY(0,0, bhstate.positions[3]);
      } else {
	// finger 3 has link1 fixed
	YAW.setRPY(0,0,0);
      }
      btTransform finger_tf = finger_link1_base[f] * btTransform(YAW);
      tf_broadcaster->sendTransform(tf::StampedTransform(finger_tf,ros::Time::now(),jref,jname));

      // LINK 2
      snprintf(jref,50,"finger%d_0",f);
      snprintf(jname,50,"finger%d_1",f);
      YAW.setRPY(0,0,bhstate.positions[f]);
      finger_tf = finger_link2_base * btTransform(YAW);
      tf_broadcaster->sendTransform(tf::StampedTransform(finger_tf,ros::Time::now(),jref,jname));

      // LINK 3
      snprintf(jref,50,"finger%d_1",f);
      snprintf(jname,50,"finger%d_2",f);
      YAW.setRPY(0,0,bhstate.positions[f]/3.0); // distal link moves at 1/3
      finger_tf = finger_link3_base * btTransform(YAW);
      tf_broadcaster->sendTransform(tf::StampedTransform(finger_tf,ros::Time::now(),jref,jname));

  }

  return true;
}
  
void BHD_280::createT(double a, double alpha, double d, double theta, double result[4][4])
{
  // code by Andrew Yeager, CMU
  double cosTheta = cos(theta);
  double sinTheta = sin(theta);
  double cosAlpha = cos(alpha);
  double sinAlpha = sin(alpha);     
  
  result[0][0] = cosTheta;
  result[0][1] = -sinTheta;
  result[0][2] = 0;
  result[0][3] = a;
  result[1][0] = sinTheta*cosAlpha;
  result[1][1] = cosTheta*cosAlpha;
  result[1][2] = -sinAlpha;
  result[1][3] = -sinAlpha*d;
  result[2][0] = sinTheta*sinAlpha;
  result[2][1] = cosTheta*sinAlpha;
  result[2][2] = cosAlpha;
  result[2][3] = cosAlpha*d;
  result[3][0] = 0;
  result[3][1] = 0;
  result[3][2] = 0;
  result[3][3] = 1;  
}

// Handle requests for DOF information
bool BHD_280::GetDOF(pr_msgs::GetDOF::Request &req,
			 pr_msgs::GetDOF::Response &res) {
  ROS_INFO_NAMED("service","Received GetDOF; returning 4");
  res.nDOF =4;
  return true;
}

// Relax command
bool BHD_280::RelaxHand(pr_msgs::RelaxHand::Request &req,
			pr_msgs::RelaxHand::Response &res) {
  if (bus->hand_relax() == OW_SUCCESS) {
    return true;
  } else {
    return false;
  }
}

// Reset command
bool BHD_280::ResetHand(pr_msgs::ResetHand::Request &req,
			pr_msgs::ResetHand::Response &res) {
  if (bus->hand_reset() == OW_SUCCESS) {
    bhstate.state = pr_msgs::BHState::state_done;
    return true;
  } else {
    bhstate.state = pr_msgs::BHState::state_uninitialized;
    return false;
  }
  return false;
}

// Move command
bool BHD_280::MoveHand(pr_msgs::MoveHand::Request &req,
		       pr_msgs::MoveHand::Response &res) {
  if (bhstate.state == pr_msgs::BHState::state_uninitialized) {
    ROS_WARN_NAMED("bhd280","Rejected MoveHand: hand is uninitialized");
    return false;
  }

  if (req.positions.size() != 4) {
    ROS_ERROR_NAMED("bhd280","Expected 4 joints for MoveHand; received %zd",
	      req.positions.size());
    return false;
  }
  
  if (req.movetype == pr_msgs::MoveHand::Request::movetype_position) {
    if (req.positions.size() != 4) {
      ROS_ERROR_NAMED("bhd280", "MoveHand requires 4 position arguments");
      return false;
    }
    bhstate.state = pr_msgs::BHState::state_moving;
    ROS_INFO_NAMED("bhd280", "Received MoveHand Position %2.2f %2.2f %2.2f %2.2f",
		   req.positions[0],
		   req.positions[1],
		   req.positions[2],
		   req.positions[3]);
    pub_handstate.publish(bhstate);  // ensure at least 1 moving msg
    if (bus->hand_move(req.positions) != OW_SUCCESS) {
      return false;
    } else {
      return true;
    }
  } else if (req.movetype == pr_msgs::MoveHand::Request::movetype_velocity) {
    if (req.positions.size() != 4) {
      ROS_ERROR_NAMED("bhd280", "MoveHand Velocity requires 4 velocity arguments");
      return false;
    }
    ROS_INFO_NAMED("bhd280", "Received MoveHand Velocity %2.2f %2.2f %2.2f %2.2f",
		   req.positions[0],
		   req.positions[1],
		   req.positions[2],
		   req.positions[3]);
    bhstate.state = pr_msgs::BHState::state_moving;

    /*  this was a temporary override to change velocity commands to
	position moves just for the Personal Robotics project at Intel.
   ROS_ERROR_NAMED("bhd280", "Received MoveHand Velocity; changing to MoveHand");
    // change all the velocities to positions at the appropriate end
    for (unsigned int i=0; i<4; ++i) {
      if (req.positions[i] > 0) {
	req.positions[i] = 2.6;
      } else {
	req.positions[i] = 0;
      }
    }
    req.movetype = pr_msgs::MoveHand::Request::movetype_position;
    return MoveHand(req,res);
    */
    

    for (unsigned int i=0; i<4; ++i) {
      if (req.positions[i] < -max_velocity) {
	ROS_WARN_NAMED("bhd280",
		       "Joint %d velocity request of %2.2f limited to max of %2.2f radians/sec",
		       i+1,req.positions[i],-max_velocity);
	req.positions[i]=-max_velocity;
      } else if (req.positions[i] > max_velocity) {
	ROS_WARN_NAMED("bhd280",
		       "Joint %d velocity request of %2.2f limited to max of %2.2f radians/sec",
		       i+1,req.positions[i],max_velocity);
	req.positions[i]=max_velocity;
      }
    }
    pub_handstate.publish(bhstate);  // ensure at least 1 moving msg
    if (bus->hand_velocity(req.positions[0],
			   req.positions[1],
			   req.positions[2],
			   req.positions[3]) != OW_SUCCESS) {
      return false;
    } else {
      return true;
    }
  } else if (req.movetype == 3) { // "hidden" torque mode
    bhstate.state = pr_msgs::BHState::state_moving;
    pub_handstate.publish(bhstate);  // ensure at least 1 moving msg
    if (bus->hand_torque(req.positions[0],
			 req.positions[1],
			 req.positions[2],
			 req.positions[3]) != OW_SUCCESS) {
      return false;
    } else {
      return true;
    }
  } else {
    return false; // unknown move type
  }
}

bool BHD_280::SetHandProperty(pr_msgs::SetHandProperty::Request &req,
			  pr_msgs::SetHandProperty::Response &res) {
  if (bus->hand_set_property(req.nodeid,req.property,req.value) != OW_SUCCESS) {
    ROS_WARN_NAMED("bhd280","Failed to set property %d = %d on puck %d",
		   req.property,req.value,req.nodeid);
    return false;
  }
  return true;
}

bool BHD_280::GetHandProperty(pr_msgs::GetHandProperty::Request &req,
			  pr_msgs::GetHandProperty::Response &res) {
  if (bus->hand_get_property(req.nodeid,req.property,&res.value) != OW_SUCCESS) {
    ROS_WARN_NAMED("bhd280","Failure getting property %d from puck %d",
		   req.property,req.nodeid);
    return false;
  }
  return true;
}
