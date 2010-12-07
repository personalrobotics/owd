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
  bhstate.strain.resize(4, 0.0f);
  baseShift = btTransform(btMatrix3x3(cos(M_PI_2), -sin(M_PI_2), 0, sin(M_PI_2), cos(M_PI_2), 0, 0, 0, 1), btVector3(0,0,.06));
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
  n.param("max_velocity",max_velocity,2.4);
}
  
void BHD_280::Unadvertise() {
  pub_handstate.shutdown();
  ss_gethanddof.shutdown();
  ss_movehand.shutdown();
  ss_resethand.shutdown();
  ss_relaxhand.shutdown();
}

void BHD_280::Pump(const ros::TimerEvent& e) {
  // publish our state info
  this->Publish();

  // let the driver update
  // this->Update();
}

bool BHD_280::Publish() {
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

  // calculate the transforms
  // code by Andrew Yeager, CMU
  static double t[4][4][4];
  
  static const int r[3] = {-1, 1, 0};
  static const int j[3] = {1, 1, -1};
  
  static const double A[4] = {25, 50, 70, 50};
  static const double D[4] = {84-19, 0, 0, 9.5};  // new 280 hand is 19mm shorter
  static const double PHI[3] = {0, .0429351, .8726646};
    
  int i;

  for (int finger=1; finger <=3; ++finger) {
    
    createT(r[finger-1]*A[0], 0, D[0], r[finger-1] * bhstate.positions[(finger-1)*3] - M_PI_2*j[finger-1], t[0]);
    createT(A[1], M_PI_2, D[1], bhstate.positions[1 + (finger-1)*3] + PHI[1], t[1]);
    createT(A[2], 0, D[2], bhstate.positions[2 + (finger-1)*3] + PHI[2], t[2]);                 
    createT(A[3], -M_PI_2, D[3], 0, t[3]);
    
    //Publish Transform Information
    tf::StampedTransform baseShift_stf(baseShift, ros::Time::now(), "wam7" , "BHDBase");
    tf_broadcaster->sendTransform(baseShift_stf);
    for(i = 0; i < 4; i++) {
      char jref[50], jname[50];
      if(i > 0)
	snprintf(jref, 50, "finger%d_%d", finger-1, i-1);
      else
	snprintf(jref, 50, "BHDBase");
      
      snprintf(jname, 50, "finger%d_%d", finger-1, i);
      
      btTransform finger_tf = btTransform(btMatrix3x3(t[i][0][0],t[i][0][1],t[i][0][2],t[i][1][0],t[i][1][1],t[i][1][2],t[i][2][0],t[i][2][1],t[i][2][2]), btVector3(t[i][0][3]/1000,t[i][1][3]/1000,t[i][2][3]/1000));                                                  
      tf::StampedTransform finger_stf(finger_tf, ros::Time::now(), jref, jname);
      
      
      tf_broadcaster->sendTransform(finger_stf);
    }
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
    ROS_ERROR_NAMED("bhd280","Expected 4 joints for MoveHand; received %d",
	      req.positions.size());
    return false;
  }
  
  if (req.movetype == pr_msgs::MoveHand::Request::movetype_position) {
    bhstate.state = pr_msgs::BHState::state_moving;
    ROS_ERROR_NAMED("bhd280", "Received MoveHand");
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
    ROS_ERROR_NAMED("bhd280", "Received MoveHand Velocity");
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
  }
  else {
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
