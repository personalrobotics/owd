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

#include "ft.hh"

FT::FT(CANbus *cb) : bus(cb), node("~"), valid_data(false) {
  AdvertiseAndSubscribe(node);
}

FT::~FT() {
  Unadvertise();
}

void FT::AdvertiseAndSubscribe(ros::NodeHandle &n) {
  pub_ft = n.advertise<geometry_msgs::Wrench>("forcetorque", 10);
}

void FT::Unadvertise() {
  pub_ft.shutdown();
}

void FT::Pump(const ros::TimerEvent& e) {
  // publish our state info
  this->Publish();

  // let the driver update
  // this->Update();
}

bool FT::Publish() {
  static double ft_values[6];
  if (bus->ft_get_data(ft_values) != OW_SUCCESS) {
    ROS_WARN_NAMED("ft","Unable to get data from Force/Torque sensor");
    return false;
  }

  wrench.force.x=ft_values[0];
  wrench.force.y=ft_values[1];
  wrench.force.z=ft_values[2];
  wrench.torque.x=ft_values[3];
  wrench.torque.y=ft_values[4];
  wrench.torque.z=ft_values[5];
  pub_ft.publish(wrench);
  
  return true;
}
  
