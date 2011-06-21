/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
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

#include "Plugin.hh"
#include "WAM.hh"
#include "../openwamdriver.h"

namespace OWD {

  Plugin::Plugin() {
    // register this instance
    children.push_back(this);
  }

  Plugin::~Plugin() {
    for (std::vector<Plugin *>::iterator it=children.begin(); it!=children.end(); ++it) {
      if (*it == this) {
	children.erase(it);
	return;
      }
    }
  }

  void Plugin::Publish() {
  }

  void Plugin::PublishAll() {
    for (std::vector<Plugin *>::iterator it=children.begin(); it!=children.end(); ++it) {
      (*it)->Publish();
    }
  }


  uint32_t Plugin::AddTrajectory(Trajectory *traj,
				 std::string &failure_reason) {
    if (!wamdriver) {
      failure_reason="invalid WamDriver pointer; should have been set by openwamdriver.cpp";
      return 0;
    }
    return wamdriver->AddTrajectory(traj, failure_reason);
  }

  bool Plugin::hand_move(std::vector<double> p) {
    if (!wam) {
      throw "invalid WAM pointer; should have been set by openwamdriver.cpp";
    }
    return (wam->bus->hand_move(p) == OW_SUCCESS);
  }

  bool Plugin::hand_velocity(std::vector<double> v) {
    if (!wam) {
      throw "invalid WAM pointer; should have been set by openwamdriver.cpp";
    }
    return (wam->bus->hand_velocity(v) == OW_SUCCESS);
  }

  bool Plugin::hand_torque(std::vector<double> t) {
    if (!wam) {
      throw "invalid WAM pointer; should have been set by openwamdriver.cpp";
    }
    return (wam->bus->hand_torque(t) == OW_SUCCESS);
  }

  bool Plugin::ft_tare() {
    if (!wam) {
      throw "invalid WAM pointer; should have been set by openwamdriver.cpp";
    }
    return (wam->bus->ft_tare() == OW_SUCCESS);
  }

  //  const double* Plugin::Jacobian0() {
  //    return (double *) OWD::Kinematics::Jacobian0;
  //  }

  //  const double* Plugin::JacobianEE() {
  //    return (double *) OWD::Kinematics::JacobianEE;
  //  }

  const R6 Plugin::Jacobian_times_vector(JointPos v) {
    R6 result;
    OWD::Kinematics::Jacobian0_times_vector(&v[0], (double *)result);
    return result;
  }

  const JointPos Plugin::JacobianPseudoInverse_times_vector(R6 &v) {
    double result[OWD::Kinematics::NJOINTS];
    OWD::Kinematics::JacobianPseudoInverse_times_vector(v,result);
    JointPos jp;
    jp.SetFromArray(OWD::Kinematics::NJOINTS,result);
    return jp;
  }

  const JointPos Plugin::JacobianTranspose_times_vector(R6 &v) {
    double result[OWD::Kinematics::NJOINTS];
    OWD::Kinematics::Jacobian0Transpose_times_vector(v,result);
    JointPos jp;
    jp.SetFromArray(OWD::Kinematics::NJOINTS,result);
    return jp;
  }

  const JointPos Plugin::Nullspace_projection(JointPos v) {
    double result[OWD::Kinematics::NJOINTS];
    OWD::Kinematics::Nullspace_projection(&v[0], result);
    JointPos jp;
    jp.SetFromArray(OWD::Kinematics::NJOINTS, result);
    return jp;
  }

  std::vector<double> Plugin::_arm_position;
  std::vector<double> Plugin::_target_arm_position;
  std::vector<double> Plugin::_pid_torque;
  std::vector<double> Plugin::_dynamic_torque;
  std::vector<double> Plugin::_trajectory_torque;
  std::vector<double> Plugin::_ft_force;
  std::vector<double> Plugin::_ft_torque;
  std::vector<double> Plugin::_hand_position;
  std::vector<double> Plugin::_target_hand_position;
  std::vector<double> Plugin::_strain;
  std::vector<float> Plugin::_tactile_f1;
  std::vector<float> Plugin::_tactile_f2;
  std::vector<float> Plugin::_tactile_f3;
  std::vector<float> Plugin::_tactile_palm;
  SE3 Plugin::_endpoint;
  std::vector<Plugin *> Plugin::children;

  const std::vector<double> &Plugin::arm_position=Plugin::_arm_position;
  const std::vector<double> &Plugin::target_arm_position=Plugin::_target_arm_position;
  const std::vector<double> &Plugin::pid_torque=Plugin::_pid_torque;
  const std::vector<double> &Plugin::dynamic_torque=Plugin::_dynamic_torque;
  const std::vector<double> &Plugin::trajectory_torque=Plugin::_trajectory_torque;
  const std::vector<double> &Plugin::ft_force=Plugin::_ft_force;
  const std::vector<double> &Plugin::ft_torque=Plugin::_ft_torque;
  const std::vector<double> &Plugin::hand_position=Plugin::_hand_position;
  const std::vector<double> &Plugin::target_hand_position=Plugin::_target_hand_position;
  const std::vector<double> &Plugin::strain=Plugin::_strain;
  const std::vector<float> &Plugin::tactile_f1=Plugin::_tactile_f1;
  const std::vector<float> &Plugin::tactile_f2=Plugin::_tactile_f2;
  const std::vector<float> &Plugin::tactile_f3=Plugin::_tactile_f3;
  const std::vector<float> &Plugin::tactile_palm=Plugin::_tactile_palm;
  const SE3 &Plugin::endpoint=Plugin::_endpoint;

  WAM *Plugin::wam = NULL;
  WamDriver *Plugin::wamdriver = NULL;

}; // namespace OWD
