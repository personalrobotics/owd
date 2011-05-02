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

#ifndef OWD_PLUGIN_HH
#define OWD_PLUGIN_HH

#include <vector>
#include <string>
#include <stdint.h>
#include "Link.hh"

class WAM;

namespace OWD {

  class WamDriver;
  class Trajectory;

  /// Base class for OWD user-defined plugins.  Users can use the
  /// interfaces in this class for interacting with the WAM from
  /// within their own OWD runtime-loadable plugin.  This class
  /// can also be subclassed for users who want to publish their
  /// own data in sync with OWD's message publishing.
  class Plugin {
  public:
    
    Plugin();
    ~Plugin();

    /// \brief Publish user data
    ///
    /// If you subclass the Plugin class and override this function,
    /// OWD will automatically call it at the rate set by the ROS param
    /// <b>~/publish_frequency</b>.
    virtual void Publish();

    /// \brief Add a trajectory to the OWD trajectory queue
    ///
    /// \param traj A pointer to a Trajectory instance
    /// \param failure_reason A string describing why the trajectory
    ///                       was rejected (if it was)
    /// \returns The non-zero trajectory id on success, or zero on failure.
    static uint32_t AddTrajectory(Trajectory *traj,
				  std::string &failure_reason);

    /// \brief Move the hand
    ///
    /// Send a position movement command to the model 280 hand (if present)
    ///
    /// \param p The four target positions, in radians
    /// \returns True on success, false otherwise
    static bool hand_move(std::vector<double> p);

    /// \brief Set hand velocity
    ///
    /// Send a velocity command to the model 280 hand (if present)
    ///
    /// \param v The four velocities, in radians per second
    /// \returns True on success, false otherwise
    static bool hand_velocity(std::vector<double> v);

    /// \brief Set hand torque
    ///
    /// Send a raw torque command to the model 280 hand (if present)
    ///
    /// \param t The four torques, in puck units (approx mA)
    /// \returns True on success, false otherwise
    static bool hand_torque(std::vector<double> t);

    /// \brief Tare the force/torque sensor
    ///
    /// Tare (zero-out) the force/torque sensor (if present)
    ///
    /// \returns True on success, false otherwise
    static bool ft_tare();

    /// \brief Current position
    ///
    /// A read-only reference to the arm joint values.
    ///
    /// \attention 4-DOF wams will only have 4 values, not 7
    static const std::vector<double> &arm_position;

    /// \brief Target position for PID controllers
    ///
    /// A read-only reference to the target position
    /// that the controllers are trying to hold for each joint.
    ///
    /// \attention 4-DOF wams will only have 4 values, not 7
    static const std::vector<double> &target_arm_position;

    /// \brief Current torques from PID controllers
    ///
    /// A read-only reference to the torques calculated by the last
    /// iteration of the individual PID controllers.
    ///
    /// \attention 4-DOF wams will only have 4 values, not 7
    static const std::vector<double> &pid_torque;

    /// \brief Current torques from dynamic model
    ///
    /// A read-only reference to the torques calculated for each joint
    /// by the dynamic model.  Includes compensation for gravity and
    /// for the dynamic forces created by the specified velocity and
    /// acceleration.
    ///
    /// \attention 4-DOF wams will only have 4 values, not 7
    static const std::vector<double> &dynamic_torque;

    /// \brief Current torques from active trajectory
    ///
    /// A read-only reference to the torques most recently specified
    /// by the active trajectory.
    ///
    /// \attention 4-DOF wams will only have 4 values, not 7
    static const std::vector<double> &trajectory_torque;

    /// \brief Current forces from F/T sensor
    ///
    /// A read-only reference to the X/Y/Z forces in Newtons from the
    /// force/torque sensor
    ///
    /// \attention vector will have a size of zero if the force/torque
    /// sensor is not present
    static const std::vector<double> &ft_force;

    /// \brief Current torques from F/T sensor
    ///
    /// A read-only reference to the X/Y/Z torques in Newton-meters
    /// from the force/torque sensor
    ///
    /// \attention vector will have a size of zero if the force/torque
    /// sensor is not present
    static const std::vector<double> &ft_torque;

    /// \brief Current position of the hand joints
    ///
    /// A read-only reference to the position of the four hand joints
    /// for the model 280 hand
    ///
    /// \attention vector will have a size of zero if a 280 hand is
    /// not present
    static const std::vector<double> &hand_position;

    /// \brief Target position of the hand joints
    ///
    /// A read-only reference to the target position of the four hand
    /// joints for the model 280 hand
    ///
    /// \attention vector will have a size of zero if a 280 hand is
    /// not present
    static const std::vector<double> &target_hand_position;

    /// \brief Current strain gauge readings from the hand
    ///
    /// A read-only reference to the straingauge readings from the
    /// three fingers for the model 280 hand
    ///
    /// \attention vector will have a size of zero if a 280 hand is
    /// not present
    static const std::vector<double> &strain;

    /// \brief Finger 1 tactile sensor values
    ///
    /// A read-only reference to the readings from the 24 tactile
    /// sensors on finger 1
    ///
    /// \attention vector will have a size of zero if the tactile
    /// sensors are not present
    static const std::vector<float> &tactile_f1;

    /// \brief Finger 2 tactile sensor values
    ///
    /// A read-only reference to the readings from the 24 tactile
    /// sensors on finger 2
    ///
    /// \attention vector will have a size of zero if the tactile
    /// sensors are not present
    static const std::vector<float> &tactile_f2;

    /// \brief Finger 3 tactile sensor values
    ///
    /// A read-only reference to the readings from the 24 tactile
    /// sensors on finger 3
    ///
    /// \attention vector will have a size of zero if the tactile
    /// sensors are not present
    static const std::vector<float> &tactile_f3;

    /// \brief Palm tactile sensor values
    ///
    /// A read-only reference to the readings from the 24 tactile
    /// sensors on the palm
    ///
    /// \attention vector will have a size of zero if the tactile
    /// sensors are not present
    static const std::vector<float> &tactile_palm;

    static const SE3 &endpoint;

  private:
    friend class ::WAM;
    friend class WamDriver;

    static void PublishAll();
    static WAM *wam;
    static WamDriver *wamdriver;

    static std::vector<double> _arm_position;
    static std::vector<double> _target_arm_position;
    static std::vector<double> _pid_torque;
    static std::vector<double> _dynamic_torque;
    static std::vector<double> _trajectory_torque;
    static std::vector<double> _ft_force;
    static std::vector<double> _ft_torque;
    static std::vector<double> _hand_position;
    static std::vector<double> _target_hand_position;
    static std::vector<double> _strain;
    static std::vector<float> _tactile_f1;
    static std::vector<float> _tactile_f2;
    static std::vector<float> _tactile_f3;
    static std::vector<float> _tactile_palm;
    static SE3 _endpoint;

    static std::vector<Plugin *> children;

  };

};

#endif // OWD_PLUGIN_HH
