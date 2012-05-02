/***********************************************************************

  Copyright 2007-2010 Carnegie Mellon University and Intel Corporation
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

#ifndef OPENWAMDRIVER_H
#define OPENWAMDRIVER_H

#include <list>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <pr_msgs/AddTrajectory.h>
#include <pr_msgs/AddTimedTrajectory.h>
#include <pr_msgs/DeleteTrajectory.h>
#include <pr_msgs/CancelAllTrajectories.h>
#include <pr_msgs/PauseTrajectory.h>
#include <pr_msgs/ReplaceTrajectory.h>
#include <pr_msgs/MassProperties.h>
#include <pr_msgs/SetStiffness.h>
#include <pr_msgs/SetJointStiffness.h>
#include <pr_msgs/SetJointOffsets.h>
#include <pr_msgs/SetSpeed.h>
#include <pr_msgs/GetSpeed.h>
#include <pr_msgs/SetExtraMass.h>
#include <pr_msgs/SetStallSensitivity.h>
#include <pr_msgs/WAMState.h>
#include <pr_msgs/WAMInternals.h>
#include <pr_msgs/GetDOF.h>
#include <pr_msgs/Servo.h>
#include <pr_msgs/Reset.h>
#include <pr_msgs/SetForceInputThreshold.h>
#include <pr_msgs/SetTactileInputThreshold.h>
#include <pr_msgs/CalibrateJoints.h>
#include <pr_msgs/StepJoint.h>
#include <pr_msgs/SetGains.h>
#include <tf/transform_broadcaster.h>

#ifdef BUILD_FOR_SEA
  #include <pr_msgs/WamRequestSeaCtrlTorqLimit.h>
  #include <pr_msgs/WamRequestSeaCtrlKp.h>
  #include <pr_msgs/WamRequestSeaCtrlKd.h>
  #include <pr_msgs/WamRequestSeaCtrlKi.h>
#endif

#include "openwam/Joint.hh"
#include "openwam/TrajType.hh"

// forward declaration of a few classes we keep pointers to, 
// so that we don't have to include their .hh files right now
#include "openwam/WAM.hh"
class CANbus;

namespace OWD {
  class Trajectory;

  class WamDriver
{
public:
  /// Constructor
  /// \param canbus_number numeric suffix for CAN bus device (/dev/can# for
  ///                      ESD cards or /dev/pcan# for PEAK cards)
  /// \param bh_model 260 for the serial hand, 280 for the CANbus hand,
  ///                 or 0 for no hand
  /// \param forcetorque true if the force/torque sensor is installed
  /// \param tactile true if the tactile sensors are installed
  ///                (BH model 280 only)
  WamDriver(int canbus_number, int bh_model, bool forcetorque, bool tactile);

    ~WamDriver();

    bool Init(const char *joint_calibration_file);

    void Pump(const ros::TimerEvent& e);
    
    bool Publish();

    void Update();

    bool AddTrajectory(Trajectory *traj, std::string &failure_reason);

    bool AddTrajectory(pr_msgs::AddTrajectory::Request &req,
                       pr_msgs::AddTrajectory::Response &res);
    bool AddTimedTrajectory(pr_msgs::AddTimedTrajectory::Request &req,
                       pr_msgs::AddTimedTrajectory::Response &res);
    bool DeleteTrajectory(pr_msgs::DeleteTrajectory::Request &req,
                          pr_msgs::DeleteTrajectory::Response &res);
    bool CancelAllTrajectories(pr_msgs::CancelAllTrajectories::Request &req,
                          pr_msgs::CancelAllTrajectories::Response &res);
    bool PauseTrajectory(pr_msgs::PauseTrajectory::Request &req,
                         pr_msgs::PauseTrajectory::Response &res);
    bool ReplaceTrajectory(pr_msgs::ReplaceTrajectory::Request &req,
                           pr_msgs::ReplaceTrajectory::Response &res);
    bool SetStiffness(pr_msgs::SetStiffness::Request &req,
                      pr_msgs::SetStiffness::Response &res);
    bool SetJointStiffness(pr_msgs::SetJointStiffness::Request &req,
                      pr_msgs::SetJointStiffness::Response &res);
    bool SetJointOffsets(pr_msgs::SetJointOffsets::Request &req,
                      pr_msgs::SetJointOffsets::Response &res);
    bool SetSpeed(pr_msgs::SetSpeed::Request &req,
                  pr_msgs::SetSpeed::Response &res);
    bool GetSpeed(pr_msgs::GetSpeed::Request &req,
		  pr_msgs::GetSpeed::Response &res);
    bool SetExtraMass(pr_msgs::SetExtraMass::Request &req,
		      pr_msgs::SetExtraMass::Response &res);
    bool SetStallSensitivity(pr_msgs::SetStallSensitivity::Request &req,
			     pr_msgs::SetStallSensitivity::Response &res);
    bool GetDOF(pr_msgs::GetDOF::Request &req,
                pr_msgs::GetDOF::Response &res);
    bool CalibrateJoints(pr_msgs::CalibrateJoints::Request &req,
			 pr_msgs::CalibrateJoints::Response &res);
    bool StepJoint(pr_msgs::StepJoint::Request &req,
		   pr_msgs::StepJoint::Response &res);
    bool SetGains(pr_msgs::SetGains::Request &req,
		  pr_msgs::SetGains::Response &res);
    bool ReloadPlugins(pr_msgs::Reset::Request &req,
		       pr_msgs::Reset::Response &res);
    bool SetForceInputThreshold(pr_msgs::SetForceInputThreshold::Request &req,
				pr_msgs::SetForceInputThreshold::Response &res);
    bool SetTactileInputThreshold(pr_msgs::SetTactileInputThreshold::Request &req,
				pr_msgs::SetTactileInputThreshold::Response &res);

    void AdvertiseAndSubscribe(ros::NodeHandle &n);


    pr_msgs::AddTrajectory::Response AddTrajectory(
						   pr_msgs::AddTrajectory::Request *);

    void update_xmission_ratio(const char *param_name, double &current_value, double nominal_value);
    void wamservo_callback(const boost::shared_ptr<const pr_msgs::Servo> &message);
    void MassProperties_callback(const boost::shared_ptr<const pr_msgs::MassProperties> &message);

    inline void SetModifiedJ1(bool mj1) {modified_j1 = mj1;}

#ifdef BUILD_FOR_SEA
    void wamjointtargets_callback(const boost::shared_ptr<const pr_msgs::IndexedJointValues> &message);

    void resetSeaCtrl();

    void wam_seactrl_settl_callback(const boost::shared_ptr<const pr_msgs::WamSetupSeaCtrl> &message);

    void publishCurrentTorqLimits();
    bool WamRequestSeaCtrlTorqLimit(pr_msgs::WamRequestSeaCtrlTorqLimit::Request &req,
                                    pr_msgs::WamRequestSeaCtrlTorqLimit::Response &res);

    void wam_seactrl_setkp_callback(const boost::shared_ptr<const pr_msgs::WamSetupSeaCtrl> &message);
    void publishCurrentKp();
    bool WamRequestSeaCtrlKp(pr_msgs::WamRequestSeaCtrlKp::Request &req,
                             pr_msgs::WamRequestSeaCtrlKp::Response &res);

    void wam_seactrl_setkd_callback(const boost::shared_ptr<const pr_msgs::WamSetupSeaCtrl> &message);
    void publishCurrentKd();
    bool WamRequestSeaCtrlKd(pr_msgs::WamRequestSeaCtrlKd::Request &req,
                             pr_msgs::WamRequestSeaCtrlKd::Response &res);

    void wam_seactrl_setki_callback(const boost::shared_ptr<const pr_msgs::WamSetupSeaCtrl> &message);
    void publishCurrentKi();
    bool WamRequestSeaCtrlKi(pr_msgs::WamRequestSeaCtrlKi::Request &req,
                             pr_msgs::WamRequestSeaCtrlKi::Response &res);

    void publishAllSeaSettings();
#endif


private:
    Trajectory *BuildTrajectory(pr_msgs::JointTraj &jt);
    pr_msgs::WAMState wamstate;
    pr_msgs::WAMInternals waminternals;
    pr_msgs::Servo servocmd;
    double gravity_comp_value;
    vector<double> max_joint_vel;
    double wamhome[8];
    double min_accel_time;
    double max_jerk;
    vector<double> joint_vel, joint_accel;

    bool discard_movements;

    // internal structures
    char *joint_calibration_file;
    unsigned int nJoints;
    int32_t puck_offsets[Joint::Jn+1];
    JointPos desiredJointPositions, vLastCommand;
    struct timeval trajstarttime;
    bool intraj;
    std::list<Trajectory *> trajectory_list;
    char last_trajectory_error[200];
    int BH_model; /// model number of the hand, either 260, 280, or 0 (no hand)
    bool ForceTorque; /// whether the Force/Torque sensor is installed
    bool Tactile;  /// whether the Tactile sensors are installed (280 hand only)
    bool log_controller_data;
    typedef pair<void *,bool (*)()> PluginPointers;
    std::vector<PluginPointers> loaded_plugins;

    // update internal structures
    void resetDesiredJointPositions(void);
    void resetTrajectory();
    void queueJointPositions(void);

    int QueueJointPositions();
    void calibrate_wam_mass_model();
    void calibrate_joint_angles();
    void write_pulse_data();
    void set_home_position();
    void start_control_loop();
    void stop_control_loop();
    bool verify_home_position();
    bool move_joint(int joint, double newpos, double velocity);
    bool move_until_stop(int joint, double stop, double limit, double velocity,
			 double &orig_joint_pos);

    int get_puck_offset(int puckid,int32_t *mech = NULL,int32_t *apout = NULL);
    void save_joint_offset(double jointval, double *offset);
    int get_joint_num();
    double get_nearest_joint_value(double jointval, double tolerance);
    void apply_joint_offsets(double *joint_offsets);
    TrajType ros2owd_traj (pr_msgs::JointTraj &jt);
    void get_transmission_ratios();
    void load_plugins(std::string plugin_list);
    void unload_plugins();

 public: // make this public so that it can be shared with BHD_280
    static CANbus *bus;
    static WAM *owam;
    bool running;

 private:
    boost::mutex queue_mutex;
    boost::mutex wscb_mutex;
    boost::mutex plugin_mutex;
    bool modified_j1;

    ros::Publisher
      pub_wamstate,
      pub_waminternals;

    ros::Subscriber
      sub_wamservo,
      sub_wam_joint_targets,
      sub_MassProperties;

    ros::ServiceServer 
      ss_AddTrajectory,
      ss_AddTimedTrajectory,
      ss_SetStiffness,
      ss_SetJointStiffness,
      ss_SetJointOffsets,
      ss_DeleteTrajectory, 
      ss_CancelAllTrajectories,
      ss_PauseTrajectory,
      ss_ReplaceTrajectory,
      ss_SetSpeed,
      ss_GetSpeed,
      ss_SetExtraMass,
      ss_SetStallSensitivity,
      ss_GetArmDOF,
      ss_CalibrateJoints,
      ss_StepJoint,
      ss_SetGains,
      ss_ReloadPlugins,
      ss_SetForceInputThreshold,
      ss_SetTactileInputThreshold;

    tf::TransformBroadcaster tf_broadcaster;
    btTransform wam_tf_base[7];
 
#ifdef BUILD_FOR_SEA
    ros::Publisher
      pub_wam_seactrl_curtl,
      pub_wam_seactrl_curkp,
      pub_wam_seactrl_curkd,
      pub_wam_seactrl_curki;

    ros::Subscriber 
      sub_wam_seactrl_settl,
      sub_wam_seactrl_setkp,
      sub_wam_seactrl_setkd,
      sub_wam_seactrl_setki;

    ros::ServiceServer
      ss_WamRequestSeaCtrlTorqLimit,
      ss_WamRequestSeaCtrlKp,
      ss_WamRequestSeaCtrlKd,
      ss_WamRequestSeaCtrlKi;

#endif // BUILD_FOR_SEA

    friend class PositionCommand;
    friend class TrajectoryCommand;
    friend class Trajectory;
};
};
#endif //  OPENWAMDRIVER_H
