#ifndef OPENWAMDRIVER_H
#define OPENWAMDRIVER_H

#include <TrajType.hh>
#include "openwam/Trajectory.hh"
#include "openwam/Joint.hh"
#include "CANbus.hh"
#include "WAM.hh"
#include <list>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <pr_msgs/AddTrajectory.h>
#include <pr_msgs/DeleteTrajectory.h>
#include <pr_msgs/CancelAllTrajectories.h>
#include <pr_msgs/PauseTrajectory.h>
#include <pr_msgs/ReplaceTrajectory.h>
#include <pr_msgs/MassProperties.h>
#include <pr_msgs/SetStiffness.h>
#include <pr_msgs/SetSpeed.h>
#include <pr_msgs/WAMState.h>
#include <pr_msgs/WAMInternals.h>
#include <pr_msgs/GetDOF.h>
#include <pr_msgs/Servo.h>
#include <owd/CalibrateJoints.h>
#include <owd/StepJoint.h>
#include <tf/transform_broadcaster.h>

#ifdef BUILD_FOR_SEA
  #include <pr_msgs/WamRequestSeaCtrlTorqLimit.h>
  #include <pr_msgs/WamRequestSeaCtrlKp.h>
  #include <pr_msgs/WamRequestSeaCtrlKd.h>
  #include <pr_msgs/WamRequestSeaCtrlKi.h>
#endif

class WamDriver
{
public:
    
    WamDriver(const char *robotname);

    ~WamDriver();

    bool Init(const char *joint_calibration_file);

    void Pump(const ros::TimerEvent& e);
    
    bool Publish();

    void Update();

    bool AddTrajectory(pr_msgs::AddTrajectory::Request &req,
                       pr_msgs::AddTrajectory::Response &res);
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
    bool SetSpeed(pr_msgs::SetSpeed::Request &req,
                  pr_msgs::SetSpeed::Response &res);
    bool GetDOF(pr_msgs::GetDOF::Request &req,
                pr_msgs::GetDOF::Response &res);
    bool CalibrateJoints(owd::CalibrateJoints::Request &req,
			 owd::CalibrateJoints::Response &res);
    bool StepJoint(owd::StepJoint::Request &req,
			      owd::StepJoint::Response &res);

    void AdvertiseAndSubscribe(ros::NodeHandle &n);

    void wamservo_callback(const boost::shared_ptr<const pr_msgs::Servo> &message);
    void MassProperties_callback(const boost::shared_ptr<const pr_msgs::MassProperties> &message);

    // LLL
    void wamjointtargets_callback(const boost::shared_ptr<const pr_msgs::IndexedJointValues> &message);

#ifdef BUILD_FOR_SEA
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




    std::string robotname;

private:
    Trajectory *BuildTrajectory(pr_msgs::JointTraj &jt);
    pr_msgs::WAMState wamstate;
    pr_msgs::WAMInternals waminternals;
    pr_msgs::Servo servocmd;
    double gravity_comp_value;
    vector<double> max_joint_vel;
    double min_accel_time;
    double max_jerk;
    vector<double> joint_vel, joint_accel;
    //    static const float lowerjointlimit_deg[] 
    //          = {-150,-113,-157, -50,-275,-90,-172};
    //    static const float upperjointlimit_deg[] 
    //          = { 150, 113, 157, 180,  75, 90, 172};

    bool discard_movements;

    int cmdnum; // command counter

    // internal structures
    char *joint_calibration_file;
    unsigned int nJoints;
    long puck_offsets[Joint::Jn+1];
    JointPos desiredJointPositions, vLastCommand;
    struct timeval trajstarttime;
    bool intraj;
    std::list<Trajectory *> trajectory_list;

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

    int get_puck_offset(int puckid,long *mech = NULL,long *apout = NULL);
    void save_joint_offset(double jointval, double *offset);
    int get_joint_num();
    double get_nearest_joint_value(double jointval, double tolerance);
    void apply_joint_offsets(double *joint_offsets);
    TrajType ros2owd_traj (pr_msgs::JointTraj &jt);

    CANbus bus;
    WAM *owam;
    boost::mutex queue_mutex;
    boost::mutex wscb_mutex;

    ros::Publisher
      pub_wamstate,
      pub_waminternals;

    ros::Subscriber
      sub_wamservo,
      sub_wam_joint_targets,
      sub_MassProperties;

    ros::ServiceServer 
      ss_AddTrajectory,
      ss_SetStiffness,
      ss_DeleteTrajectory, 
      ss_CancelAllTrajectories,
      ss_PauseTrajectory,
      ss_ReplaceTrajectory,
      ss_SetSpeed,
      ss_GetArmDOF,
      ss_CalibrateJoints,
      ss_StepJoint;

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
};

#endif //  OPENWAMDRIVER_H
