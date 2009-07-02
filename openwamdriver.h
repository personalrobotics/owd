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
#include <ros/node.h>
#include <boost/thread/mutex.hpp>
#include <owd/AddTrajectory.h>
#include <owd/DeleteTrajectory.h>
#include <owd/PauseTrajectory.h>
#include <owd/ReplaceTrajectory.h>
#include <owd/SetExtraMass.h>
#include <owd/SetStiffness.h>
#include <owd/SetSpeed.h>
#include <owd/WAMState.h>
#include <owd/WAMInternals.h>
#include <owd/GetDOF.h>
#include <owd/Servo.h>

class WamDriver
{
public:
    
    WamDriver(const char *robotname);

    ~WamDriver();

    bool Init(const char *joint_calibration_file);
    
    bool Publish(ros::Node &n);

    void Update();

    bool AddTrajectory(owd::AddTrajectory::Request &req,
		       owd::AddTrajectory::Response &res);
    bool DeleteTrajectory(owd::DeleteTrajectory::Request &req,
			  owd::DeleteTrajectory::Response &res);
    bool PauseTrajectory(owd::PauseTrajectory::Request &req,
			 owd::PauseTrajectory::Response &res);
    bool ReplaceTrajectory(owd::ReplaceTrajectory::Request &req,
			   owd::ReplaceTrajectory::Response &res);
    bool SetExtraMass(owd::SetExtraMass::Request &req,
		      owd::SetExtraMass::Response &res);
    bool SetStiffness(owd::SetStiffness::Request &req,
		      owd::SetStiffness::Response &res);
    bool SetSpeed(owd::SetSpeed::Request &req,
		  owd::SetSpeed::Response &res);
    bool GetDOF(owd::GetDOF::Request &req,
		owd::GetDOF::Response &res);

    void wamservo_callback(void *message);

    std::string robotname;

private:
    Trajectory *BuildTrajectory(owd::JointTraj &jt);
    owd::WAMState wamstate;
    owd::Servo servocmd;
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
    bool move_until_stop(int joint, double stop, double limit, double velocity);

    int get_puck_offset(int puckid,long *mech = NULL,long *apout = NULL);
    void save_joint_offset(double jointval, double *offset);
    int get_joint_num();
    double get_nearest_joint_value(double jointval, double tolerance);
    void apply_joint_offsets(double *joint_offsets);
    TrajType ros2owd_traj (owd::JointTraj &jt);

    CANbus bus;
    WAM *owam;
    boost::mutex queue_mutex;
    boost::mutex wscb_mutex;

    friend class PositionCommand;
    friend class TrajectoryCommand;
};

#endif //  OPENWAMDRIVER_H
