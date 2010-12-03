/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Corporation *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

#include <list>

#ifdef WRIST
#define NUMDOF 7
#else
#define NUMDOF 4
#endif

class ScreenBuf {
public:
    typedef enum {
        NONE,
        FRICTION_CALIB,
        ACCEL_PULSE,
        TRAJ_RECORD } modetype;
        
    static long motor_torqs[7];
    static int active_link;
    static double link_mass;
    static double link_COG[3];
    static double link_mass_moment[7];
    static double link_inertia[6];
    static double link_effective_arm;
    static double ccg_link_accel;
    static double ccg_torque;
    static double joint_pos[7];
    static double target_jpos[7];
    static double joint_ideal_vel[7];
    static double joint_torqs[7];
    static double dyn_torqs[7];
    static double pid_torqs[7];
    static double friction_torq[7];
    static double joint_PIDs[7];
    static double ws_torqs[7];
    static double jacobian[6][7];
    static double test[5][7];
    static const char *pulse_msg;
    static const char *status_msg;
    static const char *traj_msg;
    static double pulse_accel;
    static int pulse_dur;
    static bool pulse_pair;
    static double friction_pulse_velocity;
    static std::list<const char *> scrolling_log;
    static const unsigned int scrolling_log_max=30;
    static modetype mode;
    static int savedpoints;
    static bool redraw_screen;

    static void clear_and_print();
    static void log_message(const char *msg, ...);
};


