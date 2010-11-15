/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openwam is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openwam is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Modified 2007-2010 by:
      Mike Vande Weghe <vandeweg@cmu.edu>
      Robotics Institute
      Carnegie Mellon University
*/

//LLL
//#define BUILD_FOR_SEA

#include <semaphore.h>
#include <list>

#include "CANbus.hh"
#include "Joint.hh"
#include "Motor.hh"
#include "ControlLoop.hh"
#include "Link.hh"
#include "Kinematics.hh"
#include "Dynamics.hh"

#include "PulseTraj.hh"
#include "SE3Traj.hh"
#include "SE3CtrlPD.hh"
#include "JointsTraj.hh"
#include "ParaJointTraj.hh"
#include "MacJointTraj.hh"
#include "Sigmoid.hh"
#include "JointCtrlPID.hh"
#include <pr_msgs/PIDgains.h>

//LLL

#ifdef BUILD_FOR_SEA
#include "positionInterface/SmoothArm.h"
#include "JointCtrlSea.hh"
#endif // BUILD_FOR_SEA

#include "DataRecorder.cc"

#ifndef __WAM_H__
#define __WAM_H__

using namespace std;

class WAMstats{
public:
  double trajtime;
  double jsctrltime;
  int safetycount;
  double loopread;
  double loopctrl;
  double loopsend;
  double looptime;
  double slowcount;
  double slowavg;
  double slowmax;
  double slowreadtime;
  double slowctrltime;
  double slowsendtime;
  double dyntime;
  int hitorquecount[7];
  double hitorqueavg[7];

  WAMstats() : trajtime(0.0f),
               jsctrltime(0.0f),
               safetycount(0),
               loopread(0.0f),
               loopctrl(0.0f),
               loopsend(0.0f),
               looptime(0.0f),
               slowcount(0.0f),
               slowavg(0.0f),
	       slowmax(0.0f),
               slowreadtime(0.0f),
               slowctrltime(0.0f),
               slowsendtime(0.0f),
               dyntime(0.0f)
  {
    for (unsigned int i=0; i<7; ++i) {
      hitorquecount[i]=0;
      hitorqueavg[i]=0;
    }
  }

  void rosprint(int rc) const;
  
};


class WAM{
public:
  static const double mN1 = 41.782;  //joint ratios
  static const double mN2 = 27.836;
  static const double mN3 = 27.836;
  static const double mN4 = 17.860;
  static const double mN5 =  9.68163;
  static const double mN6 =  9.68163;
  static const double mN7 = 14.962;
  static const double mn3 = 1.68;
  static const double mn6 = 1.00;

  static const double M2_MM2 = 1.0/1000000;


#ifdef OWD_RT
  RT_MUTEX rt_mutex;
#else // ! OWD_RT
  pthread_mutex_t mutex;                   // Use to lock the WAM
#endif // ! OWD_RT
  Joint joints[Joint::Jn+1];               // Array of joints
  Motor motors[Motor::Mn+1];               // Array of motors
  Link links[Link::Ln+1];                  // Array of links
  Link sim_links[Link::Ln+1];                  // Array of links (simulated)
  Link L7_with_hand, L7_without_hand,
    L4_without_wrist_with_hand, L4_without_wrist_without_hand;
  double heldPositions[Joint::Jn+1];
  bool suppress_controller[Joint::Jn+1];    // flag to disable PID control
  bool check_safety_torques;
  double pid_torq[Joint::Jn+1];
  double sim_torq[Joint::Jn+1];
 
  SE3 E0n;                                 // forward kinematics transformation

  SE3Traj     *se3traj;                    // Cartesian trajectory
  SE3CtrlPD    se3ctrl;                    // basic Cartesian controller
  Trajectory *jointstraj;
  PulseTraj   *pulsetraj;                  // trajectory of joint acceleration pulses
  int safetytorquecount[7];
  double safetytorquesum[7];


#ifdef BUILD_FOR_SEA
  JointCtrlSea jointsctrl[Joint::Jn+1];    // joint controllers
  void loadSeaParams();
#else
  JointCtrlPID jointsctrl[Joint::Jn+1];    // joint controllers
#endif

  bool rec;                                // Are we recording data
  bool wsdyn;                              // Is the WS dynamics turned on?
  bool jsdyn;                              // Is the JS dynamics turned on?
  bool holdpos;                            // Should we hold position?
  bool exit_on_pendant_press;              // once running, should we exit if the user
                                           // presses e-stop or shift-idle?
  double pid_sum;
  int pid_count;
  bool safety_hold;

  WAMstats stats;
  inline void rosprint_stats() { stats.rosprint(recorder.count); bus->rosprint_stats();}

  int recv_mpos();
  int send_mtrq();

  void mpos2jpos();
  void jpos2mpos();   
  void jtrq2mtrq();

  R6 WSControl(double dt);
  void JSControl(double qdd[Joint::Jn+1], double dt);
  void newJSControl_rt(double q_target[], double q[], double dt, double pid_torq[]); // Mike

  void lock(const char *name="unspecified");
  bool lock_rt(const char *name="unspecified");
  void unlock(const char *name=NULL);

  CANbus* bus;                             // pointer to the CAN bus
  ControlLoop ctrl_loop;            // control loop object

  // LLL a beta version of joint targs
  //JointTargets jointTargs;
  // LLL a beta version of position smoother 
#ifdef BUILD_FOR_SEA
  SmoothArm posSmoother;
#endif // BUILD_FOR_SEA

  WAM(CANbus* cb);
  ~WAM();
 
  int init();                       // initialise the WAM
  void dump();                      // print information of the WAM

  void printjpos();                 // print joints positions
  void printmtrq();                 // print motor torques

  int start();                      // start the control loop
  void stop();                      // stop the control loop
  void newcontrol_rt(double dt);          // main control function
  bool safety_torques_exceeded(double t[]); // check pid torqs against thresholds

  void set_hand(bool h);     // change the hand mass
  int  set_targ_jpos(double* pos);          // set the target joint positions online 
  int  set_jpos(double pos[Joint::Jn+1]);   // set the joint positions offline
  bool set_gains(unsigned int joint, pr_msgs::PIDgains &gains);
  bool get_gains(std::vector<pr_msgs::PIDgains> &gains);

  void get_current_data(double *pos, double *trq, 
			double *nettrq, double *sim_torq=NULL);
                  // get the joint positions, torques, net torques, and
                  // simulated torques (any pointer can be NULL)
  void get_jtrq(double trq[Joint::Jn+1]);   // get the joint torques
  void get_net_jtrq(double trq[Joint::Jn+1]); // get the net joint torques

  int hold_position(double jval[] = NULL, bool grab_lock=true); // call to hold current position.
                                            // returns position if pointer is supplied.
  void release_position(bool grab_lock=true
);          // call to release current position from PID control
  void set_stiffness(float stiffness); // how "hard" to hold position
  void move_sigmoid(const SE3& E0ns);
  void move_sigmoid(const double q[Joint::Jn+1]);
  void move_trapezoid(const SE3& E0ns);
  void move_trapezoid(const double q[Joint::Jn+1]);

  int run_trajectory(Trajectory *traj);  // start a trajectory
  int pause_trajectory();
  int resume_trajectory();
  int cancel_trajectory();

  bool& wsdynamics(){return wsdyn;} // use to set the dynamics
  bool& jsdynamics(){return jsdyn;} // use to set the dynamics
  
  bool& record(){return rec;}       // use to set the recording
  SE3 FK(){  SE3 e0n; lock();   e0n=E0n;   unlock();   return e0n;  }

  friend ostream& operator << (ostream& s, WAM& wam);

  typedef enum {MOTORS_OFF,MOTORS_IDLE,MOTORS_ACTIVE} motor_state_t;
  motor_state_t motor_state;

  float stiffness;
  DataRecorder<double> recorder;

};

#endif

