/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Corporation *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

#include "Joint.hh"
#include <math.h>
#include <vector>

#ifndef __PULSETRAJ_HH__
#define __PULSETRAJ_HH__

using namespace std;

class PulseTraj{

public:
    //variables for PulseControl() function, used to apply a pulse of acceleration at a given joint, the pulse can be part positive and part negative depending on the pulse_switch_ticks variable
    static const double _pulse_accel_limit = 50;   //maximum magnitude of acceleration       

    PulseTraj(double pulse_accel_in, int pulse_duration_ticks_in, int pulse_switch_ticks_in,int pulse_joint_i_in);
    ~PulseTraj(){}

    bool PulseOk(){return _pulseok;}
    bool GetMotion(double qd[], double qdd[]);
    bool RecordPositions(double q[]);
    bool RecordTorques(double trq[]);
    bool done;                         //is the pulse finished?
    bool record;                       // are we recording?
    std::vector< std::vector<double> > posvals,velvals,accelvals,trqvals; // fields for recording variable of the arm as it executes a pulse
    double pulse_accel;                   //acceleration to apply
    int pulse_duration_ticks;                //total length of an acceleration pulse
    int pulse_pre_record_ticks;
    int pulse_post_record_ticks;

private:
    bool _pulseok;                     //is the pulse well-formatted
    int _pulse_counter;                //internal variable to keep track of ticks
    int _record_counter;

    int pulse_switch_ticks;           //when does the pulse switch from + to - magnitude (if 0, magnitude is constant)
    int pulse_joint_i;                //index of the joint to which the pulse is being applied


 
};

#endif

