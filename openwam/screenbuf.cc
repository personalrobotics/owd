/***********************************************************************
 *                                                                     *
 * Copyright 2010 Carnegie Mellon University and Intel Labs Pittsburgh *
 * Author: Mike Vande Weghe <vandeweg@cmu.edu>                         *
 *                                                                     *
 ***********************************************************************/

#include "screenbuf.hh"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern double fv[NUMDOF+1];
extern double fs[NUMDOF+1];

void ScreenBuf::clear_and_print() {
    if (!redraw_screen) {
        return;
    }
    printf("%c[H%c[2J",27,27);// clear screen
    switch (mode) {
    case NONE:
        printf("Calibration mode: none\n");
        break;
    case FRICTION_CALIB:
        printf("Calibration mode: FRICTION\n");
        break;
    case ACCEL_PULSE:
        printf("Calibration mode: ACCEL_PULSE\n");
        break;
    case TRAJ_RECORD:
        printf("Calibration mode: TRAJECTORY RECORDING\n");
        break;
    }
    printf("Active Link: %d\n",active_link);
    printf("  Mass: %1.2f  COG: %2.2f %2.2f %2.2f\n",link_mass,link_COG[0],link_COG[1],link_COG[2]);
    printf("  Inertia: %1.5f %1.5f %1.5f %1.5f %1.5f %1.5f\n",link_inertia[0], link_inertia[1], link_inertia[2], link_inertia[3], link_inertia[4],link_inertia[5]);

    printf("  Effective arm: %- 2.2f\n",link_effective_arm);
    printf("  CCG: accel in %2.2f   torque out %2.2f\n",ccg_link_accel, ccg_torque);
    printf("\n");
#ifdef WRIST
    printf("Joint          1     2     3     4     5     6     7\njnt pos    ");
#else
    printf("Joint          1     2     3     4\njnt pos    ");
#endif
    
    for (int i = 0; i < NUMDOF; ++i) {
        printf("% 3.1f ",joint_pos[i]);
    }
    printf("\ntar pos    ");
    for (int i = 0; i < NUMDOF; ++i) {
        printf("% 3.1f ",target_jpos[i]);
    }
    printf("\njnt vel    ");
    for (int i = 0; i < NUMDOF; ++i) {
        printf("% 3.1f ",joint_ideal_vel[i]);
    }
    printf("\ntot torq   ");
    for (int i = 0; i < NUMDOF; ++i) {
        printf("% 3.1f ",joint_torqs[i]);
    }
    printf("\ndyn torq   ");
    for (int i = 0; i < NUMDOF; ++i) {
        printf("% 3.1f ",dyn_torqs[i]);
    }
    printf("\npid torq   ");
    for (int i = 0; i < NUMDOF; ++i) {
        printf("% 3.1f ",pid_torqs[i]);
    }
    printf("\nfri torq   ");
    for (int i = 0; i < NUMDOF; ++i) {
        printf("% 3.1f ",friction_torq[i]);
    }
    printf("\nmot torq   ");
    for (int i = 0; i < NUMDOF; ++i) {
        printf("% 5ld ",motor_torqs[i]);
    }


    
    /* //(DDB)
    printf("\nws torq    ");
    for (int i = 0; i < NUMDOF; ++i) {
        printf("% 3.1f ",ws_torqs[i]);
    }

    printf("\njacobian   ");
    for(int i = 0; i< 6; i++)
    {
        printf("\n           ");
        for (int j = 0; j < NUMDOF;j++) {
            printf("% 3.3f ",jacobian[i][j]);
        }

    }
    printf("\n");


    printf("\ntest   ");
    for(int i = 0; i< 5; i++)
    {
        printf("\n           ");
        for (int j = 0; j < NUMDOF;j++) {
            printf("% 3.3f ",test[i][j]);
        }

    }
    printf("\n");
    */

    printf("\n");
    switch (mode) {
    case ACCEL_PULSE:
        printf("Pulse magnitude (M/m) %2.1f  duration (D/d) %d\n",pulse_accel,pulse_dur);
        printf("Paired accel/decel (/): %s\n",pulse_pair?"yes":"no");
        printf("\n[ret] send pulse    [w]rite to file\n");
        break;
    case FRICTION_CALIB:
        printf("Friction: static (S/s) %2.1f  dynamic (D/d) %2.1f velocity (V/v) %2.2f\n",
               fs[active_link],
               fv[active_link],
               friction_pulse_velocity);
        printf("[ret] run joint traj\n");
        break;
    case TRAJ_RECORD:
        printf("Saved points: %d   [ret] Save   [w] write to file\n",savedpoints);
        break;
    }
    printf("\n");
    printf("%s\n",pulse_msg);
    printf("%s\n",traj_msg);
    printf("%s\n\n",status_msg);
    printf("[H]old un[h]old [T]raj record [F]riction calib [A]ccel pulse  [N]ormal mode [q]uit\n");
    printf("\n===============================================================================\n");
    for (std::list<const char *>::iterator it=scrolling_log.begin(); it!=scrolling_log.end(); ++it) {
        printf("%s\n",*it);
    }
    return;
}

void ScreenBuf::log_message(const char *msg, ...) {
    char tmpstr[500];
	va_list argp;
    va_start(argp, msg);
    // build the message
    vsprintf(tmpstr, msg, argp);
    va_end(argp);

    // break it into individual lines and add them to the scrolling log
    char *strtokptr;
    char *single_line = strtok_r(tmpstr,"\n",&strtokptr);
    while (single_line) {
        if (redraw_screen) {
            if (scrolling_log.size() == scrolling_log_max) {
                delete scrolling_log.front();
                scrolling_log.pop_front();
            }
            scrolling_log.push_back(strdup(single_line));
        } else {
            printf("%s\n",single_line);
        }
        // get the next line
        single_line = strtok_r(NULL,"\n",&strtokptr);
    }
}

long ScreenBuf::motor_torqs[7]={0,0,0,0,0,0,0};  
int ScreenBuf::active_link =1;  
double ScreenBuf::link_mass=0.0;  
double ScreenBuf::link_COG[3]={0.0,0.0,0.0};  
double ScreenBuf::link_mass_moment[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double ScreenBuf::link_inertia[6]={0.0,0.0,0.0,0.0,0.0,0.0};  
double ScreenBuf::link_effective_arm=0.0;  
double ScreenBuf::ccg_link_accel=0.0;  
double ScreenBuf::ccg_torque=0.0;  
double ScreenBuf::joint_pos[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};  
double ScreenBuf::target_jpos[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};  
double ScreenBuf::joint_ideal_vel[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};  
double ScreenBuf::joint_torqs[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};  
double ScreenBuf::dyn_torqs[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};  
double ScreenBuf::pid_torqs[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};  
double ScreenBuf::friction_torq[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};  
double ScreenBuf::joint_PIDs[7] ={0.0,0.0,0.0,0.0,0.0,0.0,0.0};  
double ScreenBuf::ws_torqs[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};  
double ScreenBuf::jacobian[6][7]={{0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};  
double ScreenBuf::test[5][7] = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};  
const char *ScreenBuf::pulse_msg="";  
const char *ScreenBuf::traj_msg="";  
const char *ScreenBuf::status_msg="";  
double ScreenBuf::pulse_accel=0.0;  
int ScreenBuf::pulse_dur=0;  
bool ScreenBuf::pulse_pair = true;
std::list<const char *> ScreenBuf::scrolling_log;
double ScreenBuf::friction_pulse_velocity=0.0f;
ScreenBuf::modetype ScreenBuf::mode = ScreenBuf::NONE;
int ScreenBuf::savedpoints=0;
bool ScreenBuf::redraw_screen=true;
