/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openman is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openman is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "WAM.hh"
#include <ros/ros.h>
#include <stdio.h>

static void control_loop_rt(void* argv);

extern int MODE;  // puck parameter

/*
 * Create a new WAM that uses the CAN bus cb;
 */

WAM::WAM(CANbus* cb) :
    check_safety_torques(true),rec(false),wsdyn(false),jsdyn(false),
    holdpos(false),exit_on_pendant_press(false),pid_sum(0.0f), 
    pid_count(0),safety_hold(false),bus(cb),ctrl_loop(cb->id, &control_loop_rt, this),
    motor_state(MOTORS_OFF), stiffness(1.0), recorder(50000)
{

#if defined( OWD_RT ) && ! defined ( OWDSIM )
  rt_mutex_create(&rt_mutex,"WAM_CC");
#else
  pthread_mutex_init(&mutex, NULL);
#endif

  se3traj = NULL;
  jointstraj = NULL;
  pulsetraj = NULL;
  for(int i = Joint::J1; i<=Joint::Jn; i++) {
    heldPositions[i] = 0;
    suppress_controller[i]=false;
  }

  
  links[Link::L0]=Link( DH(  0.0000,   0.0000,   0.0000,   0.0000), 0.0000, 
                        R3(  0.0000,   0.0000,   0.0000), 
                        Inertia(0,0,0,0,0,0));

  links[Link::L1]=Link( DH( -M_PI_2,   0.0000,   0.0000,   0.0000), 8.3936,
                         R3(  0.3506, 132.6795,   0.6286)*0.001,
                        Inertia(95157.4294,   246.1404,    -95.0183,
                                92032.3524,  -962.6725,  59290.5997, M2_MM2) );
            
  links[Link::L2]=Link( DH(  M_PI_2,   0.0000,   0.0000,   0.0000), 4.8487,
                         R3( -0.2230, -21.3924,  13.3754)*0.001, 
                        Inertia(29326.8098,   -43.3994,   -129.2942,
                                20781.5826,  1348.6924,  22807.3271, M2_MM2) );

  links[Link::L3]=Link( DH( -M_PI_2,   0.0450,   0.5500,   0.0000), 1.7251,
                         R3(-38.7565, 217.9078,   0.0252)*0.001, 
                        Inertia(56662.2970, -2321.6892,      8.2125,
                                   3158.0509,   -16.6307,  56806.6024, M2_MM2) );

#ifdef WRIST
#warning "COMPILING WRIST"

  /* newer numbers from Barrett's Oct 2007 WAM_MassParams_AA-01.pdf */
  links[Link::L4]=Link( DH(  M_PI_2,  -0.0450,   0.0000,   0.0000), 2.40016804,
                        R3(  0.00498512,-0.00022942,0.13271662),
                        Inertia(0.01491672,    0.00001741,  -0.00150604,
                        0.01482922,    -0.00002109,   0.00294463, 1) );

  /* March 2009, not working so great
  links[Link::L4]=Link( DH(  M_PI_2,  -0.0450,   0.0000,   0.0000), 2.17266212,
                        R3(  0.00553408,0.00006822,0.11927695),
                        Inertia(0.01067491,    0.00004503,  -0.00135557,
                                0.01058659,    -0.00011002,   0.00282036, 1) );  */
  

  links[Link::L5]=Link( DH( -M_PI_2,   0.0000,   0.3000,   0.0000), 0.35655692,
                        R3(  0.00005483,0.02886286,0.00148493), 
                        Inertia(  0.00037112,    -0.00000008,     -0.00000003,
                                  0.00019434,    -0.00001613,    0.00038209, 1) );

  links[Link::L6]=Link( DH(  M_PI_2,   0.0000,   0.0000,   0.0000), 0.40915886,
                        R3( -0.00005923,-0.01686123,0.02419052),
                        Inertia(  0.00054889,     0.00000019,     -0.00000010,
                                  0.00023846,   -0.00004430,    0.00045133, 1) );
/*old wrist
  links[Link::L4]=Link( DH(  M_PI_2,  -0.0450,   0.0000,   0.0000), 2.1824,
                        R3(  6.2895,  -0.0010, 111.0633)*0.001,
                        Inertia(10065.3990,    14.6007,  -1392.4965,
                                10000.5377,    29.4814,   2838.9554, M2_MM2) );

  links[Link::L5]=Link( DH( -M_PI_2,   0.0000,   0.3000,   0.0000), 0.4067,
                        R3(  0.0584,  28.3754,   0.1902)*0.001, 
                        Inertia(  321.0141,    -0.0667,     -0.0079,
                                  172.3637,    -2.4724,    350.6782, M2_MM2) );

  links[Link::L6]=Link( DH(  M_PI_2,   0.0000,   0.0000,   0.0000), 0.5278,
                        R3( -0.0311, -14.8635,  25.6326)*0.001,
                        Inertia(  604.1921,     0.0825,     -0.1896,
                                  269.3020,   -62.3326,    507.9036, M2_MM2) );
*/
#else

  // NO WRIST
#ifdef BH8
  // my guess is that the HAND COG is x=-.045m,y=0,z=.410m from J4
  // hand mass with camera is 1.3kg
  // new COG: X=(.01096*1.065 + .045*1.3)/(1.065+1.3) = .0297
  //          Z=(.14054*1.065 + .410*1.3)/(1.065+1.3) = .2887
  links[Link::L4]=Link( DH(  M_PI_2,  -0.0450,   0.0000,   0.0000), 2.36513649,
                        R3(  0.0297,0.00002567,0.2887),
                        Inertia(0.01848577,    0.00000219,  -0.00160868,
                                0.01891658,    -0.00000515,   0.00197517, 1) );

#else
  links[Link::L4]=Link( DH(  M_PI_2,  -0.0450,   0.0000,   0.0000), 1.06513649,
                        R3(  0.01095471,0.00002567,0.14053900),
                        Inertia(0.01848577,    0.00000219,  -0.00160868,
                                0.01891658,    -0.00000515,   0.00197517, 1) );
  /* newer numbers from Barrett's Oct 2007 WAM_MassParams_AA-01.pdf 
       Lxx = 0.01848577        Lxy = 0.00000219      Lxz = ­0.00160868 
       Lyx = 0.00000219        Lyy = 0.01891658      Lyz = 0.00000515 
       Lzx = ­0.00160868       Lzy = 0.00000515      Lzz = 0.00197517 
  */

#endif


#endif // WRIST

#ifdef BH8
#warning "COMPILING HAND"
  //
  // FAT WARNING
  // I had the mass and the center of mass from old files from Barrett.
  // I figured the inertia tensor by modeling a cylinder of 13cm long and
  // 8.5cm diameter, with a uniform density computed from the mass and the
  // cylinder's volume.
  // The kinematics remains the one of J7.

  //new inertias
  links[Link::L7]=Link( DH(  0.0000,   0.0000,   0.0620,   0.0000), 1.3754270,
                        R3(  0.0000,   0.0000,  45.0000)*0.001,
                        Inertia(   2558.1007,   0.0000,      0.0000,
                                   2558.1007,   0.0000,   1242.1825, M2_MM2) );
 

//                        Inertia(   2231.8750,   0.0000,      0.0000,                                   2231.8750,   0.0000,   1083.7500, M2_MM2) );
  /*
  links[Link::L7]=Link( DH(  0.0000,   0.0000,   0.0620,   0.0000), 1.3000,
                        R3( -30.0000,   0.0000,  45.0000)*0.001,
                        Inertia(   2231.8750,   0.0000,      0.0000,
                                   2231.8750,   0.0000,   1083.7500, M2_MM2) );
  */
#else

  links[Link::L7]=Link( DH(  0.0000,   0.0000,   0.0609,   0.0000), 0.07548270,
                        R3( 0.00014836,0.00007252,-0.00335185),
                        Inertia(   0.00003911,    0.00000019,      0.0000,
                                   0.00003877,     0.00000,     0.00007614, M2_MM2) );
            /*from wam7hand.conf:
            <0.00003911,0.00000019,0.00000000>,
            <0.00000019,0.00003877,0.00000000>,
            <0.00000000,0.00000000,0.00007614>*/


/*old wrist
  links[Link::L7]=Link( DH(  0.0000,   0.0000,   0.0620,   0.0000), 0.0557,
                        R3( -0.0001,  -0.1823,-284.7431)*0.001,
                        Inertia(   21.4102,    -0.0001,      0.0000,
                                   21.6157,     0.0040,     42.1975, M2_MM2) );
*/

#endif

  for (unsigned int i=Link::L1; i<=Link::Ln; ++i) {
    sim_links[i] = links[i];
  }

  for (unsigned int i=0; i<7; ++i) {
    safetytorquecount[i]=safetytorquesum[i]=0;
  }
    
}

int WAM::init(){
  for(int j=Joint::J1; j<=Joint::Jn; j++) {
    joints[j].ID = j;
  }
  
  motors[1].ID=1; motors[1].puckI_per_Nm = 2472;
  motors[2].ID=2; motors[2].puckI_per_Nm = 2472;
  motors[3].ID=3; motors[3].puckI_per_Nm = 2472;
  motors[4].ID=4; motors[4].puckI_per_Nm = 2472;
  if (Motor::Mn > 4) {
    motors[5].ID=5; motors[5].puckI_per_Nm = 6105;
    motors[6].ID=6; motors[6].puckI_per_Nm = 6105;
    motors[7].ID=7; motors[7].puckI_per_Nm = 21400;
  }
  if (Motor::Mn > 7) {
    motors[8].ID=8; motors[8].puckI_per_Nm = 500;
    motors[9].ID=9; motors[9].puckI_per_Nm = 500;
    motors[10].ID=10; motors[10].puckI_per_Nm = 500;
    motors[11].ID=11; motors[11].puckI_per_Nm = 500;
  }    

  //                      Kp    Kd    Ki
  jointsctrl[1].set_gains( 250, 5.0, 2.5);
  jointsctrl[2].set_gains(1250, 10.0, 5.0);
  jointsctrl[3].set_gains( 200, 5.0, 2.5);
  jointsctrl[4].set_gains( 250,  1.25, 0.5);
  if (Joint::Jn > 4) {
  //                        Kp    Kd    Ki
    jointsctrl[5].set_gains(  20,  0.25, 0.5);
    jointsctrl[6].set_gains(  20,  0.25, 0.5);
    jointsctrl[7].set_gains(  8,  0.08, 0.1);
  }
  if (Joint::Jn > 7) {
    // set 280 Hand gains here
  }

#ifdef BUILD_FOR_SEA
  loadSeaParams();
#endif
  return OW_SUCCESS;
}

bool WAM::set_gains(unsigned int joint,
		    pr_msgs::PIDgains &gains) {
  if ((joint < Joint::J1) || (joint >> Joint::Jn)) {
    return false;
  }
  jointsctrl[joint].set_gains(gains.kp,gains.kd,gains.ki);
  return true;
}

bool WAM::get_gains(std::vector<pr_msgs::PIDgains > &gains) {
  gains.resize(Joint::Jn);
  for (unsigned int j=1; j<=Joint::Jn; ++j) {
    jointsctrl[j].get_gains(gains[j-1].kp, gains[j-1].kd, gains[j-1].ki);
  }
  return true;
}

#ifdef BUILD_FOR_SEA
void WAM::loadSeaParams(){
  // retrieve sea from ros param server, if available
  ros::Node* node = ros::Node::instance();

  assert(Joint::Jn == 7);

  // assume 7 joints
  assert(Joint::Jn == 7);

  // safe settings (used when there is no torque limit)
  // j1
  double kpSafe, kiSafe, kdSafe;
  node->param(std::string("owd/JointCtrlSea/j1Kp_safe"), kpSafe, 1800.0);
  node->param(std::string("owd/JointCtrlSea/j1Ki_safe"), kiSafe, 0.0);
  node->param(std::string("owd/JointCtrlSea/j1Kd_safe"), kdSafe, 10.0);
  ROS_INFO("j1 safe settings kp:%f, ki:%f, kd:%f", kpSafe, kiSafe, kdSafe);
  jointsctrl[1].specifySafeSettings(kpSafe, kiSafe, kdSafe);
  // j2
  node->param(std::string("owd/JointCtrlSea/j2Kp_safe"), kpSafe, 1800.0);
  node->param(std::string("owd/JointCtrlSea/j2Ki_safe"), kiSafe, 0.0);
  node->param(std::string("owd/JointCtrlSea/j2Kd_safe"), kdSafe, 10.0);
  ROS_INFO("j2 safe settings kp:%f, ki:%f, kd:%f", kpSafe, kiSafe, kdSafe);
  jointsctrl[2].specifySafeSettings(kpSafe, kiSafe, kdSafe);
  // j3
  node->param(std::string("owd/JointCtrlSea/j3Kp_safe"), kpSafe, 2000.0);
  node->param(std::string("owd/JointCtrlSea/j3Ki_safe"), kiSafe, 0.0);
  node->param(std::string("owd/JointCtrlSea/j3Kd_safe"), kdSafe, 20.0);
  ROS_INFO("j3 safe settings kp:%f, ki:%f, kd:%f", kpSafe, kiSafe, kdSafe);
  jointsctrl[3].specifySafeSettings(kpSafe, kiSafe, kdSafe);
  // j4
  node->param(std::string("owd/JointCtrlSea/j4Kp_safe"), kpSafe, 1200.0);
  node->param(std::string("owd/JointCtrlSea/j4Ki_safe"), kiSafe, 0.0);
  node->param(std::string("owd/JointCtrlSea/j4Kd_safe"), kdSafe, 10.0);
  ROS_INFO("j4 safe settings kp:%f, ki:%f, kd:%f", kpSafe, kiSafe, kdSafe);
  jointsctrl[4].specifySafeSettings(kpSafe, kiSafe, kdSafe);
  // j5
  node->param(std::string("owd/JointCtrlSea/j5Kp_safe"), kpSafe, 300.0);
  node->param(std::string("owd/JointCtrlSea/j5Ki_safe"), kiSafe, 0.0);
  node->param(std::string("owd/JointCtrlSea/j5Kd_safe"), kdSafe, 5.0);
  ROS_INFO("j5 safe settings kp:%f, ki:%f, kd:%f", kpSafe, kiSafe, kdSafe);
  jointsctrl[5].specifySafeSettings(kpSafe, kiSafe, kdSafe);
  // j6
  node->param(std::string("owd/JointCtrlSea/j6Kp_safe"), kpSafe, 10.0);
  node->param(std::string("owd/JointCtrlSea/j6Ki_safe"), kiSafe, 0.0);
  node->param(std::string("owd/JointCtrlSea/j6Kd_safe"), kdSafe, 1.5);
  ROS_INFO("j6 safe settings kp:%f, ki:%f, kd:%f", kpSafe, kiSafe, kdSafe);
  jointsctrl[6].specifySafeSettings(kpSafe, kiSafe, kdSafe);
  // j7
  node->param(std::string("owd/JointCtrlSea/j7Kp_safe"), kpSafe, 2.0);
  node->param(std::string("owd/JointCtrlSea/j7Ki_safe"), kiSafe, 0.0);
  node->param(std::string("owd/JointCtrlSea/j7Kd_safe"), kdSafe, 0.1);
  ROS_INFO("j7 safe settings kp:%f, ki:%f, kd:%f", kpSafe, kiSafe, kdSafe);
  jointsctrl[7].specifySafeSettings(kpSafe, kiSafe, kdSafe);


  double torqLimit;
  // J1
  node->param(std::string("owd/JointCtrlSea/j1Limit"), torqLimit, 15.0);
  ROS_INFO("owd/JointCtrlSea/j1Limit %f", torqLimit);
  jointsctrl[1].setTorqLimit(torqLimit);
  // J2
  node->param(std::string("owd/JointCtrlSea/j2Limit"), torqLimit, 15.0);
  ROS_INFO("owd/JointCtrlSea/j2Limit %f", torqLimit);
  jointsctrl[2].setTorqLimit(torqLimit);
  // J3
  node->param(std::string("owd/JointCtrlSea/j3Limit"), torqLimit, 10.0);
  ROS_INFO("owd/JointCtrlSea/j3Limit %f", torqLimit);
  jointsctrl[3].setTorqLimit(torqLimit);
  // J4
  node->param(std::string("owd/JointCtrlSea/j4Limit"), torqLimit, 8.0);
  ROS_INFO("owd/JointCtrlSea/j4Limit %f", torqLimit);
  jointsctrl[4].setTorqLimit(torqLimit);
  // J5
  node->param(std::string("owd/JointCtrlSea/j5Limit"), torqLimit, 3.0);
  ROS_INFO("owd/JointCtrlSea/j5Limit %f", torqLimit);
  jointsctrl[5].setTorqLimit(torqLimit);
  // J6
  node->param(std::string("owd/JointCtrlSea/j6Limit"), torqLimit, 3.0);
  ROS_INFO("owd/JointCtrlSea/j6Limit %f", torqLimit);
  jointsctrl[6].setTorqLimit(torqLimit);
  // J7
  node->param(std::string("owd/JointCtrlSea/j7Limit"), torqLimit, 1.0);
  ROS_INFO("owd/JointCtrlSea/j7Limit %f", torqLimit);
  jointsctrl[7].setTorqLimit(torqLimit);

  double kp;
  // J1
  node->param(std::string("owd/JointCtrlSea/j1Kp"), kp, 1800.0);
  ROS_INFO("owd/JointCtrlSea/j1Kp %f", kp);
  jointsctrl[1].setKp(kp);
  // J2
  node->param(std::string("owd/JointCtrlSea/j2Kp"), kp, 2000.0);
  ROS_INFO("owd/JointCtrlSea/j2Kp %f", kp);
  jointsctrl[2].setKp(kp);
  // J3
  node->param(std::string("owd/JointCtrlSea/j3Kp"), kp, 1200.0);
  ROS_INFO("owd/JointCtrlSea/j3Kp %f", kp);
  jointsctrl[3].setKp(kp);
  // J4
  node->param(std::string("owd/JointCtrlSea/j4Kp"), kp, 300.0);
  ROS_INFO("owd/JointCtrlSea/j4Kp %f", kp);
  jointsctrl[4].setKp(kp);
  // J5
  node->param(std::string("owd/JointCtrlSea/j5Kp"), kp, 10.0);
  ROS_INFO("owd/JointCtrlSea/j5Kp %f", kp);
  jointsctrl[5].setKp(kp);
  // J6
  node->param(std::string("owd/JointCtrlSea/j6Kp"), kp, 15.0);
  ROS_INFO("owd/JointCtrlSea/j6Kp %f", kp);
  jointsctrl[6].setKp(kp);
  // J7
  node->param(std::string("owd/JointCtrlSea/j7Kp"), kp, 2.0);
  ROS_INFO("owd/JointCtrlSea/j7Kp %f", kp);
  jointsctrl[7].setKp(kp);

  double kd;
  // J1
  node->param(std::string("owd/JointCtrlSea/j1Kd"), kd, 10.0);
  ROS_INFO("owd/JointCtrlSea/j1Kd %f", kd);
  jointsctrl[1].setKd(kd);
  // J2
  node->param(std::string("owd/JointCtrlSea/j2Kd"), kd, 20.0);
  ROS_INFO("owd/JointCtrlSea/j2Kd %f", kd);
  jointsctrl[2].setKd(kd);
  // J3
  node->param(std::string("owd/JointCtrlSea/j3Kd"), kd, 10.0);
  ROS_INFO("owd/JointCtrlSea/j3Kd %f", kd);
  jointsctrl[3].setKd(kd);
  // J4
  node->param(std::string("owd/JointCtrlSea/j4Kd"), kd, 5.0);
  ROS_INFO("owd/JointCtrlSea/j4Kd %f", kd);
  jointsctrl[4].setKd(kd);
  // J5
  node->param(std::string("owd/JointCtrlSea/j5Kd"), kd, 1.5);
  ROS_INFO("owd/JointCtrlSea/j5Kd %f", kd);
  jointsctrl[5].setKd(kd);
  // J6
  node->param(std::string("owd/JointCtrlSea/j6Kd"), kd, 0.1);
  ROS_INFO("owd/JointCtrlSea/j6Kd %f", kd);
  jointsctrl[6].setKd(kd);
  // J7
  node->param(std::string("owd/JointCtrlSea/j7Kd"), kd, 0.1);
  ROS_INFO("owd/JointCtrlSea/j7Kd %f", kd);
  jointsctrl[7].setKd(kd);

  double ki;
  // J1
  node->param(std::string("owd/JointCtrlSea/j1Ki"), ki, 0.0);
  ROS_INFO("owd/JointCtrlSea/j1Ki %f", ki);
  jointsctrl[1].setKi(ki);
  // J2
  node->param(std::string("owd/JointCtrlSea/j2Ki"), ki, 0.0);
  ROS_INFO("owd/JointCtrlSea/j2Ki %f", ki);
  jointsctrl[2].setKi(ki);
  // J3
  node->param(std::string("owd/JointCtrlSea/j3Ki"), ki, 0.0);
  ROS_INFO("owd/JointCtrlSea/j3Ki %f", ki);
  jointsctrl[3].setKi(ki);
  // J4
  node->param(std::string("owd/JointCtrlSea/j4Ki"), ki, 0.0);
  ROS_INFO("owd/JointCtrlSea/j4Ki %f", ki);
  jointsctrl[4].setKi(ki);
  // J5
  node->param(std::string("owd/JointCtrlSea/j5Ki"), ki, 0.0);
  ROS_INFO("owd/JointCtrlSea/j5Ki %f", ki);
  jointsctrl[5].setKi(ki);
  // J6
  node->param(std::string("owd/JointCtrlSea/j6Ki"), ki, 0.0);
  ROS_INFO("owd/JointCtrlSea/j6Ki %f", ki);
  jointsctrl[6].setKi(ki);
  // J7
  node->param(std::string("owd/JointCtrlSea/j7Ki"), ki, 0.0);
  ROS_INFO("owd/JointCtrlSea/j7Ki %f", ki);
  jointsctrl[7].setKi(ki);
}
#endif


/*
 * Output the information about the WAM
 */
void WAM::dump(){
  ROS_DEBUG("WAM information:");
  ROS_DEBUG("%d joints were defined", Joint::Jn);
  //  for(int j=Joint::J1; j<=Joint::Jn; j++) 
  //    ROS_DEBUG_STREAM("" << joints[j]);
  //  ROS_DEBUG("Controllers:");
  //  for(int j=Joint::J1; j<=Joint::Jn; j++) 
  //    ROS_DEBUG_STREAM("" << jointsctrl[j]);
  //  ROS_DEBUG("%d motors were defined",Motor::Mn);
  //  for(int m=Motor::M1; m<=Motor::Mn; m++) 
  //    ROS_DEBUG_STREAM("" << motors[m]);
}

/*
 * Ask the CAN bus to query the pucks for positions and copy the result
 * in the motor positions. The positions are already converted in rads
 * (check CANBus::get_positions(double*)
 */
int WAM::recv_mpos(){
  double mpos[NUM_NODES+1]; // large enough for the all possible pucks

  if (bus->simulation) { // allow running without CANbus for simulation
    return OW_SUCCESS;
  }

  // fetch positions from the bus
  if(bus->read_positions(mpos) == OW_FAILURE){
    ROS_ERROR("WAM::recv_mpos: read_positions failed." );
    return OW_FAILURE;
  }

  // copy the positions into the motors' structures
  for(int m=Motor::M1; m<=Motor::Mn; m++)
    motors[m].pos( mpos[ motors[m].id() ] );

  return OW_SUCCESS;
}

/*
 * Used to send motor torques on the bus. The torques *are* converted
 */
int WAM::send_mtrq(){
  int32_t mtrq[NUM_NODES+1];

  for(int m=Motor::M1; m<=Motor::Mn; m++)
    mtrq[ motors[m].id() ] = (int32_t) (motors[m].t * motors[m].IPNm());

  if (bus->simulation) { // allow running without CANbus for simulation
    return OW_SUCCESS;
  }

  if(bus->send_torques(mtrq) == OW_FAILURE){
    ROS_ERROR("WAM::send_mtrq: send_torques failed." );
    return OW_FAILURE;
  }

  return OW_SUCCESS;
}

// convert motors positions to joints positions
void WAM::mpos2jpos(){
  joints[1].q = -motors[1].q/mN1 ;
  joints[2].q =  motors[2].q*0.5/mN2     -motors[3].q*0.5/mN3 ;
  joints[3].q = -motors[2].q*0.5*mn3/mN2 -motors[3].q*0.5*mn3/mN3 ;
  joints[4].q = -motors[4].q/mN4 ;
#ifdef WRIST
  joints[5].q =  motors[5].q*0.5/mN5     +motors[6].q*0.5/mN6 ;
  joints[6].q = -motors[5].q*0.5*mn6/mN5 +motors[6].q*0.5*mn6/mN6 ;
  joints[7].q = -motors[7].q/mN7 ;
#endif
}

// convert joints positions to motors positions
void WAM::jpos2mpos(){
  motors[1].q = -joints[1].q *mN1 ;
  motors[2].q =  joints[2].q*mN2 - joints[3].q*mN2/mn3 ;
  motors[3].q = -joints[2].q*mN3 - joints[3].q*mN3/mn3 ;
  motors[4].q = -joints[4].q*mN4 ;
#ifdef WRIST
  motors[5].q =  joints[5].q*mN5 - joints[6].q*mN5/mn6 ;
  motors[6].q =  joints[5].q*mN6 + joints[6].q*mN6/mn6 ;
  motors[7].q = -joints[7].q*mN7 ;
#endif
}

// convert joints torques to motor torques
void WAM::jtrq2mtrq(){
  motors[1].t = -joints[1].t/mN1 ;
  motors[2].t =  joints[2].t*0.5/mN2 - joints[3].t*0.5*mn3/mN3 ;
  motors[3].t = -joints[2].t*0.5/mN2 - joints[3].t*0.5*mn3/mN3 ;
  motors[4].t = -joints[4].t/mN4 ;
#ifdef WRIST
  motors[5].t =  joints[5].t*0.5/mN5 - joints[6].t*0.5*mn6/mN6 ;
  motors[6].t =  joints[5].t*0.5/mN5 + joints[6].t*0.5*mn6/mN6 ;
  motors[7].t = -joints[7].t/mN7 ;
#endif
}

/*
 * Used to set/initialize the pucks positions.
 * WARNING
 * Should really backup the joints and motor values in case that 
 * CANBus::set_positions(double*) fails so that we can write them back...
 */
int WAM::set_jpos(double* pos){
  double mpos[NUM_NODES+1];

  // block the control loop from messing with the motors and joints
  this->lock("set_jpos");

  // convert the joint positions to motor positions
  for(int j=Joint::J1; j<=Joint::Jn; j++)
  {
    joints[j].pos( pos[ joints[j].id() ] );
  }

  jpos2mpos();

  // copy the positions in the array
  for(int m=Motor::M1; m<=Motor::Mn; m++)
  {
    mpos[ motors[m].id() ] = motors[m].q;
  }

  // send the position array on the CAN bus
  if(bus->send_positions(mpos) == OW_FAILURE){
    ROS_ERROR("WAM::set_jpos: bus.send_position failed." );
    this->unlock("set_jpos");
    return OW_FAILURE;
  }
  this->unlock("set_jpos");
  return OW_SUCCESS;
}


void WAM::get_current_data(double* pos, double *trq, double *nettrq, double *simtrq){
    this->lock("get_current_data");
    if (pos) {
      // joint positions
      for(int j=Joint::J1; j<=Joint::Jn; j++)
        pos[ joints[j].id() ] = joints[j].q;
    }
    if (trq) {
      // actual motor torques
      for(int j=Joint::J1; j<=Joint::Jn; j++)
        trq[ joints[j].id() ] = joints[j].t;
    }
    if (nettrq) {
      // subtracts out static (gravity) and dynamic (acceleration) torques to
      // leave just what balances ext. forces
      for(int j=Joint::J1; j<=Joint::Jn; j++)
        nettrq[ joints[j].id() ] = pid_torq[j];
    }

    if (simtrq) {
      // torques computed by the simulated links (with experimental
      // mass properties)
      for(int j=Joint::J1; j<=Joint::Jn; j++)
        simtrq[ joints[j].id() ] = sim_torq[j];
    }
    this->unlock();
}

/*
 * Start the control timer and loop
 */
int WAM::start(){
  // start the control loop and uses control_loop_rt (below) as the main function
  // pass this WAM as an argument to control_loop_rt
  if(ctrl_loop.start() == OW_FAILURE){
    ROS_ERROR("WAM::start: starting control loop failed." );
  }
  return OW_SUCCESS;
}

void WAM::stop(){
  if(ctrl_loop.stop() == OW_FAILURE)
    ROS_ERROR("WAM::stop: ctrl_loop.stop() failed." );
}

/*
 * Function of the control loop thread.
 * Doesn't do much. It wakes up on the semaphore posted by the control loop
 * timer and call the WAM's control method.
 * The parameter argv is the WAM object
 */
void control_loop_rt(void* argv){
  WAM* wam = (WAM*)argv;                      // read the WAM argument
  ControlLoop* ctrl_loop = &(wam->ctrl_loop); // get the ControlLoop member
  RTIME t1,t2,t4;

  // Use __LATENCY__ to record stuff for control loop latencies
#ifdef __LATENCY__
#define ARRAY_SIZE 20000
  double time[ARRAY_SIZE];
  int i=0;
#endif

  // Use __IDENTIFICATION__ to record stuff for dynamic identification
#ifdef __IDENTIFICATION__
#define ARRAY_SIZE 30000
  double time[ARRAY_SIZE];
  double positions[ARRAY_SIZE][8];
  double torques[ARRAY_SIZE][8];
  int i=0;
#endif


  unsigned int slowcounts=0;
  unsigned int fastcounts=0;
  double readtime=0.0f;
  double controltime=0.0f;
  double sendtime=0.0f;
  double slowtime=0.0f;
  double slowmax=0.0f;
  double slowreadtime=0.0f;
  double slowctrltime=0.0f;
  double slowsendtime=0.0f;
  double looptime=0.0f;
  unsigned int loopcount = 0;
  unsigned int slowcount=0;

  t1 = ControlLoop::get_time_ns_rt();  // first-time initialization
  ROS_DEBUG("Control loop started");

  // Main control loop
  while((ctrl_loop->state_rt() == CONTROLLOOP_RUN) && (ros::ok())){
    //    printf("waiting for next timeslot\n");
      ctrl_loop->wait_rt();         // wait for the next control iteration;
      t2 = ControlLoop::get_time_ns_rt(); // record the time
      //      printf("loop execution\n");

      // GET POSITIONS
#ifdef BH280_ONLY
      // only log messages if we're not trying to control the WAM in RT
      //      ROS_DEBUG_NAMED("can_bh280","reading positions");
#endif
      if(wam->bus->read_positions_rt() == OW_FAILURE){
        ROS_FATAL("control_loop: read_positions failed");
        return;
      }
      //      printf("read positions\n");
      
      RTIME bt1 = ControlLoop::get_time_ns_rt();
      readtime += (bt1-t2 ) * 1e-6;
      
#ifndef BH280_ONLY
      // CONTROL LOOP
      wam->newcontrol_rt( ((double)(t2-t1)) * 1e-9); // ns to s
#endif // BH280_ONLY

      RTIME bt2 = ControlLoop::get_time_ns_rt();
      controltime += (bt2-bt1) * 1e-6; // ns to ms

#ifdef BH280_ONLY
      // only log messages if we're not trying to control the WAM in RT
      //      ROS_DEBUG_NAMED("can_bh280","sending torques");
#endif
      // SEND TORQUES
      if(wam->bus->send_torques_rt() == OW_FAILURE){
        ROS_FATAL("control_loop: send_torques failed.");
        return;
      }
      //      printf("sent torques\n");

      RTIME bt3 = ControlLoop::get_time_ns_rt();
      sendtime += (bt3-bt2) * 1e-6;

#ifndef BH280_ONLY
      // MOTOR STATE
      // only check the state every 200 cycles
      static int stateperiod=0;
      if (++stateperiod == 200) {
	//        printf("checking puck state\n");
        if (wam->bus->set_puck_state_rt() != OW_SUCCESS) {
	  // what to do?
	}
        stateperiod=0;
	//        printf("puck state checked\n");
      }
#endif // BH280_ONLY

#ifdef BH280
      // Hand moving?
      static int handstateperiod=25; // start offset from motor state
      if (++handstateperiod == 50) { // every 0.1 seconds
	//        printf("checking hand state\n");
	wam->bus->hand_set_state_rt();
	handstateperiod=0;
	//        printf("hand state checked\n");
      }
#endif // BH280

      // check for slow loops
      double thistime = (t2-t1)* 1e-6;  // milliseconds
      if (thistime > 5.0) {
        slowcount++;
        slowtime += thistime;
	if (thistime > slowmax) {
	  slowmax=thistime;
	}
        slowreadtime += (bt1-t2) * 1e-6;
        slowctrltime += (bt2-bt1) * 1e-6;
        slowsendtime += (bt3-bt2) * 1e-6;
      }
      looptime += thistime;

      // save time stats
      if (++loopcount == 1000) {
        wam->stats.loopread= readtime/1000.0;
        wam->stats.loopctrl= controltime/1000.0;
        wam->stats.loopsend= sendtime/1000.0;
        wam->stats.looptime= looptime/1000.0;
        wam->stats.slowcount = slowcount/10.0;
        wam->stats.slowavg = slowtime/slowcount;
        wam->stats.slowreadtime = slowreadtime/slowcount;
        wam->stats.slowctrltime = slowctrltime/slowcount;
        wam->stats.slowsendtime = slowsendtime/slowcount;
	wam->stats.slowmax = slowmax;
        readtime=controltime=sendtime=slowtime=0.0f;
        slowreadtime=slowctrltime=slowsendtime=0.0f;
	slowmax=0.0f;
        looptime=0.0f;
        loopcount=slowcount=0;
      }
      
      if (!wam->bus->simulation) { // skip if running in simulation
	int puckstate=wam->bus->get_puck_state();
	if (puckstate != 2) {
	  wam->motor_state=WAM::MOTORS_IDLE;
	  //ROS_DEBUG("Puckstate = %d",puckstate);
	  if (wam->exit_on_pendant_press) {
	    // The motors have been idled (someone pressed shift-idle), so go ahead
	    // and exit without being told.
	    // first, though, send a zero-torque packet to extinguish
	    // the torque warning light
	    for(int m=Motor::M1; m<=Motor::Mn; m++) {
	      wam->motors[m].trq(0.0);
	    }
	    wam->send_mtrq();
            
	    ROS_WARN("Detected motor switch to idle; exiting controller.");
	    ROS_WARN("If the pendant shows a torque or velocity fault, and the yellow idle");
	    ROS_WARN("button is still lit, you can clear the fault by pressing shift-idle");
	    ROS_WARN("again, and encoder values will be preserved.");
	    ROS_WARN("To restart the controller, just re-run the previous command.");
	    break;
	  }
	} else {
	  wam->motor_state=WAM::MOTORS_ACTIVE;
	}        
      }    


      // get ready for next pass
      t1 = t2;
      //      printf("bottom of loop\n");
  }
  ROS_DEBUG("Control loop finished");

#ifdef __LATENCY__
  FILE* fp = fopen("latencies_dir/latencies", "w");
  fwrite((void*)time, sizeof(double), ARRAY_SIZE, fp);
  fclose(fp);
  ROS_DEBUG("Data dumped in file: latencies_dir/latencies." );
#endif
#ifdef __IDENTIFICATION__
  FILE* fp = fopen("identification", "w");
  fwrite((void*)time,             sizeof(double), ARRAY_SIZE,   fp);
  fwrite((void*)&positions[0][0], sizeof(double), ARRAY_SIZE*8, fp);
  fwrite((void*)&torques[0][0],   sizeof(double), ARRAY_SIZE*8, fp);
  fclose(fp);
  ROS_DEBUG("Data dumped in file: identification" );
#endif

  return;
}


/*
 * This is the function called at each control loop
 * All the control stuff should happen in here.
 */

// Mike's control loop outline:
//   1. get current joint positions
//   2. get desired q, qd, qdd from:
//      a. hold position
//      b. pulsetraj
//      c. trajectory
//   3. compute jsdynamics torq using current q and desired qd, qdd
//   4. compute control torq using current q and desired q
//   5. check control torque for limits
//   6. apply jsdynamics torq plus control torq
// question: will control torque be trying to do more than necessary?
void WAM::newcontrol_rt(double dt){
    static double   q[Joint::Jn+1];
    static double   q_target[Joint::Jn+1];
    static double  qd_target[Joint::Jn+1];
    static double qdd_target[Joint::Jn+1];
    static double dyn_torq[Joint::Jn+1];
    R6 F;
    
    static int tc1=0;
    static double jscontroltime=0.0f;
    static int dyncount=0;
    static int trajcount=0;
    static double dyntime=0.0f;
    static double trajtime=0.0f;
    static int safety_torque_count=0;
    
    double traj_timestep;
    static double timestep_factor = 1.0f;
    /*    if (dt > .004) {
      traj_timestep = .004; // bound the time in case the system got delayed; don't want to lurch
    } else {
      traj_timestep = dt;
    }
    */
    
    // read new motor positions
    if(recv_mpos() != OW_SUCCESS){
      // We lost communication with the pucks (someone probably hit the e-stop).
      motor_state=MOTORS_OFF;
      if (exit_on_pendant_press) {
	ROS_FATAL("WAM::control_loop: lost communication with the WAM; shutting down.");
	ROS_FATAL("If you want to restart the controller, make sure power is applied");
	ROS_FATAL("and all e-stops are released, then re-run.");
	exit(1);
      }
    }

    mpos2jpos();    // convert to joint positions
    
    for(int j=Joint::J1; j<=Joint::Jn; j++){
      q_target[j] = q[j] = joints[j].q; // set q_target for traj->eval
      links[j].theta(q[j]);
      sim_links[j].theta(q[j]);
      qd_target[j] = qdd_target[j] = 0.0; // zero out
    }
    std::vector<double> data;
    bool data_recorded=false;

    // is there a joint trajectory running
    static int skipped_locks=0;
    
    if (! this->lock_rt("newcontrol") ) {
      // if we couldn't get the lock in the time we wanted, then
      // just republish the previous torques.  we will only do this
      // 10 times before giving up on the current trajectory
      if (++skipped_locks > 10) {
	// reduce the torque
        for(int j=Joint::J1; j<=Joint::Jn; j++){
	  joints[j].t *= 0.99; // this should bring the WAM to zero torque in about 250 cycles (0.5 secs)
        }
        jtrq2mtrq();         // results in motor::torque
	// need to push an error message through a message pipe
      }
      if(send_mtrq() == OW_FAILURE) {
	ROS_ERROR("WAM::control_loop: send_mtrq failed.");
      }
      return;
    }

    skipped_locks=0;

    if(jointstraj != NULL){
#ifdef BUILD_FOR_SEA
      posSmoother.invalidate();
#endif // BUILD_FOR_SEA
      try {
            RTIME t3 = ControlLoop::get_time_ns_rt();        
            jointstraj->evaluate(q_target, qd_target, qdd_target, traj_timestep * timestep_factor);
            RTIME t4 = ControlLoop::get_time_ns_rt();        
            trajtime += (t4-t3) / 1e6;
            for(int j=Joint::J1; j<=Joint::Jn; j++){
              qd_target[j] *= timestep_factor;
              qdd_target[j] *= timestep_factor * timestep_factor;
            }
            if(jointstraj->state() == Trajectory::DONE){
              // we've gone past the last time in the trajectory,
              // so set up to hold at the final position.  we'll let
              // the trajectory control values persist for the rest of this
              // iteration (to help deccelerate if we're still moving), but
              // the next time around we'll start holding at the endpoint
              jointstraj->endPosition().cpy(&heldPositions[Joint::J1]);
#ifdef BUILD_FOR_SEA	      
              posSmoother.reset(heldPositions, Joint::Jn+1);   
#endif
              holdpos = true;  // should have been true already, but just making sure
              delete jointstraj;
              jointstraj = NULL;
              //ROS_INFO("Trajectory done");
            }
            
            // ask the controllers to compute the correction torques.
            RTIME t1 = ControlLoop::get_time_ns_rt();
            newJSControl_rt(q_target,q,dt,pid_torq);
            RTIME t2 = ControlLoop::get_time_ns_rt();
            jscontroltime += (t2-t1) / 1e6;
            
	    // inform the trajectory of how much torque it's causing
	    if (jointstraj) { // make sure it wasn't deleted, above
	      jointstraj->update_torques(pid_torq);
	    }

	    data_recorded=true;
	    data.push_back(t1); // record the current time
	    data.push_back(timestep_factor);  // time factor
	    if (jointstraj) {
	      data.push_back(jointstraj->curtime());  // traj time
	    } else {
	      data.push_back(0);
	    }
	    for (unsigned int j=Joint::J1; j<=Joint::Jn; ++j) {
		  data.push_back(q_target[j]);  // record target position
		  data.push_back(q[j]);         // record actual position
		  data.push_back(pid_torq[j]);  // record the pid torques
	    }

            if (safety_torques_exceeded(pid_torq)) {
              // hold here with zero target velocity and acceleration,
              // and wait until the limit condition goes away.
              if (jointstraj) { // might have ended and been cleared (above)
                // March 2009: something to try
                // just slow down the time, and take a smaller step.
                // as long as we keep exceeding, keep trying smaller steps.
                // as we free up, we can start taking bigger steps again.
                if (timestep_factor > 0.05f) {
                  timestep_factor *= 0.97; // at 3% decrease per control loop
		                           // it will take 100ms to reach
		                           // 5% real-time.
                }
                safety_torque_count++;
                safety_hold=true; // let the app know that we're hitting something
                if (jointstraj->HoldOnStall) {
                  jointstraj->stop();  // still have to stop if the app wants to
                }
              }
            } else if (timestep_factor < 1.0f) {
              // we're back within the safety thresholds, so start 
	      // increasing our stepsize again
              timestep_factor *= 1.03f;
              if (timestep_factor > 1.0f) {
                timestep_factor = 1.0f;
              }
              if (jointstraj != NULL) {
                if (!jointstraj->HoldOnStall) {
                  jointstraj->run();
                }
              }
              safety_hold=false;
            }
          } catch (char *error) {
            // most likely a problem evaluating the trajectory, so
            // halt it and just hold position
            ROS_ERROR("Problem in trajectory; aborting and holding current position");
            delete jointstraj;
            jointstraj = NULL;
            double heldPositions[Joint::Jn+1];
            hold_position(heldPositions,false);

#ifdef BUILD_FOR_SEA
            // now do the same thing that the "if (holdpos)" step does, below
	    try {
	      posSmoother.getSmoothedPVA(heldPositions);
	    } catch (char *errstr) {
	      // state was invalid; ignore and use the one held (above)
	    }
#endif // BUILD_FOR_SEA
	      
            for(int i = Joint::J1; i <= Joint::Jn; i++) {
              q_target[i] = heldPositions[i];
            }
            // ask the controllers to compute the correction torques.
            RTIME t1 = ControlLoop::get_time_ns_rt();
            newJSControl_rt(q_target,q,dt,pid_torq);
            RTIME t2 = ControlLoop::get_time_ns_rt();
            jscontroltime += (t2-t1) / 1e6;
            
          }
        } else if (holdpos) {

#ifdef BUILD_FOR_SEA
          if ( posSmoother.isValid() == false ) {
            // reset
            std::vector<double> startPosVec;
            // get current joint positions
            for(int j=0; j<=Joint::Jn; j++) {
              startPosVec.push_back(joints[j].q);
            }
            posSmoother.reset(startPosVec);
          }

	  try {
	    posSmoother.getSmoothedPVA(heldPositions);
	  } catch (char *errstr) {
	    // state was invalid
	  }
          //TODO if we want to use jsdyn, I believe we will have to set qd_target and qdd_target
#endif // BUILD_FOR_SEA

          for(int i = Joint::J1; i <= Joint::Jn; i++) {
            q_target[i] = heldPositions[i];
          }
          
          // ask the controllers to compute the correction torques.
          RTIME t1 = ControlLoop::get_time_ns_rt();
          newJSControl_rt(q_target,q,dt,pid_torq);
          RTIME t2 = ControlLoop::get_time_ns_rt();
          jscontroltime += (t2-t1) / 1e6;
          
          if (safety_torques_exceeded(pid_torq)) {
            // NEW WAY:

#ifdef BUILD_FOR_SEA
            std::vector<double> slipPosVec;
            // reset the posSmoother at a new
            // target pos 10% towards the current pos 
            for(int j=0; j<=Joint::Jn; j++) {
              slipPosVec.push_back(0.9 * heldPositions[j] + 0.1 * joints[j].q);
            }
            posSmoother.reset(slipPosVec);
#else // BUILD_FOR_SEA
	    for(int jj = Joint::J1; jj <= Joint::Jn; jj++) {
	      double jointdiff = joints[jj].q - heldPositions[jj];
	      // inch the target pos 10% towards the current pos
	      heldPositions[jj] += 0.1f * jointdiff;
	    }
#endif // BUILD_FOR_SEA
          }

        } else {
#ifdef BUILD_FOR_SEA
          posSmoother.invalidate();
#endif // BUILD_FOR_SEA
          for(int i = Joint::J1; i <= Joint::Jn; i++) {
            pid_torq[i]=0.0f;  // zero out the torques otherwise
          }
        }
        if (++trajcount == 1000) {
          stats.trajtime = trajtime/1000.0;
          stats.jsctrltime=jscontroltime/1000.0;
          stats.safetycount=safety_torque_count;
	  for (unsigned int i=0; i<7; ++i) {
	    stats.hitorquecount[i]=safetytorquecount[i];
	    stats.hitorqueavg[i]=safetytorquesum[i] / safetytorquecount[i];
	    safetytorquesum[i]=safetytorquecount[i]=0;
	  }
          trajcount=safety_torque_count=0;
          trajtime=jscontroltime=0.0f;
        }
        
     
        // compute the torque required to meet the desired qd and qdd
	// always calculate simulated dynamics (we'll overwrite dyn_torq later)
	JSdynamics(sim_torq, sim_links, qd_target, qdd_target); 
	if (data_recorded) {
	  for (unsigned int j=Joint::J1; j<=Joint::Jn; ++j) {
	    data.push_back(dyn_torq[j]);
	  }
	}
        if(jsdyn){
          RTIME t4 = ControlLoop::get_time_ns_rt();
          JSdynamics(dyn_torq, links, qd_target, qdd_target); 
          RTIME t5 = ControlLoop::get_time_ns_rt();
          dyntime += (t5-t4) / 1e6;
          if (++dyncount == 1000) {
            stats.dyntime = dyntime/1000.0;
            dyncount=0; dyntime=0.0f;
          }

        } else {
            for (int j=Joint::J1; j<=Joint::Jn; j++) {
                dyn_torq[j]=0.0;  // zero out the torques otherwise
            }
        }
	if (data_recorded) {
	  for (unsigned int j=Joint::J1; j<=Joint::Jn; ++j) {
	    data.push_back(dyn_torq[j]);
	  }
	  recorder.add(data);
	}

        
        // finally, sum the control torq and dynamics torq
        for(int j=Joint::J1; j<=Joint::Jn; j++){
            joints[j].trq(stiffness*pid_torq[j] + dyn_torq[j]);  // send to the joints
        }
        
        jtrq2mtrq();         // results in motor::torque
        
        if(send_mtrq() == OW_FAILURE) {
          ROS_ERROR("WAM::control_loop: send_mtrq failed.");
        }
	if (bus->simulation) {
	  // If we're running in simulation mode, then set the joint positions
	  // as if they instantly moved to where we wanted.
	  this->unlock();
	  set_jpos(q_target);
	  this->lock_rt("newcontrol2");
	}

    this->unlock();
}

bool WAM::safety_torques_exceeded(double t[]) {
    //    static double safety_torqs[8]={0.0,10.0,10.0,5.0,5.0,2,2,2};
    static double safety_torqs[8]={0.0, 15.0, 30.0, 10.0, 15.0, 3.0, 3.0, 3.0};
    //static double safety_torqs[8]={0.0,50.0,75.0,75.0,60.0,15.0,15.0,15.0};
    bool bExceeded = false;
    if (!check_safety_torques) {
        return bExceeded;
    }
    for (int i = 1; i <= Joint::Jn; i++) {
        if (stiffness * fabs(t[i])>safety_torqs[i]) {
          //          ROS_WARN("SAFETY TORQUE EXCEEDED: J%d, %2.2f>%2.2f\n",
          //                                   i,fabs(t[i]),safety_torqs[i]);
          // set s.t. it will not exceed
          if (stiffness != 0.0) {
            //t[i] = safety_torqs[i]/stiffness *  t[i]/fabs(t[i]);
          }
          bExceeded = true;
	  safetytorquecount[i-1]++;
	  safetytorquesum[i-1] += fabs(t[i]);
        }
    }
    return bExceeded;
}


R6 WAM::WSControl(double dt){
  R3 pd, wd, pdd, wdd;  // desired velocities and accelerations
  SE3 E0ns = E0n;       // desired forward kinematics (initialized to FK
                        // in case the trajectory is not running but the 
                        // controller below is)
  
  // if the trajectory isn't running all values are left intact
  se3traj->evaluate(E0ns, pd, pdd, wd, wdd, dt);
  SO3 Rn0 = !((SO3)E0n);
  pdd = Rn0*pdd;        // align the linear acc in the body frame
  wdd = Rn0*wdd;        // align the angular acc in the body frame
  
  R6 F( pdd, wdd );                     // feedforward forces/moments

  // if the controller isn't running the values are left intact
  F = F + se3ctrl.evaluate(E0ns, E0n, dt); // + feedback forces/moments

  return F;
}

void WAM::newJSControl_rt(double q_target[],double q[],double dt,double pid_torq[]) {
   for(int j=Joint::J1; j<=Joint::Jn; j++) {
       if (suppress_controller[j]) {
           pid_torq[j] = 0;
       } else {
           pid_torq[j]= jointsctrl[j].evaluate(q_target[j], q[j], dt );
       }
   }
}

void WAM::JSControl(double qdd[Joint::Jn+1], double dt){
  double q[Joint::Jn+1], qd[Joint::Jn+1];

  for(int j=Joint::J1; j<=Joint::Jn; j++){
    qdd[j] = qd[j] = 0.0;
    q[j] = joints[j].q;   // initialize the position to the current values
  }                           // in case that no trajectory is running and the
                              // controllers are
  // if no trajectory is running, then all values are left intact
  jointstraj->evaluate(q, qd, qdd, dt);     // feedforward acceleration

  // if the controllers aren't running the values are left intact
  for(int j=Joint::J1; j<=Joint::Jn; j++)   // + feedback
    qdd[j] = qdd[j] + jointsctrl[j].evaluate( q[j], joints[j].q, dt );
}

int WAM::run_trajectory(Trajectory *traj) {
    if (jointstraj) {
        // we still have a previous active trajectory
      ROS_WARN("Previous trajectory (id=%d) still active; cannot run new one (id=%d)",jointstraj->id,traj->id);
        return OW_FAILURE;
    }
    if (!holdpos) {
        // we can only start if the controllers are already holding the initial
        // position; otherwise the arm could have drifted and we'll get a bad lurch
        // when we try to start the trajectory
      ROS_ERROR("Cannot start a trajectory without first holding position");
      return OW_FAILURE;
    }
    if (! traj->WaitForStart) {
      traj->run(); // put into running state
      ROS_DEBUG("Starting trajectory %d",traj->id);
    } else {
      ROS_DEBUG("Starting trajectory %d in paused state",traj->id);
      // app must call resume_trajectory() to start
    }
    this->lock("run_traj");
    jointstraj = traj;
    this->unlock("run_traj");
    return OW_SUCCESS;
}

int WAM::pause_trajectory() {
    if (!jointstraj) {
        // there was no trajectory running
        return OW_FAILURE;
    }
    jointstraj->stop();
    return OW_SUCCESS;
}

int WAM::resume_trajectory() {
    if (!jointstraj) {
        // there was no trajectory running
        return OW_FAILURE;
    }

    this->lock("resume_traj");
    jointstraj->run();
    this->unlock("resume_traj");
    return OW_SUCCESS;
}

int WAM::cancel_trajectory() {
    if (!jointstraj) {
        // no trajectory exists
        return OW_SUCCESS;
    }
    this->lock("cancel_traj");
    jointstraj->stop();

    for(int i = Joint::J1; i<=Joint::Jn; i++) {
      heldPositions[i] = joints[i].q;
    }
#ifdef BUILD_FOR_SEA
    posSmoother.reset(heldPositions, Joint::Jn+1);   
#endif // BUILD_FOR_SEA     
    holdpos = true;  // should have been true already, but just making sure
    delete jointstraj;
    jointstraj = NULL;
    this->unlock("cancel_traj");
    return OW_SUCCESS;
}


void WAM::move_sigmoid(const SE3& E02){
  se3traj = new SE3Traj( FK(), E02, 
                         new Sigmoid(Sigmoid::VMAX), 
                         new Sigmoid(Sigmoid::WMAX) );
  se3traj->run();
  se3ctrl.reset();
  se3ctrl.set(FK());
  se3ctrl.run();
}

void WAM::set_stiffness(float s) {
  this->lock("set_stiffness");
  stiffness = s;
  this->unlock();
}

int WAM::hold_position(double jval[],bool grab_lock) 
   // jval defaults to NULL, grab_lock defaults to true
   // If jval is non-null, it returns the position being held.
   // if grab_lock is false, it assumes the mutex has already been acquired
   
{
    if (holdpos) { // we're already holding
        if (jval != NULL) {  // but we still need to return the position if requested
#ifdef BUILD_FOR_SEA
	    try {
	      posSmoother.getSmoothedPVA(heldPositions);
	    } catch (char *errstr) {
	      // state was invalid
	    }
#endif // BUILD_FOR_SEA
            for (int i = Joint::J1; i<=Joint::Jn; i++) {
                jval[i]=heldPositions[i];
            }
        }
        return OW_FAILURE;
    }

    if (grab_lock) {
      this->lock("hold pos");
    }

    if(pulsetraj != NULL && !pulsetraj->done)
    {
      ROS_WARN("Cannot hold position, pulse trajectory is running\n");
        if (grab_lock) {
            this->unlock("hold pos");
        }
        return OW_FAILURE;
    }

    if(jointstraj != NULL)
    {
        jointstraj->stop();
    }

    for(int i = Joint::J1; i <= Joint::Jn; i++)
    {
        heldPositions[i] = joints[i].q;
        jointsctrl[i].reset();
        jointsctrl[i].set(heldPositions[i]);
        jointsctrl[i].run();
        if (jval != NULL) {
            jval[i]=heldPositions[i];
        }
    }
#ifdef BUILD_FOR_SEA
    posSmoother.reset(heldPositions, Joint::Jn+1);   
#endif // BUILD_FOR_SEA
    holdpos = true;

    if (grab_lock) {
        this->unlock("hold pos");
    }
    return OW_SUCCESS;
}


void WAM::release_position(bool grab_lock)
{

    if(holdpos)
    {
        if (grab_lock) {
            this->lock("release pos");
        }
        for(int i = Joint::J1; i <= Joint::Jn; i++)
        {
            jointsctrl[i].stop();
        }
        holdpos = false;

#ifdef BUILD_FOR_SEA
        posSmoother.invalidate();
#endif // BUILD_FOR_SEA

        if (grab_lock) {
            this->unlock("release pos");
        }
    }
}


void WAM::printjpos(){
  for(int j=Joint::J1; j<=Joint::Jn; j++)
    ROS_DEBUG("J%d: %2.2f",j,joints[j].q);
}
void WAM::printmtrq(){
  for(int m=Motor::M1; m<=Motor::Mn; m++)
    ROS_DEBUG("M%d: %2.2f",m,motors[m].t);
}

void WAM::lock(const char *name) {
  static char last_locked_by[100];
#if defined( OWD_RT ) && ! defined ( OWDSIM )
  rt_mutex_acquire(&rt_mutex,TM_INFINITE);
#else
  pthread_mutex_lock(&mutex);
#endif
  strncpy(last_locked_by,name,100);
  last_locked_by[99]=0; // just in case it was more than 99 chars long
    if (name) {
      static char msg[200];
      //      sprintf(msg,"OPENWAM locked by %s",name);
      //      syslog(LOG_ERR,msg);
        //    } else {
        //        syslog(LOG_ERR,"OPENWAM locked by (unknown)");
    }
}

bool WAM::lock_rt(const char *name) {
  static char last_locked_by[100];
#if defined( OWD_RT ) && ! defined ( OWDSIM )
  RTIME waittime(100000); // wait up to 100us for the lock
  if (rt_mutex_acquire(&rt_mutex,waittime)) {
    return false;
  }
#else
  pthread_mutex_lock(&mutex);
#endif
  strncpy(last_locked_by,name,100);
  last_locked_by[99]=0; // just in case it was more than 99 chars long
  //  if (name) {
  //    static char msg[200];
  //      sprintf(msg,"OPENWAM locked by %s",name);
  //      syslog(LOG_ERR,msg);
  //    } else {
  //        syslog(LOG_ERR,"OPENWAM locked by (unknown)");
  //  }
}

void WAM::unlock(const char *name) {
  //  if (name) {
  //    static char msg[200];
  //    sprintf(msg,"OPENWAM unlocked by %s",name);
  //    syslog(LOG_ERR,msg);
  //  } else {
  //    syslog(LOG_ERR,"OPENWAM unlocked by (unknown)");
  //  }
#if defined( OWD_RT ) && ! defined ( OWDSIM )
  rt_mutex_release(&rt_mutex);
#else
  pthread_mutex_unlock(&mutex);
#endif
}

void WAMstats::rosprint(int recorder_count) const {
  ROS_DEBUG_NAMED("times",
                  "Loop: %2.2fms (read %2.1fms, control %2.1fms, send %2.1fms)",
                  looptime,
                  loopread,
                  loopctrl,
                  loopsend);
  if (slowcount > 0) {
    ROS_INFO_NAMED("times","Slow cycles %2.1f%% of the time (avg=%2.1fms, max=%2.1fms)",
                   slowcount, slowavg, slowmax);
    ROS_INFO_NAMED("times",
                   "  Slow breakdown: read %2.1fms, ctrl %2.1fms, send %2.1fms",
                   slowreadtime,
                   slowctrltime,
                   slowsendtime);
    if (slowmax > 90.0) {
      ROS_WARN_NAMED("times",
		     "Realtime control thread took %2.1fms", slowmax);
    }
  }
  ROS_DEBUG_NAMED("times",
                  "trajectory eval %2.2fms, jscontrol %2.2fms, safetycount=%d, recordercount=%d",
                  trajtime,jsctrltime,safetycount,recorder_count);
  if (safetycount > 0) {
    ROS_DEBUG_NAMED("times",
		    "Safety torque exception counts and averages:\n");
    for (unsigned int i=0; i<7; ++i) {
      if (hitorquecount[i]>0) {
	ROS_DEBUG_NAMED("times",
			"J%d: %4d %2.2f", i+1, hitorquecount[i],
			hitorqueavg[i]);
      }
    }
  }

  ROS_DEBUG_NAMED("times","newcontrol dynamics time: %2.2fms",
                  dyntime);
}

WAM::~WAM() {
  if (recorder.count > 0) {
    recorder.dump("/tmp/wamstats_final.csv");
  }
}
