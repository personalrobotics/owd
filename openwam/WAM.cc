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

/* Modified 2007-2011 by:
      Mike Vande Weghe <vandeweg@cmu.edu>
      Robotics Institute
      Carnegie Mellon University
*/

#include "WAM.hh"
#include "Plugin.hh"
#include "Kinematics.hh"
#include <ros/ros.h>
#include <stdio.h>

using namespace OWD;

static void control_loop_rt(void* argv);

extern int MODE;  // puck parameter

#define GROUPID(n)   (0x0400 + (n))

/*
 * Create a new WAM that uses the CAN bus cb;
 */

WAM::WAM(CANbus* cb, int bh_model, bool forcetorque, bool tactile,
	 bool log_ctrl_data) :
  check_safety_torques(true),stall_sensitivity(1.0),
  tc(Joint::Jn), rec(false),wsdyn(false),
  jsdyn(false), holdpos(false),exit_on_pendant_press(false),pid_sum(0.0f), 
  pid_count(0),safety_hold(false),
  log_controller_data(log_ctrl_data), 
  bus(cb),ctrl_loop(cb->id, &control_loop_rt, this),
  motor_state(MOTORS_OFF), stiffness(1.0), recorder(50000),
  BH_model(bh_model), ForceTorque(forcetorque), Tactile(tactile),
  slip_joints_on_high_torque(false)
{

#ifdef OWD_RT
  rt_mutex_create(&rt_mutex,"WAM_CC");
#else // ! OWD_RT
  pthread_mutex_init(&mutex, NULL);
#endif // ! OWD_RT

  se3traj = NULL;
  jointstraj = NULL;
  last_traj_state=Trajectory::DONE;
  pulsetraj = NULL;
  for(int i = Joint::J1; i<=Joint::Jn; i++) {
    heldPositions[i] = 0;
    suppress_controller[i]=false;
  }
  last_control_position = (double *)malloc((Joint::Jn+1)*sizeof(double));
  
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
#else // !WRIST

  // NO WRIST

  // my guess is that the HAND COG is x=-.045m,y=0,z=.410m from J4
  // hand mass with camera is 1.3kg
  // new COG: X=(.01096*1.065 + .045*1.3)/(1.065+1.3) = .0297
  //          Z=(.14054*1.065 + .410*1.3)/(1.065+1.3) = .2887
  Link L4_without_wrist_with_260_hand
    =Link( DH(  M_PI_2,  -0.0450,   0.0000,   0.0000), 2.36513649,
	   R3(  0.0297,0.00002567,0.2887),
	   Inertia(0.01848577,    0.00000219,  -0.00160868,
		   0.01891658,    -0.00000515,   0.00197517, 1) );
  Link L4_without_wrist_without_hand
    =Link( DH(  M_PI_2,  -0.0450,   0.0000,   0.0000), 1.06513649,
	   R3(  0.01095471,0.00002567,0.14053900),
	   Inertia(0.01848577,    0.00000219,  -0.00160868,
		   0.01891658,    -0.00000515,   0.00197517, 1) );
  /* newer numbers from Barrett's Oct 2007 WAM_MassParams_AA-01.pdf 
       Lxx = 0.01848577        Lxy = 0.00000219      Lxz = ­0.00160868 
       Lyx = 0.00000219        Lyy = 0.01891658      Lyz = 0.00000515 
       Lzx = ­0.00160868       Lzy = 0.00000515      Lzz = 0.00197517 
  */
#ifdef BH8
  links[Link::L4] = L4_without_wrist_with_260_hand;
#else // !BH8
  links[Link::L4] = L4_without_wrist_without_hand;
#endif // !BH8

#endif // !WRIST

  //
  // FAT WARNING
  // I had the mass and the center of mass from old files from Barrett.
  // I figured the inertia tensor by modeling a cylinder of 13cm long and
  // 8.5cm diameter, with a uniform density computed from the mass and the
  // cylinder's volume.
  // The kinematics remains the one of J7.

  //new inertias
  L7_with_260_hand 
    = Link( DH(  0.0000,   0.0000,   0.1800,   0.0000), 1.3754270,
	    R3(  0.0000,   0.0000,  -75.0000)*0.001,
	    Inertia(   2558.1007,   0.0000,      0.0000,
		       2558.1007,   0.0000,   1242.1825, M2_MM2) );
  L7_with_280_hand  // same as 260 but with closer CG
    = Link( DH(  0.0000,   0.0000,   0.1800,   0.0000), 1.27548270,
	    R3(  0.0000,   0.0000,  -84.0000)*0.001,
	    Inertia(   2558.1007,   0.0000,      0.0000,
		       2558.1007,   0.0000,   1242.1825, M2_MM2) );
  L7_with_280FT_hand // FT sensor adds 0.13kg more mass and 0.012m more length
    = Link( DH(  0.0000,   0.0000,   0.1800,   0.0000), 1.40548270,
	    R3(  0.0000,   0.0000,  -76.0000)*0.001,
	    Inertia(   2558.1007,   0.0000,      0.0000,
		       2558.1007,   0.0000,   1242.1825, M2_MM2) );
 

  L7_without_hand 
    = Link( DH(  0.0000,   0.0000,   0.1800,   0.0000), 0.06864753,
	    R3( 0.00014836,0.00007252, -0.1232352),
	    Inertia(   0.00003911,    0.00000019,      0.0000,
		       0.00003877,     0.00000,     0.00007614, M2_MM2) );

  // for the Darpa ARM-S program
  // target adds 0.839kg of mass at a point 8mm out from L7 end plate
  // net CG is the L7_without_hand mass combined with the target mass
  L7_with_ARMS_calibration_target 
    = Link( DH(  0.0000,   0.0000,   0.1800,   0.0000), 0.90765,
	    R3( 0.00014836, 0.00007252, -0.112847),
	    Inertia(   0.00003911,    0.00000019,      0.0000,
		       0.00003877,     0.00000,     0.00007614, M2_MM2) );

  for (int i=0; i<Joint::Jn; ++i) {
    safetytorquecount[i]=safetytorquesum[i]=0;
    pid_torq[i+1]=dyn_torq[i+1]=sim_torq[i+1]=traj_torq[i+1]=0;
  }
    
  if (Link::Ln == Link::L7) {
    // wrist installed
    if (BH_model == 260) {
      links[Link::L7] = L7_with_260_hand;
    } else if (BH_model == 280) {
      if (forcetorque) {
	links[Link::L7] = L7_with_280FT_hand;
      } else {
	links[Link::L7] = L7_with_280_hand;
      }
    } else if (BH_model == 999) {
      links[Link::L7] = L7_with_ARMS_calibration_target;
    } else {
      links[Link::L7] = L7_without_hand;
    }
  } else {
    // no wrist
    if (BH_model == 0) {
      links[Link::L4] = L4_without_wrist_without_hand;
    } else {
      // we don't have a separate mass model for L4 with the 280 hand,
      // so use the same model for both hand types
      links[Link::L4] = L4_without_wrist_with_260_hand;
    }
  }
  // remember the original link properties so that we can restore them
  // after someone has added extra mass via the SetExtraMass call.
  for (int l=Link::L1; l<=Link::Ln; ++l) {
    original_links[l] = links[l];
  }

  // set up the sim_links for comparing our experimental dynamic
  // simulation improvement
  for (int i=Link::L1; i<=Link::Ln; ++i) {
    sim_links[i] = links[i];
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
  jointsctrl[1].set_gains( 900, 10.0, 2.5);
  jointsctrl[2].set_gains(2500, 20.0, 5.0);
  jointsctrl[3].set_gains( 600, 10.0, 2.5);
  jointsctrl[4].set_gains( 500,  2.5, 0.5);
  if (Joint::Jn > 4) {
  //                        Kp    Kd    Ki
    jointsctrl[5].set_gains(  40,  0.5, 0.5);
    jointsctrl[6].set_gains(  40,  0.5, 0.5);
    jointsctrl[7].set_gains(  16,  0.16, 0.1);
  }
  if (Joint::Jn > 7) {
    // set 280 Hand gains here
  }

  return OW_SUCCESS;
}


bool WAM::set_gains(int joint,
		    pr_msgs::PIDgains &gains) {
  if ((joint < Joint::J1) || (joint >> Joint::Jn)) {
    return false;
  }
  jointsctrl[joint].set_gains(gains.kp,gains.kd,gains.ki);
  return true;
}

bool WAM::get_gains(std::vector<pr_msgs::PIDgains > &gains) {
  gains.resize(Joint::Jn);
  for (int j=1; j<=Joint::Jn; ++j) {
    jointsctrl[j].get_gains(gains[j-1].kp, gains[j-1].kd, gains[j-1].ki);
  }
  return true;
}



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
    // never happens; this call always succeeds
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
  joints[1].q = -motors[1].q/mN1 
    + joints[1].offset;
  joints[2].q =  motors[2].q*0.5/mN2     - motors[3].q*0.5/mN3
    + joints[2].offset;
  joints[3].q = -motors[2].q*0.5*mn3/mN2 - motors[3].q*0.5*mn3/mN3
    + joints[3].offset;
  joints[4].q = -motors[4].q/mN4
    + joints[4].offset;
#ifdef WRIST
  joints[5].q =  motors[5].q*0.5/mN5     + motors[6].q*0.5/mN6
    + joints[5].offset;
  joints[6].q = -motors[5].q*0.5*mn6/mN5 + motors[6].q*0.5*mn6/mN6
    + joints[6].offset;
  joints[7].q = -motors[7].q/mN7
    + joints[7].offset;
#endif
}

// convert joints positions to motors positions
void WAM::jpos2mpos(){
  motors[1].q = -(joints[1].q - joints[1].offset) *mN1 ;
  motors[2].q =  (joints[2].q - joints[2].offset) *mN2 
    - (joints[3].q - joints[3].offset) *mN2/mn3 ;
  motors[3].q = -(joints[2].q - joints[2].offset) *mN3
    - (joints[3].q - joints[3].offset) *mN3/mn3 ;
  motors[4].q = - (joints[4].q - joints[4].offset) *mN4 ;
#ifdef WRIST
  motors[5].q =  (joints[5].q - joints[5].offset) *mN5
    - (joints[6].q - joints[6].offset) *mN5/mn6 ;
  motors[6].q =  (joints[5].q - joints[5].offset) *mN6 
    + (joints[6].q - joints[6].offset) *mN6/mn6 ;
  motors[7].q = -(joints[7].q - joints[7].offset) *mN7 ;
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
int WAM::set_jpos(double pos[]){
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

int WAM::set_joint_offsets(double offsets[]) {
  // block the control loop until we update both the offsets and the
  // new held position
  this->lock("set_joint_offsets");
  
  // check for running trajectory
  if (jointstraj != NULL) {
    // cannot change joint offsets while a trajectory is running
    // because it would require recomputing the trajectory vals
    this->unlock();
    return OW_FAILURE;
  }

  // check for held position
  if (holdpos) {
    // Adjust the held position to reflect the new offsets.
    // We will add the new offset and subtract out the previous
    // offset (if any)
    for (int j=Joint::J1; j<Joint::Jn; ++j) {
      heldPositions[j] += offsets[ joints[j].id() ]
        - joints[j].offset;
    }
  }

  // now set the new offset
  for (int j=Joint::J1; j<Joint::Jn; ++j) {
    joints[j].offset = offsets[ joints[j].id() ];
  }

  this->unlock();
  return OW_SUCCESS;
}
   
void WAM::get_current_data(double* pos, double *trq, double *nettrq, double *simtrq,double *trajtrq){
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
    if (trajtrq) {
      // torques output by the current trajectory
      for (int j=Joint::J1; j<=Joint::Jn; ++j) {
	trajtrq[joints[j].id()] = traj_torq[j];
      }
    }
    this->unlock();
}

void WAM::get_abs_positions(double* jpos) {
  if (jpos){
    this->lock();
    for(int j=1; j<=4; ++j) {
      jpos[ j ] = bus->jpos[j];
    }
    this->unlock();
  }
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


  double readtime(0.0f);
  double controltime(0.0f);
  double sendtime(0.0f);
  double slowtime(0.0f);
  double slowmax(0.0f);
  double slowreadtime(0.0f);
  double slowctrltime(0.0f);
  double slowsendtime(0.0f);
  double this_cycle_time(0.0f);
  unsigned int loopcount ( 0);
  unsigned int slowcount(0);
  int missing_data_cycles(0);
  RTIME control_start_time, sendtorque_start_time, sendtorque_end_time;

  RTIME last_sendtorque_time = ControlLoop::get_time_ns_rt() - ControlLoop::PERIOD * 1e9;  // first-time initialization
  RTIME last_loopstart_time = last_sendtorque_time;

  ROS_DEBUG("Control loop started");

  int hand_cycles=12;  // we need 12 cycles to get everything we need
                       // from the hand pucks (tactile takes 9 cycles)
  int hand_counter=0;
  // Main control loop
  bool failure=false;
  while((ctrl_loop->state_rt() == CONTROLLOOP_RUN) && (ros::ok())){
      RTIME loopstart_time = ControlLoop::get_time_ns_rt(); // record the time
#ifndef BH280_ONLY
      // REQUEST POSITIONS
      if(wam->bus->request_positions_rt(GROUPID(4)) == OW_FAILURE){
        ROS_FATAL("control_loop: request_positions failed");
	failure=true;
	break;
      }
#endif
      
      if (wam->bus->forcetorque_data) {
	// REQUEST F/T DATA (every cycle)
	if (wam->bus->request_forcetorque_rt() != OW_SUCCESS) {
	  ROS_FATAL("control_loop: request_forcetorque_rt failed");
	  failure=true;
	  break;
	}
      }
      
      // While the pucks are receiving and processing the position request,
      // send out our secondary request for retrieval afterwards.
      // Each of these use a unique counter, so their update rates need
      // not be the same, as long as
      //      (1) they are always offset from one another so that there
      //          are not multiple requests in the same cycle, and
      //      (2) no additional requests are made from the hand for several
      //          cycles after the tactile data is requested.
#ifndef BH280_ONLY
      static int state_cycles(0);
      if (++state_cycles==50) { // once every 50 cycles
	if (wam->bus->request_puck_state_rt(1) != OW_SUCCESS) {
	  ROS_FATAL("control_loop: request_puck_state_rt failed");
	  failure=true;
	  break;
	}
	state_cycles=0;
      }
#endif // ! BH280_ONLY
      if (wam->bus->BH280_installed) {
	if (hand_counter==0) {
	  if (wam->bus->request_hand_state_rt() != OW_SUCCESS) {
	    ROS_FATAL("control_loop: request_hand_state_rt failed");
	    failure=true;
	    break;
	  }
	}

	// if one or more fingers are in the process of performing a
	// HI command, we cannot interrupt them by sending any non-MODE
	// requests, so we'll skip all the other finger requests
	bool pending_hi = false;
	for (int f=0; f<4; ++f) {
	  if (wam->bus->finger_hi_pending[f]) {
	    pending_hi=true;
	    hand_cycles=100; // don't send requests as frequently
	    break;
	  }
	}
	if (!pending_hi) {
          hand_cycles = 12;
	  if (hand_counter==1) {
	    if (wam->bus->request_positions_rt(GROUPID(5)) != OW_SUCCESS) {
	      ROS_FATAL("control_loop: request_positions_rt failed");
	      failure=true;
	      break;
	    }
	  }
	  
	  if (hand_counter==2) {
	    if (wam->bus->request_strain_rt() != OW_SUCCESS) {
	      ROS_FATAL("control_loop: request_strain_rt failed");
	      failure=true;
	      break;
	    }
	  }

	  if (wam->bus->tactile_data) {
	    if (hand_counter==3) {
	      if (wam->bus->request_tactile_rt() != OW_SUCCESS) {
		ROS_FATAL("control_loop: request_tactile_rt failed");
		failure=true;
		break;
	      }
	    }
	  }
	  
	} // ! pending_hi
	
	// increment our counter
	if (++hand_counter > hand_cycles) {
	  hand_counter=0;
	}
      }
      
      // Now just read the response packets for as much time as we have
      // until the next control cycle.  As soon as we get back all seven
      // joint angles we will compute the new torques and send them out, then
      // keep processing the remaining (presumably lower-priority) responses.
      RTIME read_start_time = ControlLoop::get_time_ns_rt(); // record the time
      int32_t time_to_wait = ControlLoop::PERIOD * 1e6  // sec to usecs
	- (read_start_time - loopstart_time) * 1e-3;  // nsecs to usecs
      bool torques_sent(false);
      while (time_to_wait>0) {
	uint8_t  msg[8];
	int32_t msgid, msglen;
	
	if (wam->bus->read_rt(&msgid, msg, &msglen, time_to_wait) == OW_FAILURE){
	  break; // nothing more to read
	}
	
	// process the response
	uint32_t TO_GROUP = msgid & 0x41F;
	uint32_t FROM_NODE = (msgid & 0x3E0) >> 5;
	uint32_t PROPERTY = msg[0] & 0x7F;
	if ((FROM_NODE == wam->bus->get_property_expecting_id) &&
	    (PROPERTY == wam->bus->get_property_expecting_prop)) {
	  wam->bus->process_get_property_response_rt(msgid, msg, msglen);
	} else if (TO_GROUP == 0x403) { // group 3
	  // 22-bit AP response
	  wam->bus->process_positions_rt(msgid, msg, msglen);
	} else if (FROM_NODE == 8) {
	  // forcetorque puck
	  wam->bus->process_forcetorque_response_rt(msgid,msg,msglen);
	  wam->lock_rt("control_loop");
	  if (wam->jointstraj) {
	    // pass the new values to the running trajectory
	    wam->jointstraj->ForceFeedback(wam->bus->filtered_forcetorque_data);
	  }
	  wam->unlock("control_loop");
	} else if (FROM_NODE == 10) {
	  // safety puck
	  wam->bus->process_safety_response_rt(msgid,msg,msglen);
	} else if ((FROM_NODE >= 11) && (FROM_NODE <= 14)) {
	  // hand puck
	  wam->bus->process_hand_response_rt(msgid,msg,msglen);
	  // if this was tactile data and we're running a trajectory,
	  // let the trajectory know about the new tactile readings
	  if (((msgid & 0x41F) == 0x408) && // group 8 (tactile Top 10)
	      wam->bus->valid_tactile_data &&
	      wam->jointstraj) {
	    wam->jointstraj->TactileFeedback(wam->bus->tactile_data,4);
	  } else if (((msgid & 0x41F) == 0x409) // group 9 (tactile hi res)
		     && wam->bus->valid_tactile_data
		     && wam->jointstraj) {
	    wam->jointstraj->TactileFeedback(wam->bus->tactile_data,20);
	  }
	} else if ((FROM_NODE >= 1) && (FROM_NODE <= 7)) {
	  // arm puck
	  wam->bus->process_arm_response_rt(msgid,msg,msglen);
	  if ((wam->bus->received_state_flags & 2) == 2) {
	    if (wam->check_for_idle_rt()) {
	      // The user has pressed shift-idle.  If we wanted to 
	      // keep running and wait for them to activate again,
	      // we could handle it here.
	      // would have to turn off the controllers and
	      // turn off dynamics so that the torques go to zero,
	      // then keep running the loop until they pressed activate
	      // so that we could turn dynamics back on.
	      if (wam->exit_on_pendant_press) {
		goto CONTROL_DONE;
	      }
	    } else {
	      // here's where we could check to see if the user has
	      // re-activated after pressing shift-idle
	    }
	    wam->bus->received_state_flags &= 0xFD; // clear the #1 flag
	  }
	} else {
	  // unknown
	  wam->bus->stats.canread_badpackets++;
	  continue;
	}

#ifndef BH280_ONLY
#ifdef WRIST
	int all_pucks = 0xFE;
#else // !WRIST
	int all_pucks = 0x1E;
#endif // WRIST
	if (!torques_sent && ((wam->bus->received_position_flags & 0xFE) == all_pucks)) {
	  // we've received all of the arm joint values, so compute and
	  // send out the torques

	  control_start_time = ControlLoop::get_time_ns_rt();
	  // tell the control function how long it was between successive
	  // request_position_rt() calls.  
	  wam->lock_rt("control_loop");
	  wam->newcontrol_rt( ((double)(loopstart_time - last_loopstart_time)) * 1e-9); // ns to s
	  wam->unlock("control_loop");
	  last_loopstart_time = loopstart_time;
	  sendtorque_start_time = ControlLoop::get_time_ns_rt();
	  if(wam->bus->send_torques_rt() == OW_FAILURE){
	    ROS_FATAL("control_loop: send_torques failed.");
	    failure=true;
	    break;
	  }
	  torques_sent=true;
	  sendtorque_end_time = ControlLoop::get_time_ns_rt();

	  // make sure we're not falling behind
	  this_cycle_time = 
	    (sendtorque_end_time - last_sendtorque_time)* 1e-6;  // millisecs
	  if (this_cycle_time > ControlLoop::PERIOD * 1000 * 1.25) {
	    // more than 25% above expected period
	    slowcount++;
	    slowtime += this_cycle_time;
	    if (this_cycle_time > slowmax) {
	      slowmax=this_cycle_time;
	    }
	    slowreadtime += (control_start_time-read_start_time) * 1e-6;
	    slowctrltime += (sendtorque_start_time-control_start_time) * 1e-6;
	    slowsendtime += (sendtorque_end_time-sendtorque_start_time) * 1e-6;
	  }
	  // remember time for next cycle
	  last_sendtorque_time = sendtorque_end_time;

	  readtime += (control_start_time-read_start_time ) * 1e-6; // ns to ms
	  controltime += (sendtorque_start_time-control_start_time) * 1e-6;
	  sendtime += (sendtorque_end_time-sendtorque_start_time) * 1e-6;
	}
#endif // ! BH280_ONLY

	// spend some time checking the F/T sensor and BH280 hand
	if (wam->bus->BH280_installed || wam->bus->forcetorque_data) {
	  if (wam->bus->extra_bus_commands() != OW_SUCCESS) {
	    ROS_WARN("control_loop: extra_bus_commands failed.");
	  }
	}

	time_to_wait = ControlLoop::PERIOD * 1e6  // sec to usecs
	  - (ControlLoop::get_time_ns_rt() - loopstart_time) * 1e-3;  // nsecs to usecs
      } // END OF READ LOOP

      static int total_missed_data_cycles=0;
#ifndef BH280_ONLY
      if (! torques_sent) {
	// we must have not received all the joint values before the
	// time expired
	++total_missed_data_cycles;
	if (++missing_data_cycles == 50) {
	  // we went 50 cycles in a row while missing values from at
	  // least 1 puck; the safety puck has generated a heartbeat
	  // fault by now, so give up!
	  ROS_FATAL("Missed CANbus replies from 50 cycles in a row");
	  missing_data_cycles=0; // reset
	  goto CONTROL_DONE;
	}
      } else {
	missing_data_cycles = 0;
      }
#endif // BH280_ONLY
      
      // save time stats
      if (++loopcount == 1000) {
        wam->stats.loopread= readtime/1000.0;
        wam->stats.loopctrl= controltime/1000.0;
        wam->stats.loopsend= sendtime/1000.0;
        wam->stats.looptime= this_cycle_time/1000.0;
        wam->stats.slowcount = slowcount/10.0;
        wam->stats.slowavg = slowtime/slowcount;
        wam->stats.slowreadtime = slowreadtime/slowcount;
        wam->stats.slowctrltime = slowctrltime/slowcount;
        wam->stats.slowsendtime = slowsendtime/slowcount;
	wam->stats.missed_reads = total_missed_data_cycles;
	wam->stats.slowmax = slowmax;
        readtime=controltime=sendtime=slowtime=0.0f;
        slowreadtime=slowctrltime=slowsendtime=0.0f;
	total_missed_data_cycles=0;
	slowmax=0.0f;
        this_cycle_time=0.0f;
        loopcount=slowcount=0;
      }
  }
  if (!failure && wam->exit_on_pendant_press) {
  CONTROL_DONE:
    ROS_WARN("Detected motor switch to idle; exiting controller.");
    ROS_WARN("If the pendant shows a torque or velocity fault, and the yellow idle");
    ROS_WARN("button is still lit, you can clear the fault by pressing shift-idle");
    ROS_WARN("again, and encoder values will be preserved.");
    ROS_WARN("To restart the controller, just re-run the previous command.");

    // Send a zero-torque packet to extinguish the torque warning light
    for(int m=Motor::M1; m<=Motor::Mn; m++) {
      wam->motors[m].trq(0.0);
    }
    wam->send_mtrq();
    wam->bus->send_torques_rt();
#ifdef BH280_ONLY
    // turn off the hand pucks
    for (int p=11; p<=14; ++p) {
      wam->bus->set_property_rt(p,MODE,MODE_IDLE,false,15000);
    }
#endif // BH280_ONLY
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
  
  // Now idle all of the pucks
  /*
  for(int p=1; p<=wam->bus->n_arm_pucks; p++){
    if(wam->bus->set_property_rt(wam->bus->pucks[p].id(), MODE, MODE_IDLE, false, 10000) == OW_FAILURE){
      ROS_WARN("set_property MODE=IDLE failed for puck %d",p);
    }
  }
  if (wam->bus->BH280_installed) {
    for(int p=11; p<=14; p++){
      if(wam->bus->set_property_rt(p, MODE, MODE_IDLE, false, 10000) == OW_FAILURE){
	ROS_WARN("set_property MODE=IDLE failed for puck %d",p);
      }
    }
  }
  if (wam->bus->forcetorque_data) {
    if(wam->bus->set_property_rt(8, MODE, MODE_IDLE, false, 10000) == OW_FAILURE){
      ROS_WARN("set_property MODE=IDLE failed for puck 8");
    }
  }
  if(wam->bus->set_property_rt(10, MODE, MODE_IDLE, false, 10000) == OW_FAILURE){
      ROS_WARN("set_property MODE=IDLE failed for safety puck 10");
      }

  ROS_FATAL("Motors have been idled; exiting OWD");
  */



  // have to tell the other threads that it's time to shut down,
  // but only if we were the ones to detect the shutdown.  If
  // an outside function told us to stop the control loop by setting
  // the state to CONTROLLOOP_STOP, then we will let that function
  // take care of shutting down the CANbus and ROS.
  if ((wam->exit_on_pendant_press) && 
      (ctrl_loop->state_rt() == CONTROLLOOP_RUN)) {
    // delete the WAM object so that it has the opportunity to 
    // write any accumulated activity logs
    delete wam;
    wam=NULL;
    ros::shutdown();
  }
  return;
}

bool WAM::check_for_idle_rt() {

#ifdef BH280_ONLY
  return false;
#endif // BH280_ONLY

  if (bus->simulation) { // skip if running in simulation
    motor_state=WAM::MOTORS_ACTIVE;
    return false;
  }

  int puckstate=bus->get_puck_state();
  if (puckstate == 2) {
    motor_state=WAM::MOTORS_ACTIVE;
    return false;
  }

  //// The motors have been idled (someone pressed shift-idle)

  motor_state=WAM::MOTORS_IDLE;
  return true;
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
  //  static double   q_target[Joint::Jn+1];
  //  static double  qd_target[Joint::Jn+1];
  //  static double qdd_target[Joint::Jn+1];
  R6 F;
    
  static double jscontroltime=0.0f;
  static int dyncount=0;
  static int trajcount=0;
  static double dyntime=0.0f;
  static double trajtime=0.0f;
  static int safety_torque_count=0;
    
  double traj_timestep;
  static double timestep_factor = 1.0f;
  traj_timestep = dt;
  /*    if (dt > .004) {
	traj_timestep = .004; // bound the time in case the system got delayed; don't want to lurch
	} else {
	traj_timestep = dt;
	}
  */
    
  std::vector<double> data;
  bool data_recorded=false;

  // read new motor positions
  recv_mpos(); // will always succeed, since it's just a copy

  mpos2jpos();    // convert to joint positions
    
  for(int j=Joint::J1; j<=Joint::Jn; j++){
    tc.q[j-1] = q[j] = joints[j].q; // set tc.q for traj->eval
    links[j].theta(q[j]);
    sim_links[j].theta(q[j]);
    tc.qd[j-1] = tc.qdd[j-1] = tc.t[j-1] = 0.0; // zero out
  }

  // update the forward kinematics
  SE3_endpoint = OWD::Kinematics::forward_kinematics(links);

  // update the Jacobian and related values
  OWD::Kinematics::update_jacobians(links);

  // update the values in the Plugin base class
  OWD::Plugin::_endpoint = SE3_endpoint;
  for (int i=0; i<Joint::Jn; ++i) {
    OWD::Plugin::_arm_position[i]=q[i+1];
    OWD::Plugin::_target_arm_position[i]=last_control_position[i+1];
    OWD::Plugin::_pid_torque[i]=pid_torq[i+1];
    OWD::Plugin::_dynamic_torque[i]=dyn_torq[i+1];
    OWD::Plugin::_trajectory_torque[i]=traj_torq[i+1];
  }
  if (bus->forcetorque_data) {
    for (unsigned int i=0; i<3; ++i) {
      if (bus->valid_forcetorque_data) {
	OWD::Plugin::_ft_force[i]=bus->forcetorque_data[i];
	OWD::Plugin::_ft_torque[i]=bus->forcetorque_data[i+3];
	OWD::Plugin::_filtered_ft_force[i]=bus->filtered_forcetorque_data[i];
	OWD::Plugin::_filtered_ft_torque[i]=bus->filtered_forcetorque_data[i+3];
	
      } else {
	// recently tared and still waiting to collect the tare correction
	OWD::Plugin::_ft_force[i] =OWD::Plugin::_filtered_ft_force[i]=0;
	OWD::Plugin::_ft_torque[i]=OWD::Plugin::_filtered_ft_torque[i]=0;
      }
    }
  }
  if (bus->BH280_installed) {
    for (int j=0; j<3; ++j) {
      OWD::Plugin::_hand_position[j]=bus->finger_encoder_to_radians(bus->hand_positions[j+1]);
      OWD::Plugin::_target_hand_position[j]=bus->finger_encoder_to_radians(bus->hand_goal_positions[j+1]);
      OWD::Plugin::_strain[j]=bus->hand_strain[j];
    }
    OWD::Plugin::_hand_position[3]=bus->spread_encoder_to_radians(bus->hand_positions[4]);
    OWD::Plugin::_target_hand_position[3]=bus->finger_encoder_to_radians(bus->hand_goal_positions[4]);
  }
  if (bus->tactile_data) {
    for (unsigned int i=0; i<24; ++i) {
      OWD::Plugin::_tactile_f1[i]=bus->tactile_data[i];
      OWD::Plugin::_tactile_f2[i]=bus->tactile_data[i+24];
      OWD::Plugin::_tactile_f3[i]=bus->tactile_data[i+48];
      OWD::Plugin::_tactile_palm[i]=bus->tactile_data[i+72];
    }
  }

  // is there a joint trajectory running?
  if(jointstraj != NULL){
    bool abort(false);
    try {
      RTIME t3 = ControlLoop::get_time_ns_rt();        
      jointstraj->evaluate(tc, traj_timestep * timestep_factor);
      RTIME t4 = ControlLoop::get_time_ns_rt();
      for (int j=0; j<Joint::Jn; ++j) {
	traj_torq[j+1] = tc.t[j];
      }
      trajtime += (t4-t3) / 1e6;
      for(int j=0; j<Joint::Jn; j++){
	// if we're running at less than real-time (due to a stall
	// condition), scale back the vel and accel accordingly
	tc.qd[j] *= timestep_factor;
	tc.qdd[j] *= timestep_factor * timestep_factor;
      }
      last_traj_state = jointstraj->state();
      if(jointstraj->state() == OWD::Trajectory::DONE){
	// we've gone past the last time in the trajectory,
	// so set up to hold at the final position.  we'll let
	// the trajectory control values persist for the rest of this
	// iteration (to help deccelerate if we're still moving), but
	// the next time around we'll start holding at the endpoint
	jointstraj->endPosition().cpy(&heldPositions[Joint::J1]);
	holdpos = true;  // should have been true already, but just making sure
	delete jointstraj;
	jointstraj = NULL;
	for (int j=Joint::J1; j<Joint::Jn; ++j) {
	  traj_torq[j]=0;  // make sure we don't leave these non-zero
	}
      } else if(jointstraj->state() == OWD::Trajectory::ABORT){
	// The trajectory wants us to end right where we are,
	// so hold the current position and delete the trajectory.
	for(int i = Joint::J1; i<=Joint::Jn; i++) {
	  heldPositions[i] = joints[i].q;
	}
	holdpos = true;  // should have been true already, but just making sure
	delete jointstraj;
	jointstraj = NULL;
	for (int j=Joint::J1; j<Joint::Jn; ++j) {
	  traj_torq[j]=0;  // make sure we don't leave these non-zero
	}
      }
            
      // ask the controllers to compute the correction torques.
      RTIME t1 = ControlLoop::get_time_ns_rt();
      newJSControl_rt((&tc.q[0])-1,q,dt,pid_torq);
      RTIME t2 = ControlLoop::get_time_ns_rt();
      jscontroltime += (t2-t1) / 1e6;
            
      if (log_controller_data) {
	data_recorded=true;
	data.push_back(t1); // record the current time
	data.push_back(timestep_factor);  // time factor
	if (jointstraj) {
	  data.push_back(jointstraj->curtime());  // traj time
	} else {
	  data.push_back(0);
	}
	for (int j=Joint::J1; j<=Joint::Jn; ++j) {
	  data.push_back(tc.q[j-1]);  // record target position
	  data.push_back(q[j]);         // record actual position
	  data.push_back(pid_torq[j]);  // record the pid torques
	}
      }

#ifndef OWDSIM
      if (safety_torques_exceeded(pid_torq)) {
	// hold here with zero target velocity and acceleration,
	// and wait until the limit condition goes away.
	if (jointstraj  // might have ended and been cleared (above)
	    && (jointstraj->state()
		== OWD::Trajectory::RUN)) { // only mess with the traj if
	  //                              we are still running

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
	  if (jointstraj->CancelOnStall) {
	    /*jointstraj->stop();*/ // still have to stop if the app wants to
	    jointstraj->runstate = Trajectory::ABORT;
	    last_traj_state = jointstraj->state();
	    for(int i = Joint::J1; i<=Joint::Jn; i++) {
	      heldPositions[i] = joints[i].q;
	    }
	    holdpos = true;  // should have been true already, but just making sure
	    delete jointstraj;
	    jointstraj = NULL;
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
	  jointstraj->run();
	}
	safety_hold=false;
      }
#endif // OWDSIM
    } catch (char *error) {
      // most likely a problem evaluating the trajectory, so
      // halt it and just hold position
      ROS_ERROR("Exception in trajectory: %s",error);
      ROS_ERROR("Aborting trajectory and holding current position");
      abort=true;
    } catch (const char *error) {
      // most likely a problem evaluating the trajectory, so
      // halt it and just hold position
      ROS_ERROR("Exception in trajectory: %s",error);
      ROS_ERROR("Aborting trajectory and holding current position");
      abort=true;
    } catch (...) {
      ROS_ERROR("Unknown exception; aborting trajectory and holding current position");
      abort=true;
    }
    if (abort) {
      last_traj_state = OWD::Trajectory::ABORT;
      delete jointstraj;
      jointstraj = NULL;
      for (int j=Joint::J1; j<Joint::Jn; ++j) {
	traj_torq[j]=0;  // make sure we don't leave these non-zero
      }
      for(int i = Joint::J1; i<=Joint::Jn; i++) {
	heldPositions[i] = joints[i].q;
	tc.q[i-1] = heldPositions[i];
      }
      holdpos = true;  // should have been true already, but just making sure

    }
      
  } else if (holdpos) {

    for(int i = Joint::J1; i <= Joint::Jn; i++) {
      tc.q[i-1] = heldPositions[i];
    }
          
    // ask the controllers to compute the correction torques.
    RTIME t1 = ControlLoop::get_time_ns_rt();
    newJSControl_rt((&tc.q[0])-1,q,dt,pid_torq);
    RTIME t2 = ControlLoop::get_time_ns_rt();
    jscontroltime += (t2-t1) / 1e6;
          
    if (log_controller_data) {
      data_recorded=true;
      data.push_back(t1); // record the current time
      data.push_back(0);  // time factor (zero for no traj)
      data.push_back(0); // trajectory time (zero for no traj)
      for (int j=Joint::J1; j<=Joint::Jn; ++j) {
	data.push_back(tc.q[j-1]);  // record target position
	data.push_back(q[j]);         // record actual position
	data.push_back(pid_torq[j]);  // record the pid torques
      }
    }

    if (safety_torques_exceeded(pid_torq) && slip_joints_on_high_torque) {
      // NEW WAY:

      for(int jj = Joint::J1; jj <= Joint::Jn; jj++) {
	double jointdiff = joints[jj].q - heldPositions[jj];
	// inch the target pos 10% towards the current pos
	heldPositions[jj] += 0.1f * jointdiff;
      }
    }

  } else {
    for(int i = Joint::J1; i <= Joint::Jn; i++) {
      pid_torq[i]=0.0f;  // zero out the torques otherwise
      last_control_position[i] = joints[i].q;
    }
  }
  if (++trajcount == 1000) {
    stats.trajtime = trajtime/1000.0;
    stats.jsctrltime=jscontroltime/1000.0;
    stats.safetycount=safety_torque_count;
    for (unsigned int i=0; i<Joint::Jn; ++i) {
      stats.hitorquecount[i]=safetytorquecount[i];
      stats.hitorqueavg[i]=safetytorquesum[i] / safetytorquecount[i];
      safetytorquesum[i]=safetytorquecount[i]=0;
    }
    trajcount=safety_torque_count=0;
    trajtime=jscontroltime=0.0f;
  }
        
     
  // compute the torque required to meet the desired qd and qdd
  // always calculate simulated dynamics (we'll overwrite dyn_torq later)
  JSdynamics(sim_torq, sim_links,(& tc.qd[0])-1, (&tc.qdd[0])-1); 
  if (data_recorded) {
    for (int j=Joint::J1; j<=Joint::Jn; ++j) {
      data.push_back(sim_torq[j]); // torques from sim model
    }
  }
  if(jsdyn){
    RTIME t4 = ControlLoop::get_time_ns_rt();
    JSdynamics(dyn_torq, links, (&tc.qd[0])-1, (&tc.qdd[0])-1); 
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
    for (int j=Joint::J1; j<=Joint::Jn; ++j) {
      data.push_back(dyn_torq[j]); // dynamic torques
    }
    for (int j=Joint::J1; j<=Joint::Jn; ++j) {
      data.push_back(tc.t[j-1]); // trajectory torques
    }
    recorder.add(data);
  }

        
  // finally, sum the control, dynamics, and trajectory torques
  for(int j=Joint::J1; j<=Joint::Jn; j++){
    // set the joint torques
    joints[j].trq(stiffness*pid_torq[j] + dyn_torq[j] + tc.t[j-1]);
  }
        
  jtrq2mtrq();         // results in motor::torque
        
  if(send_mtrq() == OW_FAILURE) {
    ROS_ERROR("WAM::control_loop: send_mtrq failed.");
  }
  if (bus->simulation) {
    // If we're running in simulation mode, then set the joint positions
    // as if they instantly moved to where we wanted.
    this->unlock();
    set_jpos((&tc.q[0])-1);
    this->lock_rt("control_loop");
  }

}

bool WAM::safety_torques_exceeded(double t[]) {
  //    static double safety_torqs[8]={0.0,10.0,10.0,5.0,5.0,2,2,2};
  static double safety_torqs[8]={0.0, 15.0, 30.0, 10.0, 15.0, 3.0, 3.0, 3.0};
  //static double safety_torqs[8]={0.0,50.0,75.0,75.0,60.0,15.0,15.0,15.0};
  bool bExceeded = false;
  if (!check_safety_torques) {
    return false;
  }
  for (int i = 1; i <= Joint::Jn; i++) {
    if (stiffness * stall_sensitivity * fabs(t[i])>safety_torqs[i]) {
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
    last_control_position[j]=q_target[j];
  }
  
}

int WAM::run_trajectory(OWD::Trajectory *traj) {
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
  //    this->lock("run_traj");
  jointstraj = traj;
  //    this->unlock("run_traj");
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

  //    this->lock("resume_traj");
  jointstraj->run();
  //    this->unlock("resume_traj");
  return OW_SUCCESS;
}

int WAM::cancel_trajectory() {
  this->lock("cancel_traj");
  if (!jointstraj) {
    // no trajectory exists
    this->unlock("cancel_traj");
    return OW_SUCCESS;
  }
  jointstraj->stop();

  char endstr[200];
  strcpy(endstr,"");
  for(int i = Joint::J1; i<=Joint::Jn; i++) {
    heldPositions[i] = joints[i].q;
    sprintf(endstr+strlen(endstr)," %1.4f",heldPositions[i]);
  }
  holdpos = true;  // should have been true already, but just making sure
  ROS_INFO_NAMED("AddTrajectory","Trajectory #%d has been cancelled; final reference position is [%s ]",
		 jointstraj->id,endstr);

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

  if (grab_lock) {
    this->lock("hold pos");
  }

  if(pulsetraj != NULL && !pulsetraj->done) {
    ROS_WARN("Cannot hold position, pulse trajectory is running\n");
    if (grab_lock) {
      this->unlock("hold pos");
    }
    return OW_FAILURE;
  }

  if(jointstraj != NULL) {
    jointstraj->stop();
  }

  if (!holdpos) {
    // we weren't already holding, so set the held position 
    // to the current position
    // (if we were already holding, then we won't change the target)
    for(int i = Joint::J1; i <= Joint::Jn; i++) {
      heldPositions[i] = joints[i].q;
      jointsctrl[i].reset();
      jointsctrl[i].run();
      suppress_controller[i] = false;
    }
  }
  
  if (jval != NULL) {
    // return the held positions
    for(int i = Joint::J1; i <= Joint::Jn; i++) {
      jval[i]=heldPositions[i];
    }
  }

  if (grab_lock) {
    this->unlock("hold pos");
  }
  
  if (holdpos) { 
    // we were already holding, so return an error
    return OW_FAILURE;
  }
  
  holdpos = true;
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
#ifdef OWD_RT
  rt_mutex_acquire(&rt_mutex,TM_INFINITE);
#else // ! OWD_RT
  pthread_mutex_lock(&mutex);
#endif // ! OWD_RT
  strncpy(last_locked_by,name,100);
  last_locked_by[99]=0; // just in case it was more than 99 chars long
  //if (name) {
    //static char msg[200];
    //      sprintf(msg,"OPENWAM locked by %s",name);
    //      syslog(LOG_ERR,msg);
    //    } else {
    //        syslog(LOG_ERR,"OPENWAM locked by (unknown)");
  //  }
}

bool WAM::lock_rt(const char *name) {
  static char last_locked_by[100];
#ifdef OWD_RT
  RTIME waittime(100000); // wait up to 100us for the lock
  if (rt_mutex_acquire(&rt_mutex,waittime)) {
    return false;
  }
#else // ! OWD_RT
  pthread_mutex_lock(&mutex);
#endif // ! OWD_RT
  strncpy(last_locked_by,name,100);
  last_locked_by[99]=0; // just in case it was more than 99 chars long
  //  if (name) {
  //    static char msg[200];
  //      sprintf(msg,"OPENWAM locked by %s",name);
  //      syslog(LOG_ERR,msg);
  //    } else {
  //        syslog(LOG_ERR,"OPENWAM locked by (unknown)");
  //  }
  return true;
}

void WAM::unlock(const char *name) {
  //  if (name) {
  //    static char msg[200];
  //    sprintf(msg,"OPENWAM unlocked by %s",name);
  //    syslog(LOG_ERR,msg);
  //  } else {
  //    syslog(LOG_ERR,"OPENWAM unlocked by (unknown)");
  //  }
#ifdef OWD_RT
  rt_mutex_release(&rt_mutex);
#else // ! OWD_RT
  pthread_mutex_unlock(&mutex);
#endif // ! OWD_RT
}

void WAMstats::rosprint(int recorder_count) const {
  ROS_DEBUG_NAMED("times",
                  "Loop: %2.2fms (read %2.1fms, control %2.1fms, send %2.1fms)",
                  looptime,
                  loopread,
                  loopctrl,
                  loopsend);
  if (slowcount > 0) {
    ROS_DEBUG_NAMED("times","Slow cycles %2.1f%% of the time (avg=%2.1fms, max=%2.1fms)",
                   slowcount, slowavg, slowmax);
    ROS_DEBUG_NAMED("times",
                   "  Slow breakdown: read %2.1fms, ctrl %2.1fms, send %2.1fms",
                   slowreadtime,
                   slowctrltime,
                   slowsendtime);
    if (slowmax > 90.0) {
      ROS_WARN_NAMED("times",
		     "Realtime control thread took %2.1fms", slowmax);
    }
  }
  if (missed_reads > 0) {
    ROS_DEBUG_NAMED("times","Incomplete read cycles %d",missed_reads);
  }
  ROS_DEBUG_NAMED("times",
                  "trajectory eval %2.2fms, jscontrol %2.2fms, safetycount=%d, recordercount=%d",
                  trajtime,jsctrltime,safetycount,recorder_count);
  if (safetycount > 0) {
    ROS_DEBUG_NAMED("times",
		    "Safety torque exception counts and averages:\n");
    for (unsigned int i=0; i<Joint::Jn; ++i) {
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
#ifndef OWDSIM // don't log data during simulations
  if (recorder.count > 0) {
    char filename[200];
    snprintf(filename,200,"/tmp/wamstats%02d-final.csv",bus->id);
    recorder.dump(filename);
  }
  if (jointstraj) {
    delete jointstraj;
    jointstraj=NULL;
  }
  if (bus) {
    delete bus;
    bus=NULL;
  }
#endif // ! OWDSIM
}

