/***********************************************************************

Author:        Kevin Knott (SRI), 2011

File:          HelixPlugin.cpp

Description:   This OWD Plugin creates a helical trajectory in the
               z-direction. It will stop upon force greater than the
               #defined value of Z_THRESHOLD in the z-direction. 

Note:          You are free to modify and reuse this code without
               restrictions.

Instructions:  To run this plugin, after typing rosmake in the folder 
               intel-pkg/gfe_owd_plugin, call the service StartHelixTraj with 
               the two parameters for the amplitude and pitch of the helix. To
               run the trajectory with a radius of 5 cm and a pitch of 10 cm,

               Ex. rosservice call right/owd/StartHelixTraj .05 .10

               To stop the plugin manually, call the service StopHelixTraj. 
               No parameters are needed when cancelling the trajectory. 

               Ex. rosservice call right/owd/StopHelixTraj
 
**********************************************************************/

#include "HelixPlugin.h"

//Save space for service
ros::ServiceServer HelixTraj::ss_Add;

/********************************************************************
Class:         JointsToTF

Function:      Constructor

Description:   Class for transforming from a given joint coordinate frame
               to WAM0, based on the dimensions from the WAM manual. The 
               constructor initializes values of the base coordinate 
               system. 

*/
// Begin JointsToTF Constructor

JointsToTF::JointsToTF()
{
  btQuaternion HALFPI_ROLL, NEG_HALFPI_ROLL;
  HALFPI_ROLL.setRPY(M_PI/2.0,0,0);
  NEG_HALFPI_ROLL.setRPY(-M_PI/2.0,0,0);

  wam_tf_base[0] = btTransform::getIdentity();
  wam_tf_base[1] = btTransform(NEG_HALFPI_ROLL);
  wam_tf_base[2] = btTransform(HALFPI_ROLL);
  wam_tf_base[3] = btTransform(NEG_HALFPI_ROLL,btVector3(0.045,0,0.55));
  wam_tf_base[4] = btTransform(HALFPI_ROLL,btVector3(-0.045,0,0));
  wam_tf_base[5] = btTransform(NEG_HALFPI_ROLL,btVector3(0,0,0.30));
  wam_tf_base[6] = btTransform(HALFPI_ROLL);
}

// End JointsToTF Constructor
//********************************************************************

/*********************************************************************
Class:         JointsToTF

Function:      Destructor

Description:   Class for transforming from a given joint coordinate frame
               to WAM0, based on the dimensions from the WAM manual. The 
               destructor currently does nothing. 

*/
// Begin JointsToTF Destructor

JointsToTF::~JointsToTF()
{
}

// End JointsToTF Destructor
//********************************************************************

/*********************************************************************
Class:         JointsToTF

Function:      GetTF

Description:   Class for transforming from a given joint coordinate frame
               to WAM0, based on the dimensions from the WAM manual. The 
               GetTF function retrieves the desired transform from the input
               joint. Joints 1 to 7 are transformed to wam0 using the 
               indices 0:6. 

Usage:         In order to use this function, send it a vector of joint 
               values with the current joint position coordinates and an 
               empty vector for the Transform. It will return a Transform
               vector with the transform for a given joint to wam0 saved in 
               each index from 0 to 6.  For example, to transform from joint
               7 to wam0, type:
                                
                             GetTF(joints,wamTransforms);
                             newpoint = wamTransforms[6] * point;

*/
// Begin GetTF

void JointsToTF::GetTF(std::vector<double> &Joints, std::vector<btTransform> &Transforms)
{
  btTransform toWam0;
  Transforms.resize(7);

  for (unsigned int i=0; i<NDOF; ++i) 
   {
     btQuaternion YAW;
     YAW.setRPY(0, 0, Joints[i]);
     btTransform wam_tf = wam_tf_base[i] *  btTransform(YAW);
     if (i==0)
       toWam0 = wam_tf;
     else
       toWam0 = toWam0 * wam_tf;

     Transforms[i] = toWam0;
   } 
}
// End GetTF
//********************************************************************


/*********************************************************************
Class:         HelixTraj

Function:      HelixTraj Constructor

Description:   The trajectory constructor sets the start and end
               positions to the current position of the WAM. It 
               instantiates the JointsToTF class, initializes values 
               to default, saves the initial transform for wam7, and
               handles all input value checks. 

Note 1:        We have to explicitly call the OWD::Trajectory constructor
               so that we can pass in the name of our trajectory for
               display in the trajectory queue.

Note 2:        The start_position must match either OWD's current
               position, if there is no active trajectory, or the
               end_position of the last trajectory in the queue. 
               Otherwise, OWD will reject the trajectory when you try to
               add it. 
*/
//BEGIN HelixTraj Constructor

HelixTraj::HelixTraj(double amplitude_in, double pitch_in)
  : OWD::Trajectory("GFE HelixTraj Trajectory"),
    JointsToTF(),
    amplitude(amplitude_in),
    pitch(pitch_in),
    stophelixtraj(false) {
  
  // Test to make sure GfePlugin has been initialized
  if(gfeplug){

     // Initialize a vector of size 7, with all zeros for zeroing out future parameters
     zeros = OWD::JointPos(std::vector<double>(NDOF,0.0));

     // Zero out the force/torque sensor and make sure it is tared
     if( gfeplug->ft_tare() ) {

#ifdef HP_VERBOSE
       ROS_INFO("The force/torque sensor has been tared by HelixTraj");
       ROS_INFO("The force in the x-direction is %4.2f", gfeplug->ft_force[0]);
       ROS_INFO("The force in the y-direction is %4.2f", gfeplug->ft_force[1]);
       ROS_INFO("The force in the z-direction is %4.2f", gfeplug->ft_force[2]);
       ROS_INFO("The force in the x-direction is %4.2f", gfeplug->ft_torque[0]);
       ROS_INFO("The force in the y-direction is %4.2f", gfeplug->ft_torque[1]);
       ROS_INFO("The force in the z-direction is %4.2f", gfeplug->ft_torque[2]);
#endif

     }
     else {
       ROS_WARN("The force/torque sensor has NOT been properly tared by HelixTraj");
     }
  
     // Check to make sure that the force/torque sensor has been properly tared
     const double max_tared_force = 0.5;
     const double max_tared_torque= 0.5;
     if ( (gfeplug->ft_force.size() < 3) || (gfeplug->ft_torque.size() < 3) ) {
       throw "HelixTraj requires that the Force/Torque sensor is installed and configured";
     }
     for (int i=0; i<3; i++) {
       if ( (gfeplug->ft_force[i] > max_tared_force) || (gfeplug->ft_torque[i] > max_tared_torque) ) {
         ROS_INFO("The excessive force calculated by the F/T sensor is %4.2f", gfeplug->ft_force[i]);
         ROS_INFO("The excessive torque calculated by the F/T sensor is %4.2f", gfeplug->ft_torque[i]);
	 throw "HelixTraj did not properly tare the F/T sensor in the current configuration";
       }
     }

#ifdef HP_SAFE
     // Check to make sure we are not close to a singularity, if in SAFE mode
     if (OWD::Kinematics::max_condition > 15) {
       throw "Arm is too close to a singularity for accurate force control;";
       throw "Please move it to a different configuration and try again";
     }
#endif

     // Set the start and end positions from the current position
     start_position = gfeplug->target_arm_position;
     end_position   = start_position;

     // Initialize all position, velocity and acceleration values
     prev_pos = start_position;
     prev_vel = zeros;
     prev_acc = zeros;

     // Initialize start position of arm as coordinate frame
     joints = start_position;
     ROS_INFO("The positions of the joints are %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f",
         joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6]);

     // Save all initial wam transforms to wam0 in this initial pose
     JointsToTF::GetTF( joints, wamTransforms);
   
     // Initialize desired velocity vector to all zeros
     for (int i=0; i<3; i++){
       vel_des.v[i] = 0.0;
       vel_des.w[i] = 0.0;
     }

     // Coerce amplitude and pitch inputs to within reasonable limits
     if (amplitude > AMPLITUDE_MAX){
       ROS_INFO("Amplitude input has exceeded AMPLITUDE_MAX and will be reduced to AMPLITUDE_MAX");
       amplitude = AMPLITUDE_MAX;
     }
     if (pitch > PITCH_MAX){
       ROS_INFO("Pitch input has exceeded PITCH_MAX and will be reduced to PITCH_MAX");
       pitch = PITCH_MAX;
     }

     // Initialize Joint Limits Array
     JointLimits[0][0] = -2.6;
     JointLimits[0][1] =  2.6;
     JointLimits[1][0] = -2.0;
     JointLimits[1][1] =  2.0;
     JointLimits[2][0] = -2.8;
     JointLimits[2][1] =  2.8;
     JointLimits[3][0] = -0.9;
     JointLimits[3][1] =  3.1;
     JointLimits[4][0] = -4.76;
     JointLimits[4][1] =  1.24;
     JointLimits[5][0] = -1.6;
     JointLimits[5][1] =  1.6;
     JointLimits[6][0] = -3.0;
     JointLimits[6][1] =  3.0;
 
   }
   else {
     throw "Could not get current WAM values from GfePlugin";
   }
     // Setup ROS communication
     ros::NodeHandle n("~");

     // Initialize service for stopping trajectory if/when asked by user
     ss_StopHelixTraj = n.advertiseService("StopHelixTraj", &HelixTraj::StopHelixTraj, this);
}

//END HelixTrajConstructor
//********************************************************************

/*********************************************************************
Class:         HelixTraj

Function:      HelixTraj Destructor

Description:   Currently does nothing but print completion message and 
               shut down the stopping service if called
*/
//BEGIN HelixTraj Destructor

HelixTraj::~HelixTraj() {

  // Shut down the Stopping service if it was used to stop
  ss_StopHelixTraj.shutdown();

  // Print Completion message
  ROS_INFO("HelixTraj::~HelixTraj: Helix Trajectory finished");

}

//END HelixTraj Destructor
//*********************************************************************

/**********************************************************************
Class:         HelixTraj

Function:      AddTrajectory

Description:   This is the handler for  ROS callbacks.  We register
               our own ROS service for getting requests from the client
               so that we have control over all the message parameters.
               OWD itself doesn't know anything about this service, and
               won't know anything about our trajectory type until we
               create one and ask OWD to run it using the 
               Trajectory::AddTrajectory function in the base class.

Note:          Always return true for the service call so that the client knows
               that the call was actually processed, as opposed to
               there being a network communication error.  The client
               will examine the "ok" field to see if the command was
               actually successful.  
*/
//BEGIN AddTrajectory

bool HelixTraj::AddTrajectory(gfe_owd_plugin::StartHelixTraj::Request &req,
			      gfe_owd_plugin::StartHelixTraj::Response &res) {

  ROS_INFO("GfePlugin: received Helix Trajectory service call");

  // Compute a new trajectory
  try {

    // Allocation for the pointer that will hold the single instantiation
    // of our Trajectory class. 
    HelixTraj *newtraj = new HelixTraj(req.amplitude,req.pitch);

    // Send the trajectory to the arm
    res.id = OWD::Plugin::AddTrajectory(newtraj,res.reason);
    if (res.id > 0) {
      res.ok=true;
    } else {
      res.ok=false;
    }
  } catch (const char *err) {
    res.ok=false;
    res.reason=err;
    res.id=0;
  }

  // Always return true for the service call so that the client knows that
  // the call was actually processed, as opposed to there being a
  // network communication error.
  return true;

}

//END AddTrajectory
//*********************************************************************
 
/**********************************************************************
Class:         HelixTraj

Function:      evaluate

Description:   When the trajectory is active, OWD will call evaluate at the
               control frequency (typically 500Hz) to get position, velocity,
               acceleration, and torque updates. 

Parameters:    There are two parameters, tc and dt. 

               tc (trajcontrol) - On input, tc contains current positions
               of the arm joints, and zero for all the velocity, acceleration
               and extra torque terms. On return, tc should contain the
               desired position (tc.q), velocity (tc.qd), acceleration (tc.qdd),
               and additional torque (tc.t) for each joint.
   
               dt - dt is the time that has elapsed since the previous call
               to evaluate().  The evaluate() function should increment
               the ::time variable by dt.

Returns:       tc (trajcontrol) - On return, tc should contain the desired
               position, velocity, acceleration, and additional torque for
               each joint. If the members of traj control are left 
               unmodified by evaluate(), no torques will be applied to the
               arm beyond gravity compensation, and the arm will be "free."

Note:          The evaluate() function should change runstate to DONE when
               the trajectory is completed.        
*/
//BEGIN evaluate

void HelixTraj::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
 
  time += dt;
    
  // If user initiated service call to StopHelix, stop trajectory and shutdown
  if (stophelixtraj) {
    ROS_INFO("User has cancelled the trajectory");

    // Keep tracking the current position of the arm to avoid OWD crash
    end_position = tc.q; 

    // Shut down the plugin
    runstate=OWD::Trajectory::DONE;

    return;
  }

  // Check to see if we have collided with the keypad (or something else...)
  if ( (gfeplug->ft_force[2]) < -Z_THRESHOLD ){
    ROS_INFO("Force in Z-Direction has exceeded Z_THRESHOLD, shutting down trajectory");
    ROS_INFO("The observed force in the Z-Direction is %4.2f", gfeplug->ft_force[2]);

    // Raise flag to shut down the trajectory
    stophelixtraj = true;

    return;
    }
    else {
    // Compute the Helix Trajectory
    HelixTraj::HelixTrajImplementation(tc, dt);
    }
   
  // Keep tracking the current position of the arm
  end_position = tc.q; 

  return;
}

//END evaluate
//*********************************************************************

/**********************************************************************
Class:            HelixTraj

Function:         HelixTrajImplementation

Description:      Creates a helical motion moving forward in the Z
                  direction, using the frame of WAM7 in its initial
                  pose when the plugin is called, which should be directly
                  in front of the desired endpoint. 

*/
// BEGIN HelixTrajImplementation

void HelixTraj::HelixTrajImplementation(OWD::Trajectory::TrajControl &tc, double dt) {

  // Create three zeroed out JointPos vectors of size 7 for the position, vel, and accel
  OWD::JointPos pos(std::vector<double>(NDOF,0.0));
  OWD::JointPos vel(std::vector<double>(NDOF,0.0));
  OWD::JointPos acc(std::vector<double>(NDOF,0.0));

  double distance = time * (pitch / (2*PI) );

  // Stop after traveling in Z-direction a distance of DISTANCE_THRESHOLD in meters
  if ( distance > DISTANCE_THRESHOLD) {
    // Drop some knowledge
    ROS_INFO("HelixTraj::HelixTrajImplementation: Trajectory has exceeded DISTANCE_THRESHOLD");
    ROS_INFO("The end-effector has moved a distance of %4.2f meters", distance);

    // Set the boolean flag to stop the trajectory
    stophelixtraj=true;

    // Quit here and return to calling function
    return;
  }

  // Frequency of sinusoidals - if needed to adjust, change #defined value
  static double f = FREQUENCY;

  // Velocity Vectors for helical motion in wam7 and wam0
  btVector3 wam7vel, wam0vel;

  // Slowly ramp up speed to avoid one large jump from zero
  double amp = (amplitude/3) * time;
  if (amp < amplitude){
    wam7vel[0] = - amp * 2 * PI * f * sin(2*PI*f*time);
    wam7vel[1] =   amp * 2 * PI * f * cos(2*PI*f*time); 
    wam7vel[2] =   (pitch / (2 * PI)) * (1-exp(-time));  
  }
  else{
  wam7vel[0] = - amplitude * 2 * PI * f * sin(2*PI*f*time);
  wam7vel[1] =   amplitude * 2 * PI * f * cos(2*PI*f*time); 
  wam7vel[2] =   (pitch / (2 * PI)) * (1-exp(-time)); 
  }     
 
#ifdef HP_VERBOSE
  static int counter = 0;
  counter++;
  if (counter == 500){
    ROS_INFO("The x-value of the requested velocity is %f", wam7vel[0]);
    ROS_INFO("The y-value of the requested velocity is %f", wam7vel[1]);
    ROS_INFO("The z-value of the requested velocity is %f", wam7vel[2]);
    counter = 0;
  }
#endif

  // Translate from initial WAM7 to wam0/base using wamTransforms(index 6 for wam7)
  btTransform transform(wamTransforms[6].getRotation(), btVector3(0,0,0));
  wam0vel = transform * wam7vel;

#ifdef HP_VERBOSE
 if (counter == 0){
    ROS_INFO("The x-value of the requested velocity IN WAM0 is %f", wam0vel[0]);
    ROS_INFO("The y-value of the requested velocity IN WAM0 is %f", wam0vel[1]);
    ROS_INFO("The z-value of the requested velocity IN WAM0 is %f", wam0vel[2]);
  }
#endif

  // Convert desired velocity into an R6 
  // vel_des = desired end-effector velocity and angular velocity in base frame
  // vel_des = [vx, vy, vz, wx, wy, wz]
  for (int i=0; i<3; i++){
    // Set velocities from above code for helical motion
    vel_des.v[i] = wam0vel[i];

    // Set all angular velocities to zero
    vel_des.w[i] = 0.0;
  }

  // Calculate dq_pose and vel
  // The Inverse Jacobian translates end-effector velocities to joint velocities in the base frame
  // dq_pose = vector of joint velocities = JacobianInverse * vector of end-effector velocities
  OWD::JointPos dq_pose = OWD::Plugin::JacobianPseudoInverse_times_vector(vel_des);

  // This next bit is just for error correcton:
  // Start position is ideal position. If we can get close to it, do it. 
  OWD::JointPos q_offset = start_position - tc.q;

  // ToDo: q_offset: hardcode preferred joint config instead of using start_position config
  // ToDo: q_offset: maybe use weighted nullspace to make base joints move less for rotation changes

  // Keep the arm away from its joint limits and near a desired configuration.
  // Otherwise, the arm will eventually drift toward the joint limits and stop moving.
  // Note: Due to redundancy in the 7 DOF WAM arm, vectors multiplied by the Jacobian which 
  // live in the nullspace will not produce any motion. 
  OWD::JointPos dq_offset = OWD::Plugin::Nullspace_projection(q_offset);

  // If there is an offset in our position, make the correction, which may need to be weighted
  vel = dq_pose * 1.0 + dq_offset * 1.0;

  // Coerce to within angular velocity limits, velocity limit arbitrarily set at safe limit
  double vel_max = 0.0;
  for (uint i = 0; i < vel.size(); i++) {
    if ( (fabs(vel[i]))  > vel_max) {
      vel_max = fabs(vel[i]);
    }
  }
  double vel_limit = VELOCITY_LIMIT;
  if (vel_max > vel_limit) {
    vel = vel*(vel_limit/vel_max);
  }
  
#ifdef HP_VERBOSE
  static int counter2 = 0;
  counter2++;
  if (counter2 == 500){
    ROS_INFO("The joint velocities are %f %f, %f, %f, %f, %f, %f",
             vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], vel[6]);
    counter2 = 0;
  }
#endif

  // ToDo: Maybe smooth velocity commands
  // BELOW EXAMPLE IN PYTHON FROM reactiveactions.py in newmanipapp/manipactions for smoothing velocity
  /*weight = 0.7
            if size(vels,0) > 1:
                for i in range(len(activedofs)):
                    velocities[activedofs[i]] = weight*velocities[activedofs[i]] + (1-weight)*vels[-1,i]
            vels = r_[vels, array([velocities[activedofs]])]
  */

  // Check for joint limits. Project arm 0.5sec into the future
  OWD::JointPos q_future = prev_pos + vel*0.5;
  // Stop arm if q_future exceeds joint limits
  /*if ( CheckExceedJointLimits(q_future) ){
    ROS_ERROR("HelixTraj::HelixTrajImplementation: q_future exceeds joint limits. Setting velocity to 0");
    vel = zeros;
    }*/

  // Set variables
  pos = prev_pos + vel*dt;
  acc = zeros;

  // Save values for next iteration
  prev_pos = pos;
  prev_vel = vel;
  prev_acc = acc;

#ifdef VERBOSE
  static int counter3 = 0;
  counter3++;
  if (counter3 == 500){
    ROS_INFO("The positions of the joints are %f, %f, %f, %f, %f, %f, %f",
             pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]);
    counter3=0;
  }
#endif

  // Set Command Values
  
  tc.q   = pos;
  tc.qd  = zeros;
  tc.qdd = zeros;
  tc.t   = zeros;
  
}
//END HelixTrajImplementation
//*********************************************************************


/**********************************************************************
Class:           HelixTraj

Function:        CheckExceedJointLimits

Description:     This function checks to see if the input position of
                 the arm exceeds the limits of any of the joints. The
                 input is typically a predicted future position of the
                 arm which can be used to alter the current projection
                 to avoid the joint limit, either by computing a new
                 trajectory or simpling stopping the arm prior to 
                 reaching a joint limit. 

Parameters:      This function inputs a JointPos vector, which holds
                 the position values of each one of the joints (typically
                 a projected future position). 

Returns:         This function returns a boolean. TRUE if one of the
                 values has exceeded a joint limit.  FALSE if all
                 positions are within joint limits. 

Notes:           Below are the joint limits from the WAM Manual:
                 http://wiki.barrett.com/support/wiki/WAMManual_Section9#9.2

   Joint	Positive Joint Limit Rad (deg)	Negative Joint Limit Rad (deg)
   1	        2.6 (150)               	-2.6 (-150)
   2	        2.0 (113)               	-2.0 (-113)
   3	        2.8 (157)               	-2.8 (-157)
   4 	        3.1 (180)               	-0.9 (-50)
   5	        1.24 (71)               	-4.76 (-273)
   6	        1.6 (90)                 	-1.6 (-90)
   7	        3.0 (172)               	-3.0 (-172) 

*/
//BEGIN CheckExceedJointLimits

bool HelixTraj::CheckExceedJointLimits(OWD::JointPos position) {

  //Compare input position vector and corresponding value in joint limit array
  for (int i=0; i<NDOF; i++){

    if( ( position[i] < JointLimits[i][0] ) || ( position[i] > JointLimits[i][1] ) )
      
      //Return true if value is outside of joint limit
      return true;
  }

  //Return false if all projected position values are within joint limits
  return false;
}
//END CheckExceedJointLimits
//*********************************************************************


/**********************************************************************
Class:         HelixTraj

Function:      Register

Description:   This helper function is called by our register_owd_plugin
               function when the plugin is first loaded.  It just sets
               up our ROS service for receiving trajectory requests.
*/
//BEGIN Register 

bool HelixTraj::Register() {
  ros::NodeHandle n("~");
  ss_Add = n.advertiseService("StartHelixTraj",&HelixTraj::AddTrajectory);
  return true;
}

//END Register
//*********************************************************************
 

/**********************************************************************
Class:         HelixTraj

Function:      Shutdown

Description:   Our unregister_owd_plugin function will call this when
               OWD shuts down. By shutting down our ROS service we will
               remove its listing from the ROS master.
*/
//BEGIN Shutdown

void HelixTraj::Shutdown() {
  ss_Add.shutdown();
}

//END Shutdown
//*********************************************************************


/**********************************************************************
Class:           HelixTraj

Function:        StopHelixTraj

Description:     Stop the Trajectory when asked by the client

*/
// BEGIN StopHelixTraj

bool HelixTraj::StopHelixTraj(gfe_owd_plugin::StopHelixTraj::Request &req,
			      gfe_owd_plugin::StopHelixTraj::Response &res) {
  
  stophelixtraj=true;
  return true;
}

// END StopHelixTraj
//*********************************************************************

