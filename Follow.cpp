/***********************************************************************

  Copyright 2011 Carnegie Mellon University

  Author:        Kyle Strabala <strabala@cmu.edu>

  Modifications: Kevin Knott, SRI

***********************************************************************/

#include "Follow.h"

// Allocate storage for ss_Add, a static member of the Follow class
ros::ServiceServer Follow::ss_Add;

/**********************************************************************
Class:           Follow

Function:        Class Constructor

Description:     The constructor sets the start position to the current
                 position of the WAM arm. It initializes all values of
                 position, velocity and acceleration for each of the 
                 joints. Velocity and Acceleration are set to zero, 
                 position is just the current start position. 

                 If the service call requests mode_joy to put the 
                 program in joystick mode, it initializes things for joy
                 teleop (SetupTrajForJoyTeleop()). If an unknown value
                 is requested, it prints an error and stops the plugin. 

                 The constructor also sets up communication with ROS. 
                 It will publish a message "follow_joint_state" and start
                 a service for collision avoidance. 

                 A new thread will be initialized using StartRosCommThread. 
 */
// BEGIN Follow Constructor

Follow::Follow(int mode_in): OWD::Trajectory("Follow"), mode(mode_in), whatWeAreControlling(VELOCITY), jitter(true), stopfollow(false) {
  
  //Initialize a vector of size 7, with all zeros
  zeros = OWD::JointPos(std::vector<double>(FOLLOW_NDOF,0.0));

  // If the start position doesn't match the WAM's current position,
  // OWD will reject the trajectory.  These help to let other plugins know
  // what we are doing and can be accessed by them. 
 
  // Note how we get the current position by using our plugin class.
  // if we didn't have an instantiation of our plugin class, we could
  // have also just called OWD::Plugin::target_arm_position(), since it
  // is a static class member.
  start_position = gfeplug->target_arm_position;
  end_position = start_position;
  
  // Initialize all position, velocity and acceleration values
  prev_pos = start_position;
  prev_vel = zeros;
  prev_acc = zeros;

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

  // Initialize desired velocity vector to all zeros
  for (int i=0; i<3; i++){
    vel_des.v[i] = 0.0;
    vel_des.w[i] = 0.0;
  }

  // Seed the random number generator
  srand((unsigned)time); 

  // If mode requested is for joystick control, initialize setup
  // Else print error and shut down plugin
  switch(mode) {
    case gfe_owd_plugin::StartFollow::Request::mode_joy:
      Follow::SetupTrajForJoyTeleop();
      break;
    default:
      mode = 0;
      ROS_ERROR("Follow::Follow: unknown mode integer %d (None=0, Joy=%d). Stopping trajectory.",
                 mode,
                 gfe_owd_plugin::StartFollow::Request::mode_joy);

      runstate=OWD::Trajectory::DONE;
  }
  
  // Setup ROS communication to publish what we are doing to anyone wanting to listen
  ros::NodeHandle n("~");
  joint_state_pub = n.advertise<sensor_msgs::JointState>("follow_joint_state", 1);
  last_joint_state_pub_time = 0.0;

  // Initialize check for arm configuration to see if outside limits or going to collide
  openrave_service = n.serviceClient<pr_msgs::ArmConfigCheck>("/manipulationapplet/check_for_collision", true);
  last_good_pos = start_position;
  problem = false;

  // Initialize service for stopping trajectory if/when asked by user
  ss_StopFollow = n.advertiseService("StopFollow", &Follow::StopFollow, this);
  
  // Create a new thread, use default attributes, use StartRosCommThread as the start routine
  int rc = pthread_create(&ros_comm_thread, NULL, &Follow::StartRosCommThread, this);

  // If pthread_create is successful, it returns 0, otherwise it returns an error value
  if (rc){
     printf("Error creating ROS comm thread. Return code from pthread_create() is %d\n", rc);
     exit(-1);
  }
  
  // Initialize the joy_msg_mutex mutex variable with default attributes, initially unlocked
  // Both ROS Callback function and evaluate threads look at this, so necessary to mutex
  pthread_mutex_init(&joy_msg_mutex,NULL);
  pthread_mutex_init(&roscomm_mutex,NULL);

}

// END Follow Constructor
//************************************************************************

/*************************************************************************
Class:         Follow

Function:      Class Destructor

Description:   The destructor shuts down publishing and subscription nodes. 
               It also shuts down the service which is used for stopping the 
               trajectory. 

 */
// BEGIN Follow Destructor

Follow::~Follow() {

  // stop the RosComm thread from trying to publish anything
  pthread_mutex_lock(&roscomm_mutex); 

  // Shut down the service for stopping our trajectory
  ss_StopFollow.shutdown();

  // Set to mode_none
  mode = 0;

  // Shut down publisher (I don't think this is necessary?)
  joint_state_pub.shutdown();

  // Shut down joystick subscriber
  joy_sub.shutdown();

  // Do we need to destroy the pthread and mutex variable??
  pthread_mutex_destroy(&joy_msg_mutex);
  pthread_exit(NULL);

  // Print Completion message
  ROS_INFO("Follow::~Follow: trajectory finished");
}

// END Follow Destructor
//**********************************************************************

/***********************************************************************
Class:          Follow

Function:       AddTrajectory

Description:    This is the handler for the ROS callbacks. This is where
                the program enters after a ros service call is sent a value
                of 1. It tries to open a new Follow instantiation and add the
                trajectory to OWD so we can call the evaluate function. 
*/
// BEGIN AddTrajectory

bool Follow::AddTrajectory(gfe_owd_plugin::StartFollow::Request &req,
			   gfe_owd_plugin::StartFollow::Response &res) {

  ROS_INFO("Follow::AddTrajectory: received StartFollow service call: %d", req.mode);

  // Compute a new trajectory
  try {

    // Allocation for the pointer that will hold the single instantiation
    // of our Trajectory class. Input the joystick mode variable. 
    Follow *newtraj = new Follow(req.mode);

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

// END AddTrajectory
//************************************************************************

/*************************************************************************
Class:          Follow

Function:       evaluate

Description:    OWD will call the evaluate function at 500 Hz while the 
                trajectory is running. 

Inputs:         tc:   The TrajControl Class has members q, qd, qdd, and t
                      -q = vector of size NDOF with joint positions
                      It contains the current joint positions on input and
                      on return should contain the desired target position
                      in radians. If unchanged, the torque values will 
                      remain zero and the arm will be "free" to move

                      -qd = vector of size NDOF with joint velocities
                      It contains all zeros on input and should contain
                      the current velocities that the trajectory is moving
                      each joint, in radians/sec. The velocities will be 
                      passed to the dynamics model for calculating feed-
                      forward compensation torques

                      -qdd = vector of size NDOF with joint accelerations
                      It contains all zeros on input and should contain
                      the current accelerations in radians/sec*sec that the 
                      trajectory is accelerating each joint. The accelerations
                      will be passed to the dynamics model for calculating
                      feed-forward compensation torques. 

                      -t = vector of size NDOF with joint torque values. 
                      It contains all zeros on input and should contain the
                      pure torques that it wants applied to each joint in 
                      Newton-meters. The torques will be added to the PID
                      torques (calculated by any changes made to q) and the
                      torques will be added to the dynamic torques (calculated
                      by any changes made to qd and qdd) before being sent to
                      the motors. 

                dt    A value of the elapsed time since the last call to evaluate.
*/
// BEGIN evaluate

void Follow::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {

  // Incremement the time
  time += dt;

  // If user initiated service call to StopFollow, stop trajectory and shutdown
  if (stopfollow) {
    ROS_INFO("StopFollow has cancelled the trajectory");

    // Keep tracking the current position of the arm to avoid OWD crash
    end_position = tc.q; 

    // Shut down the plugin
    runstate=OWD::Trajectory::DONE;

    return;
  }

  // If mode is mode_joy for joystick control, run the joystick teleoperation
  // else exit plugin with error message
  switch(mode) {
    case gfe_owd_plugin::StartFollow::Request::mode_joy:
      Follow::TrajForJoyTeleop(tc, dt);
      break;
    default:
      tc.q   = tc.q;
      tc.qd  = zeros;
      tc.qdd = zeros;
      tc.t   = zeros;
      ROS_ERROR("Follow::evaluate: unknown mode integer %d (None=0, Joy=%d). Stopping trajectory.",
                    mode,
                    gfe_owd_plugin::StartFollow::Request::mode_joy);
      runstate=OWD::Trajectory::DONE;
      return;
  }
  
  // Keep tracking the current position of the arm
  end_position = tc.q; 

  return;
}

// END evaluate
//*******************************************************************************

/********************************************************************************
Class:          Follow

Function:       Register

Description:    This helper function is called by our register_owd_plugin function
                when the plugin is first loaded.  It just sets up our ROS service
                for receiving trajectory requests.
*/
// BEGIN Register

bool Follow::Register() {
  ros::NodeHandle n("~");
  ss_Add = n.advertiseService("StartFollow",&Follow::AddTrajectory);
  return true;
}

// END Register
//*******************************************************************************

/********************************************************************************
Class:          Follow

Function:       Shutdown

Description:    Our unregister_owd_plugin function will call this when OWD shuts down.
                By shutting down our ROS service we will remove its listing from the
                ROS master.
 */
// BEGIN Shutdown


void Follow::Shutdown() {

  // Shutdown all services initialized in the Register function
  ss_Add.shutdown();
}

// END Shutdown
//********************************************************************************

/*********************************************************************************
Class:          Follow

Function:       SetupTrajForJoyTeleop

Description:    This function subscribes to the joy msg, which contains
                the X-box controller's configuration.  It is a joy/Joy type msg of the
                form
                       float32[] axes
                       int32[] buttons
 */
// BEGIN SetupTrajForJoyTeleop

void Follow::SetupTrajForJoyTeleop() {

  last_joy_msg_time = -1.0;
  ros::NodeHandle n("~");
  joy_sub = n.subscribe("/joy", 1, &Follow::JoyCallback, this);
}

// END SetupTrajForJoyTeleop
//********************************************************************************

/*********************************************************************************
Class:           Follow

Function:        JoyCallback

Description:     This is the callback function for the joystick subscriber. It first
                 locks the mutex topic, then records the message for use elsewhere 
                 before unlocking the mutex. 

*/
// BEGIN JoyCallback

void Follow::JoyCallback(joy::Joy msg) {

  // Make sure we're the only ones changing the joystick message
  pthread_mutex_lock(&joy_msg_mutex);

  // Update the message and time of message
  last_joy_msg_time = time;
  last_joy_msg = msg;

  // Unlock the shared data again
  pthread_mutex_unlock(&joy_msg_mutex);

}

// END JoyCallback
//*********************************************************************************

/**********************************************************************************
Class:            Follow

Function:         TrajForJoyTeleop

Description:      

*/
// BEGIN TrajforJoyTeleop

void Follow::TrajForJoyTeleop(OWD::Trajectory::TrajControl &tc, double dt) {

  // If message is old or during a stall, don't move
  if (time - last_joy_msg_time > 0.5) {
    // ToDo: What about during stalls? dt=0, so this may never execute.
    tc.q   = tc.q;           //Position of joints is not changed
    tc.qd  = zeros;          //Zero out vel input
    tc.qdd = zeros;          //Zero out accel input
    tc.t   = zeros;          //Zero out torq input
    prev_pos = tc.q;         //Save previous position of joints
    prev_vel = zeros;        //Zero out previous vel
    prev_acc = zeros;        //Zero out previous accel
    return;
  }

  // Copy latest message
  // joy_msg.buttons = [btn0, btn1, btn2, btn3, btn4, btn5, btn6, btn7, btn8, btn9, btn10]
  // joy_msg.axes = [axes0, axes1, axes2, axes3, axes4, axes5, axes6, axex7]
  pthread_mutex_lock(&joy_msg_mutex);
  joy::Joy joy_msg = last_joy_msg;
  pthread_mutex_unlock(&joy_msg_mutex);

  // Declare and initialize to zero the previous and current states of button 7
  static bool previous_button_7 = 0;
  static bool current_button_7 = 0;

  // Update the current state of button 7
  current_button_7 = joy_msg.buttons[7];

  // If Windows button 7 pressed (large central green button), toggle control between
  // velocity and angular velocity (but only on rising edge)
  if ( (current_button_7 == 1) && (current_button_7 != previous_button_7) ){
    switch(whatWeAreControlling)
    {
    case VELOCITY:
      {
	whatWeAreControlling = ANGULAR_VELOCITY;
	ROS_INFO("Button 7 Pressed: whatWeAreControlling = Angular Velocity");
	break;
      }
    case ANGULAR_VELOCITY:
      {
	whatWeAreControlling = VELOCITY;
	ROS_INFO("Button 7 Pressed: whatWeAreControlling = Velocity");
	break;
      }
    default:
      {
	ROS_WARN("Button 7 Pressed: unknown value of whatWeAreControlling");
	break;
      }
    }
  }

  // Declare and initialize to zero the previous and current states of button 6 
  static bool previous_button_6 = 0;
  static bool current_button_6 = 0;

  // Update the current state of button 6
  current_button_6 = joy_msg.buttons[6];

  // If Start button 6 pressed (labeled "Start"), toggle jittering of end-effector
  // TODO: Account for switch debounce
  if ( (current_button_6 == 1) && (current_button_6 != previous_button_6) ){
    switch(jitter)
    {
    case false:
      {
	jitter = true;
	ROS_INFO("Button 6 Pressed: jitter = true");
	break;
      }
    case true:
      {
	jitter = false;
	ROS_INFO("Button 6 Pressed: jitter = false");
	break;
      }
    default:
      {
	ROS_WARN("Button 6 Pressed: unknown value of jitter");
	break;
      }
    }
  }

  // Update the previous states of button 6 and button 7
  previous_button_6 = current_button_6;
  previous_button_7 = current_button_7;

  // Create three zeroed out JointPos vectors of size 7 for the position, vel, and accel
  OWD::JointPos pos(std::vector<double>(FOLLOW_NDOF,0.0));
  OWD::JointPos vel(std::vector<double>(FOLLOW_NDOF,0.0));
  OWD::JointPos acc(std::vector<double>(FOLLOW_NDOF,0.0));

  // vel_des = desired end-effector velocity and angular velocity
  // vel_des = [vx, vy, vz, wx, wy, wz]
  if (whatWeAreControlling == ANGULAR_VELOCITY){
    // Give angular velocity commands for rotation
    vel_des.v[0] = 0.0;
    vel_des.v[1] = 0.0;
    vel_des.v[2] = 0.0;
    vel_des.w[0] = joy_msg.axes[0];
    vel_des.w[1] = joy_msg.axes[1];
    vel_des.w[2] = joy_msg.axes[4];
  } 
  else {
    // Give regular velocity commands for translation
    vel_des.v[0] = -joy_msg.axes[0];
    vel_des.v[1] = -joy_msg.axes[1];
    vel_des.v[2] = -joy_msg.axes[4];
    vel_des.w[0] = 0.0;
    vel_des.w[1] = 0.0;
    vel_des.w[2] = 0.0;
  }

  // Threshold for controller to prevent unwanted movement if joystick gets stuck
  for(int i=0; i< 3; i++){
    if ( fabs(vel_des.v[i]) < 0.2){
      vel_des.v[i] = 0.0;
    }
    if ( fabs(vel_des.w[i]) < 0.2){
      vel_des.w[i] = 0.0;
    }
  }

  // Adjust for the full scale value of the X-box controller. 
  double joy_scale = 1.0;
  for(int i = 0; i < 3; i++) {
    // Set the vel_sign value = 1 if vel_des[i] is positive and -1 if vel_des[i] is negative
    double vel_sign = (vel_des.v[i] >= 0.0)?1.0:-1.0;

    // Scale to a desired max velocity along with making an exponential relationship to allow
    // for precise positioning.
    vel_des.v[i] = joy_scale*vel_sign*(vel_des.v[i]/FOLLOW_FULL_SCALE_JOYSTICK)*(vel_des.v[i]/FOLLOW_FULL_SCALE_JOYSTICK);

    // repeat for .w members
    vel_sign = (vel_des.w[i] >= 0.0)?1.0:-1.0;
    vel_des.w[i] = joy_scale*vel_sign*(vel_des.w[i]/FOLLOW_FULL_SCALE_JOYSTICK)*(vel_des.w[i]/FOLLOW_FULL_SCALE_JOYSTICK);

  }

  // Calculate dq_pose and vel
  // The Inverse Jacobian translates end-effector velocities to joint velocities in the base frame
  // dq_pose = vector of joint velocities = JacobianInverse * vector of end-effector velocities
  OWD::JointPos dq_pose = OWD::Plugin::JacobianPseudoInverse_times_vector(vel_des);

  // Start position is ideal position. If we can get close to it, do it. 
  OWD::JointPos q_offset = start_position - tc.q;

  // ToDo: q_offset: hardcode preferred joint config instead of using start_position config
  // ToDo: q_offset: maybe use weighted nullspace to make base joints move less for rotation changes

  // Calculate the Nullspace joint correction:
  // Keep the arm away from its joint limits and near a desired configuration.
  // Otherwise, the arm will eventually drift toward the joint limits and stop moving.
  // Note: Due to redundancy in the 7 DOF WAM arm, vectors multiplied by the Jacobian which 
  // live in the nullspace will not produce any motion. 
  OWD::JointPos dq_offset = OWD::Plugin::Nullspace_projection(q_offset);

  // Weight how much we go toward the ideal position.
  vel = dq_pose * 1.0 + dq_offset * 1.0;

  // ToDo: Maybe smooth velocity commands
  // BELOW EXAMPLE IN PYTHON FROM reactiveactions.py in newmanipapp/manipactions for smoothing velocity
  /*weight = 0.7
            if size(vels,0) > 1:
                for i in range(len(activedofs)):
                    velocities[activedofs[i]] = weight*velocities[activedofs[i]] + (1-weight)*vels[-1,i]
            vels = r_[vels, array([velocities[activedofs]])]
  */

  // Coerce to within angular velocity limits, velocity limit arbitrarily set at safe limit
  double vel_max = 0.0;
  for (uint i = 0; i < vel.size(); i++) {
    if ( (fabs(vel[i]))  > vel_max) {
      vel_max = fabs(vel[i]);
    }
  }
  double vel_limit = FOLLOW_VELOCITY_LIMIT;
  if (vel_max > vel_limit) {
    vel = vel*(vel_limit/vel_max);
  }
  
  // If a collision is imminent, stop the arm
  if (problem == true) {
    if (vel*(last_good_pos - tc.q) < 0) {
      vel = zeros;
    }
  }
  
  // Check for joint limits. Project arm 0.5sec into the future
  OWD::JointPos q_future = prev_pos + vel*0.5;
  //***TODO, Stop arm if q_future exceeds joint limits
  if ( CheckExceedJointLimits(q_future) ){
    ROS_ERROR("Follow::TrajForJoyTeleop: q_future exceeds joint limits. Setting velocity to 0");
    vel = zeros;
  }

  // Set variables
  pos = prev_pos + vel*dt;
  acc = zeros;

  // Counter for jitter function, initialized one time to zero, then counts up
  static int counter = 0;
  counter++;

  // Save values for next iteration
  if ( (jitter==true) && (counter >= counter_limit) ){
    // Create three vectors for holding the noise information
    OWD::JointPos position_noise(std::vector<double>(FOLLOW_NDOF,0.0));
    OWD::JointPos velocity_noise(std::vector<double>(FOLLOW_NDOF,0.0));
    OWD::JointPos acceleration_noise(std::vector<double>(FOLLOW_NDOF,0.0));

    // Retrieve values of noise to add to pos, vel, and acc
    Jitter(position_noise, velocity_noise, acceleration_noise);

    // Add the noise
    pos += position_noise;
    vel += velocity_noise;
    acc += acceleration_noise;

    // Reset the counter
    counter = 0;
  } 

  // Save values for next iteration
  prev_pos = pos;
  prev_vel = vel;
  prev_acc = acc;

  // Set Command Values
  tc.q   = pos;
  tc.qd  = zeros;
  tc.qdd = zeros;
  tc.t   = zeros;
  
}
// END TrajForJoyTeleop
//********************************************************************************


/*********************************************************************************
Class:           Follow

Function:        Jitter

Description:     This function creates a JointPos vector with random noise. It 
                 requires that the random number generator has been seeded prior 
                 to calling. 

Parameters:      This function has three inputs. The inputs must have been initialized
                 outside of this function call. 

Returns:         This function returns the three inputs with values for the noise
                 for position, velocity and acceleration where velocity is the
                 derivative of position, and acceleration is the derivative of the
                 velocity. 

*/
// BEGIN Jitter

void Follow::Jitter(OWD::JointPos &noise_pos, OWD::JointPos &noise_vel, OWD::JointPos &noise_accel) {

  // Jitter the wrist
  noise_pos[6] = FOLLOW_JITTER_SCALE * sin( rand() % 10000 );

  // TODO: Add values to velocity and acceleration noise (derivatives of position noise)
  noise_vel = zeros;
  noise_accel = zeros;

}

// END Jitter
//********************************************************************************

/*********************************************************************************
Class:           Follow

Function:        StopFollow

Description:     Stop the Trajectory when asked by the client

*/
// BEGIN StopFollow

bool Follow::StopFollow(gfe_owd_plugin::StopFollow::Request &req,
			gfe_owd_plugin::StopFollow::Response &res) {
  stopfollow=true;
  return true;
}

// END StopFollow
//*******************************************************************************


/*********************************************************************************
Class:           Follow

Function:        CheckExceedJointLimits

Description:     This function checks to see if the input position of the arm 
                 exceeds the limits of any of the joints. The input is typically
                 a predicted future position of the arm which can be used to 
                 alter the current projection to avoid the joint limit, either 
                 by computing a new trajectory or simpling stopping the arm prior
                 to reaching a joint limit. 

Parameters:      This function inputs a JointPos vector, which holds the position
                 values of each one of the joints (typically a projected future
                 position). 

Returns:         This function returns a boolean. TRUE if one of the values has exceeded
                 a joint limit.  FALSE if all positions are within joint limits. 

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
// BEGIN CheckExceedJointLimits

bool Follow::CheckExceedJointLimits(OWD::JointPos position) {

  // Compare input position vector and corresponding value in joint limit array
  for (int i=0; i<FOLLOW_NDOF; i++){

    if( ( position[i] < JointLimits[i][0] ) || ( position[i] > JointLimits[i][1] ) ){
      
      // Return true if value is outside of joint limit
      return true;
    }
  }

  // Return false if all projected position values are within joint limits
  return false;
}
// END CheckExceedJointLimits
//********************************************************************************


/*********************************************************************************
Class:           Follow

Function:        StartRosCommThread

Description:     This handles communication with ROS by running the RosComm function.
                 This just communicates our trajectory to the outside world if 
                 anyone wants to listen in. It currently provides no other functionality.

*/
// BEGIN StartRosCommThread

void *Follow::StartRosCommThread(void *data) {
  Follow *f = (Follow*) data;
  f->RosComm();
  return NULL;
}

// END StartRosCommThread
//*********************************************************************************

/**********************************************************************************
Class:           Follow

Function:        RosComm

Description:     This function handles sending the values of the pos, vel and accel
                 as a joint_state_msg. Uses newmanipap to avoid collisions. This
                 isn't necessary in teleoperation since typically the user is
                 responsible for obstacle avoidance and safety, but it is provided
                 anyway, for testing safely if nothing else. 
*/
// BEGIN RosComm

void Follow::RosComm() {
  while (mode != 0) {

    // only if we are not shutting down
    if (pthread_mutex_trylock(&roscomm_mutex) == 0) {

      // Create a message of position, velocity and acceleration information
      sensor_msgs::JointState joint_state_msg;
      joint_state_msg.header.frame_id = ros::this_node::getName();
      joint_state_msg.header.stamp    = ros::Time::now();
      joint_state_msg.position = prev_pos;
      joint_state_msg.velocity = prev_vel;
      joint_state_msg.effort = prev_acc;
      
      // Publish the joint state message with pos, vel, and accel information    
      joint_state_pub.publish(joint_state_msg);
      last_joint_state_pub_time = time;
      
      if(openrave_service) {
	
	// Create the service to check the configuration of the arm
	pr_msgs::ArmConfigCheck srv;
	srv.request.joint_state = joint_state_msg;
	if (openrave_service.call(srv)) {
	  bool self_collision = srv.response.current_self_collision || srv.response.future_self_collision;
	  bool env_collision = srv.response.current_env_collision || srv.response.future_env_collision;
	  bool joint_limits = srv.response.current_joint_limits_exceeded || srv.response.future_joint_limits_exceeded;
	  problem = self_collision || env_collision || joint_limits;
	  if (problem == false) {
	    last_good_pos = joint_state_msg.position;
	  }
	  ROS_DEBUG("Problem: %s", problem?"True":"False");
	} else {
	  ROS_DEBUG("openrave_service did not execute properly.");
	  problem = false;
	}
      } else {
	ROS_DEBUG("openrave_service is not running.");
	problem = false;
      }
     
      pthread_mutex_unlock(&roscomm_mutex);
    }
    if (mode == 0) break;
    sleep(0.1);
  }
}

// END RosComm
//*********************************************************************************
