/***********************************************************************

Author:        Kevin Knott (SRI), 2011

File:          MoveDirection.cpp

Description:   This OWD Plugin creates a straight forward trajectory 
               in any arbitrary direction (x, y, or z) in the coordinate
               system of the end-effector (workspace). The user will 
               input the distance that they would like to move in the 
               given direction in meters. The trajectory will
               stop upon force greater than the #defined value of the 
               FORCE_THRESHOLD depending upon the direction of motion. 
               The force must be in the direction orthogonal to that of
               the direction of motion in order to stop the arm. 
               The user will also input a velocity value for the speed
               of the arm in meters per second. The last value to be input
               is a boolean flag to determine if the trajectory should be 
               run with or without compliance. Without compliance, the arm
               will move in the direction that the palm is facing before 
               the trajector is called, and there will be no adjustment. 
               If compliance is being utilized, the coordinate system of 
               the palm will be updated regularly, so if the palm should
               change direction (either intentionally or not), the direction
               of the trajectory will be updated to the new coordinate system
               of the palm (useful for opening a door knob, for instance). 

Note:          You are free to modify and reuse this code without
               restrictions.

Instructions:  To run this plugin, after typing rosmake in the folder 
               intel-pkg/gfe_owd_plugin, call the service StartMoveDirection
               with the 4 parameters for the direction, distance, and
               velocity along with the desired compliance to include in the 
               trajectory. To move the arm in the Z-direction (first
               parameter = 0 0 1), a distance of 5 cm (second parameter is 0.05 m), 
               at a speed of 1 cm per second (third parameter is 0.01) and
               to run without compliance (fourth parameter is 0), enter the 
               following command. 

               Ex: rosservice call /right/owd/StartMoveDirection 0 0 1 0.05 0.01 0

               To stop the plugin manually, call the service StopMoveDirection. 
               No parameters are needed when cancelling the trajectory.   

               Ex: rosservice call /right/owd/StopMoveDirection
 
**********************************************************************/

#include "MoveDirection.h"

//Save space for service
ros::ServiceServer MoveDirection::ss_Add;

/*********************************************************************
Class:         MoveDirection

Function:      MoveDirection Constructor

Description:   The trajectory constructor sets the start and end
               positions to the current position of the WAM. It 
               instantiates the JointsToTF class, initializes values 
               to default, saves the initial transform for wam7, and 
               handles all input values. 

Note 1:        We have to explicitly call the OWD::Trajectory constructor
               so that we can pass in the name of our trajectory for
               display in the trajectory queue.

Note 2:        The start_position must match either OWD's current
               position, if there is no active trajectory, or the
               end_position of the last trajectory in the queue. 
               Otherwise, OWD will reject the trajectory when you try to
               add it. 
*/
//BEGIN MoveDirection Constructor

MoveDirection::MoveDirection(double direction_in_x,
                             double direction_in_y,
                             double direction_in_z,
			     double distance_in,
			     double velocity_in,
			     bool compliance_in)
  : OWD::Trajectory("GFE MoveDirection Trajectory"),
    JointsToTF(),
    compliance(compliance_in),
    distance(distance_in),
    velocity(velocity_in),
    stopmovedirection(false){
  
  // Test if GfePlugin has been initialized
  if(gfeplug){

     // Initialize a vector of size 7, with all zeros for zeroing out future parameters
     zeros = OWD::JointPos(std::vector<double>(NDOF,0.0));

     // Save the initial end_point origin for determining the distance traveled
     initial_endpoint_origin = (R3)gfeplug->endpoint;

     // Zero out the force/torque sensor and make sure it is tared
     if( gfeplug->ft_tare() ) {
       ROS_INFO("The force/torque sensor has been tared by MoveDirection");
     }
     else { 
       ROS_WARN("The force/torque sensor has NOT been properly tared by MoveDirection");
     }

     // Fill and normalize the input vector
     direction[0] = direction_in_x;
     direction[1] = direction_in_y;
     direction[2] = direction_in_z;
     direction = direction.normalize();
      
#if 0
     // Check to make sure that the force/torque sensor has been properly tared
     const double max_tared_force = 0.5;
     const double max_tared_torque= 0.5;
     if ( (gfeplug->ft_force.size() < 3) || (gfeplug->ft_torque.size() < 3) ) {
       throw "MoveDirection requires that the Force/Torque sensor is installed and configured";
     }
     for (int i=0; i<3; i++) {
       if ( (gfeplug->ft_force[i] > max_tared_force) || (gfeplug->ft_torque[i] > max_tared_torque) ) {
	 ROS_INFO("The excessive force calculated by the F/T sensor is %f", gfeplug->ft_force[i]);
         ROS_INFO("The excessive torque calculated by the F/T sensor is %f", gfeplug->ft_torque[i]);
	 throw "HelixTraj did not properly tare the F/T sensor in the current configuration";
       }
     }
#endif

#ifdef MD_SAFE
     // Check to make sure we are not too close to a singularity
     if (OWD::Kinematics::max_condition > 15) {
       throw "Arm is too close to a singularity for accurate force control;"
             "Please move it to a different configuration and try again";
     }
#endif

     // Set the start and end positions from the current position
     start_position=gfeplug->target_arm_position;
     end_position = start_position;

     // Initialize all position, velocity and acceleration values
     prev_pos = start_position;
     prev_vel = zeros;
     prev_acc = zeros;

     // Print values of the joint positions at the start for checking
#ifdef MD_VERBOSE
     ROS_INFO("The positions of the joints are %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f",
              prev_pos[0], prev_pos[1], prev_pos[2], prev_pos[3], prev_pos[4], prev_pos[5], prev_pos[6]);
#endif

     // Save all initial wam transforms to wam0 in this initial pose as wamTransforms
     JointsToTF::GetTF( prev_pos, wamTransforms);

     // Initialize desired velocity vector to all zeros
     for (int i=0; i<3; i++){
       vel_des.v[i] = 0.0;
       vel_des.w[i] = 0.0;
     }

     // Initialize the distance we have traveled to zer
     distance_traveled = 0.0;

     // Make sure distance is a positive value: if it's negative, make it positive
     if (distance < 0){
       ROS_INFO("The distance input to MoveDirection should be a positive value");
       distance = -distance;
     }

     // Coerce distance input to within reasonable limits, only AFTER checking sign of distance
     if (distance > MAX_DISTANCE){
       ROS_INFO("Distance input has exceeded MAX_DISTANCE and will be reduced to MAX_DISTANCE");
       distance = MAX_DISTANCE;
     }

     // Make sure velocity is a positive value: if it's negative make it positive
     if (velocity < 0){
       ROS_INFO("The velocity input to MoveDirection should be a positive value");
       velocity = -velocity;
     }

     // Coerce velocity input to within reasonable limits, only after checking sign of velocity
     if (velocity > VELOCITY_INPUT_LIMIT){
       velocity = VELOCITY_INPUT_LIMIT;
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
     ss_StopMoveDirection = n.advertiseService("StopMoveDirection", &MoveDirection::StopMoveDirection, this);
  
}

//END MoveDirection Constructor
//********************************************************************

/*********************************************************************
Class:         MoveDirection

Function:      MoveDirection Destructor

Description:   Currently does nothing but print completion message and 
               shut down the stopping service if called
*/
//BEGIN MoveDirection Destructor

MoveDirection::~MoveDirection() {

  // Shut down the Stopping service if it was used to stop the trajectory
  ss_StopMoveDirection.shutdown();

  // Print Completion message
  ROS_INFO("MoveDirection::~MoveDirection: MoveDirection Trajectory finished");

}

//END MoveDirection Destructor
//*********************************************************************

/**********************************************************************
Class:         MoveDirection

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

bool MoveDirection::AddTrajectory(gfe_owd_plugin::StartMoveDirection::Request &req,
				  gfe_owd_plugin::StartMoveDirection::Response &res) {

  ROS_INFO("GfePlugin: received MoveDirection Trajectory service call");

  // Compute a new trajectory
  try {

    // Allocation for the pointer that will hold the single instantiation
    // of our Trajectory class. 
    MoveDirection *newtraj = new MoveDirection(req.direction_x,req.direction_y,req.direction_z,
                                               req.distance,req.velocity,req.compliance);

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
Class:         MoveDirection

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

void MoveDirection::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
 
  time += dt;

  // If user initiated service call to StopMoveDirection, stop trajectory and shutdown
  if (stopmovedirection) {
    ROS_INFO("The trajectory has been cancelled");

    ROS_INFO("The end-effector has moved a distance of %f meters", distance_traveled);

    // Keep tracking the current position of the arm to avoid OWD crash
    end_position = tc.q; 

    // Shut down the plugin
    runstate=OWD::Trajectory::DONE;

    return;
  }

  // Check to see if we have collided with anything (must get eight readings in a row to react)
  static int forcecount(0);

  // Take the dot product of our direction vector with the force torque vector
  if (direction.dot(btVector3(gfeplug->ft_force[0], gfeplug->ft_force[1], gfeplug->ft_force[2]))
      < -FORCE_THRESHOLD){

#ifdef MD_VERBOSE
    ROS_INFO("Force has exceeded FORCE_THRESHOLD in the requested direction: %d", forcecount);
#endif
   
    // Only stop if we have eight cycles in a row that exceed the threshold
    if(++forcecount == 8){
      ROS_INFO("Trajectory is being shut down due to 8 cycles exceeding the FORCE_THRESHOLD");

      //stopmovedirection = true;

      return;
    } 
  } else {

    // Reset the force counter since we haven't had 8 cycles in a row of excessive force.
    // A stray reading may occur due to force/torque sensor noise
    forcecount = 0;

    if(compliance){
      // If compliance is requested, regularly update the orientation of the palm's coordinates
      // This will allow the hand to constantly adjust it's trajectory direction
      JointsToTF::GetTF( prev_pos, wamTransforms);

      // Update the end-effector's translational coordinates
      endpoint_origin = (R3)gfeplug->endpoint;

      // Calculate the distance traveled in the desired direction
      double x_dist_2 = pow((endpoint_origin[0]-initial_endpoint_origin[0]),2);
      double y_dist_2 = pow((endpoint_origin[1]-initial_endpoint_origin[1]),2);
      double z_dist_2 = pow((endpoint_origin[2]-initial_endpoint_origin[2]),2);

      // If in compliance mode, distance is a running change from our prev to our current pos
      distance_traveled += sqrt (x_dist_2 + y_dist_2 + z_dist_2);

      // If in compliance mode, we want to update the starting point origin
      initial_endpoint_origin = endpoint_origin;
    }
    else{

    // Update the end-effector's translational coordinates
    endpoint_origin = (R3)gfeplug->endpoint;

    // Calculate the distance traveled in the desired direction
    double x_dist_2 = pow((endpoint_origin[0]-initial_endpoint_origin[0]),2);
    double y_dist_2 = pow((endpoint_origin[1]-initial_endpoint_origin[1]),2);
    double z_dist_2 = pow((endpoint_origin[2]-initial_endpoint_origin[2]),2);

    // Without compliance, distance is just an overall change from our start to our end
    distance_traveled = sqrt (x_dist_2 + y_dist_2 + z_dist_2);
    
    }

    // Test to see if we have moved the requested distance, stop if we've reached destination
    if ( distance_traveled >= distance ){
      ROS_INFO("Trajectory has moved the desired distance in the requested direction");

      // Set the boolean flag to end the trajectory
      stopmovedirection = true;

      return;
    }
    else {
      // Compute the straight forward Trajectory
      MoveDirection::MoveDirectionImplementation(tc, dt);
    }
  }
   
  // Keep tracking the current position of the arm
  end_position = tc.q; 

  return;
}

//END evaluate
//*********************************************************************

/**********************************************************************
Class:            MoveDirection

Function:         MoveDirectionImplementation

Description:      Creates a straight motion moving forward in the requested
                  direction, using the frame of WAM7 in its initial
                  pose when the plugin is called, which should be directly
                  in front of the desired endpoint. 
*/
// BEGIN MoveDirectionImplementation

void MoveDirection::MoveDirectionImplementation(OWD::Trajectory::TrajControl &tc, double dt) {

  // Create three zeroed out JointPos vectors of size 7 for the position, vel, and accel
  OWD::JointPos pos(std::vector<double>(NDOF,0.0));
  OWD::JointPos vel(std::vector<double>(NDOF,0.0));
  OWD::JointPos acc(std::vector<double>(NDOF,0.0));

  // Velocity Vectors for motion in wam7 and wam0
  btVector3 wam7vel, wam0vel;

  // Initialize both vectors to zero
  for (int i=0; i<3; i++){
    wam7vel[i]=0.0;
    wam0vel[i]=0.0;
  }

  // Enter the requested velocity in the requested direction
  // Ramp up slowly so that by 3 seconds, we are at the requested velocity
  double vel_start = (velocity/3) * time;
  if (vel_start < velocity){
    for (int i=0; i<3; i++){
        wam7vel[i]=vel_start*direction[i];
      }
  }
  else {
    for (int i=0; i<3; i++){
      wam7vel[i]=velocity*direction[i];
    }
  }

#ifdef MD_VERBOSE
  // Print the input velocity commands one time per second
  static int counter = 0;
  counter++;
  if (counter == 500){
    ROS_INFO("The x-value of the requested velocity is %f", wam7vel[0]);
    ROS_INFO("The y-value of the requested velocity is %f", wam7vel[1]);
    ROS_INFO("The z-value of the requested velocity is %f", wam7vel[2]);
    counter = 0;
  }
#endif

  // Get rotation from initial WAM7 to wam0/base using wamTransforms(index 6 for wam7)
  btTransform transform(wamTransforms[6].getRotation(), btVector3(0,0,0));
  wam0vel = transform * wam7vel;

  // Convert desired velocity into an R6 
  // vel_des = desired end-effector velocity and angular velocity in base frame
  // vel_des = [vx, vy, vz, wx, wy, wz]
  for (int i=0; i<3; i++){
    // Set velocities from above for straight linear motion
    vel_des.v[i] = wam0vel[i];

    // Set angular velocities to zero
    vel_des.w[i] = 0.0;
  }

  // Calculate dq_pose and vel
  // The Inverse Jacobian translates end-effector velocities to joint velocities in the base frame
  // dq_pose = vector of joint velocities = JacobianInverse * vector of end-effector velocities
  OWD::JointPos dq_pose = OWD::Plugin::JacobianPseudoInverse_times_vector(vel_des);

  // Start position is ideal position. If we can get close to it, do it. 
  OWD::JointPos q_offset = start_position - tc.q;

  // ToDo: q_offset: hardcode preferred joint config instead of using start_position config
  // ToDo: q_offset: maybe use weighted nullspace to make base joints move less for rotation changes

  // Keep the arm away from its joint limits and near a desired configuration.
  // Otherwise, the arm will eventually drift toward the joint limits and stop moving.
  // Note: Due to redundancy in the 7 DOF WAM arm, vectors multiplied by the Jacobian which 
  // live in the nullspace will not produce any motion. 
  OWD::JointPos dq_offset = OWD::Plugin::Nullspace_projection(q_offset);

  // Weight how much we go toward the ideal position.
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

#ifdef MD_VERBOSE
  // Print the joint velocities one time per second using the counter from above
  if (counter == 0){
    ROS_INFO("The joint velocities are %f %f, %f, %f, %f, %f, %f",
             vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], vel[6]);
  }
#endif

  // Check for joint limits. Project arm 0.5sec into the future
  OWD::JointPos q_future = prev_pos + vel*0.5;
  // Stop arm if q_future exceeds joint limits
  if ( CheckExceedJointLimits(q_future) ){
    ROS_ERROR("MoveDirection::MoveDirectionImplementation: q_future exceeds joint limits. Setting velocity to 0 and shutting down Trajectory");
    vel = zeros;
    stopmovedirection=true;
    return;
  }

  // Set variables
  pos = prev_pos + vel*dt;
  acc = zeros;

#ifdef MD_VERBOSE
  // Print the joint positions one time per second using the counter from above
  if (counter == 0){
    ROS_INFO("The positions of the joints are %f, %f, %f, %f, %f, %f, %f",
             pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]);
  }
#endif

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
//END MoveDirectionImplementation 
//*********************************************************************


/**********************************************************************
Class:           MoveDirection

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

bool MoveDirection::CheckExceedJointLimits(OWD::JointPos position) {

  //Compare input position vector and corresponding value in joint limit array
  for (int i=0; i<NDOF; i++){

    if( ( position[i] < JointLimits[i][0] ) || ( position[i] > JointLimits[i][1] ) ){

      ROS_INFO("The joint which is near going out of range is joint %d", i+1);
      
      //Return true if value is outside of joint limit
      return true;
    }
  }

  //Return false if all projected position values are within joint limits
  return false;
}
//END CheckExceedJointLimits
//*********************************************************************


/**********************************************************************
Class:         MoveDirection

Function:      Register

Description:   This helper function is called by our register_owd_plugin
               function when the plugin is first loaded.  It just sets
               up our ROS service for receiving trajectory requests.
*/
//BEGIN Register 

bool MoveDirection::Register() {
  ros::NodeHandle n("~");
  ss_Add = n.advertiseService("StartMoveDirection",&MoveDirection::AddTrajectory);
  return true;
}

//END Register
//*********************************************************************
 

/**********************************************************************
Class:         MoveDirection

Function:      Shutdown

Description:   Our unregister_owd_plugin function will call this when
               OWD shuts down. By shutting down our ROS service we will
               remove its listing from the ROS master.
*/
//BEGIN Shutdown

void MoveDirection::Shutdown() {
  ss_Add.shutdown();
}

//END Shutdown
//*********************************************************************


/**********************************************************************
Class:           MoveDirection

Function:        StopMoveDirection

Description:     Stop the Trajectory when asked by the client

*/
// BEGIN StopMoveDirection

bool MoveDirection::StopMoveDirection(gfe_owd_plugin::StopMoveDirection::Request &req,
				      gfe_owd_plugin::StopMoveDirection::Response &res) {
  
  stopmovedirection=true;
  return true;
}

// END StopMoveDirection
//*********************************************************************



