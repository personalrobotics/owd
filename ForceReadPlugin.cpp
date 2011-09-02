/***********************************************************************

Author:        Kevin Knott (SRI), 2011

Description:   This OWD Plugin publishes the feed forward predicted 
               force reading of the end effector based upon joint torques. 

Note:          You are free to modify and reuse this code without
               restrictions.

***********************************************************************/

#include "ForceReadPlugin.h"

/*********************************************************************
Class:         ForceReadPlugin

Function:      ForceReadPlugin Constructor

Description:   The Plugin constructor  


*/
//BEGIN ForceReadPlugin Constructor

ForceReadPlugin::ForceReadPlugin() {
  
  // ROS has already been initialized by OWD, so we can just
  // create our own NodeHandle in the same namespace
  ros::NodeHandle n("~");
  force_read_pub = n.advertise<pr_msgs::ForceRead>("force_read",1);

}

//END ForceReadPlugin Constructor
//********************************************************************


/*********************************************************************
Class:         ForceReadTraj

Function:      ForceReadTraj Destructor

Description:   The publisher does not need to be shut down. 

*/
//BEGIN ForceReadPlugin Destructor

ForceReadPlugin::~ForceReadPlugin() {
}

//END ForceReadTraj Destructor
//*********************************************************************

 
/**********************************************************************
Class:         ForceReadTraj

Function:      Publish

Description:   This function publishes the feed forward predicated force
               reading of the end effector based upon joint torques. 
        
*/
//BEGIN Publish

void ForceReadPlugin::Publish() {
 
  //Create a vector of doubles for storing the total torque, initially all 0's
  OWD::JointPos total_torque(std::vector<double>(7,0.0));

  //Total torque = Controller (PID) + Modeled (Dynamic) + Current (Trajectory)
  for (int i=0; i<7; i++){
    total_torque[i] = pid_torque[i] + 
                      dynamic_torque[i] +
                      trajectory_torque[i];
  }

  //Declare an R6 variable for holding the feed forward predicted force
  R6 force_reading;
  try{
    force_reading = Jacobian_times_vector(total_torque);
  } catch(const char *err){
    ROS_ERROR_NAMED("ForceRead", "Could not multiply Jacobian times total torque");
    return;
  }

  //Declare a message of type pr_msgs and ForceRead
  pr_msgs::ForceRead force_read_msg;

  //Declare the size of the message vector
  force_read_msg.force.resize(7);
  
  //Fill the message vector with the calculated values of the force  
  for (int i=0; i<7; i++){
    force_read_msg.force[i] = force_reading[i];
  }
   
  //Publish the feedforward force reading from ForceRead
  force_read_pub.publish(force_read_msg);

  return;
}

//END evaluate
//***********************************************************************

// Allocation for the pointer that will hold the single instantiation
// of our plugin class.  We initialize it to NULL so that the register
// function can tell whether or not one has already been allocated.
ForceReadPlugin *forcereadplug = NULL;

// The register_owd_plugin() is the only function that OWD will call when
// the plugin is loaded.  It has to initialize our custom Plugin class

bool register_owd_plugin() {
  if (forcereadplug) {
    delete forcereadplug; // free the previous one in case register was called twice
  }
  try {
    ROS_WARN("Starting Forceread Plugin\n");
    // create an instantiation of our custom Plugin class
    forcereadplug = new ForceReadPlugin();
  } catch (...) {
    forcereadplug=NULL;
    return false;
  }
  return true;
}

void unregister_owd_plugin() {
  if (forcereadplug) {
    // remove our Plugin class, which will let it shut down any ROS
    // communications it created
    delete forcereadplug;
    forcereadplug=NULL;
  }
  return;
}
