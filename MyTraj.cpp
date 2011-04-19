#include "MyTraj.hh"
#include <ros/ros.h>
#include <std_msgs/String.h>

MyPlugin::MyPlugin() {
  ros::NodeHandle n("~");
  pub_info = n.advertise<std_msgs::String>("MyPluginInfo",10);
}

MyPlugin::~MyPlugin() {
  pub_info.shutdown();
}

void MyPlugin::Publish() {
  std_msgs::String myinfo;
  std::stringstream ss;
  ss << "MyPlugin: joint 1 is " << arm_position()[0];
  myinfo.data = ss.str();
  pub_info.publish(myinfo);
}

MyPlugin *myplug = NULL;

MyTraj::MyTraj(int j, double t): OWD::Trajectory("MyTraj"), joint(j), torque(t) {
  if ((j<1) || (j>7)) {
    throw "Joint out of range";
  }
  if (fabs(t)>10) {
    throw "Torque limited to 10nm";
  }
  if (myplug) {
    start_position=myplug->target_arm_position();
    end_position = start_position;
  }
}

bool MyTraj::AddTrajectory(owd_traj_example::AddMyTrajectory::Request &req,
			   owd_traj_example::AddMyTrajectory::Response &res) {
  ROS_INFO("MyTraj: received AddTrajectory service call");

  // compute a new trajectory
  try {
    MyTraj *newtraj = new MyTraj(req.joint,req.torque);
    // send it to the arm
    res.id = Trajectory::AddTrajectory(newtraj,res.reason);
    if (res.id > 0) {
      res.ok=true;
    } else {
      res.ok=false;
    }
  } catch (const char *err) {
    res.ok=false;
    res.reason=err;
    res.id=0;
    return true;
  }


  // always return true for the service call so that the client knows that
  // the call was actually processed, as opposed to there being a
  // network communication error.
  // the client will examine the "ok" field to see if the command was
  // actually successful
  return true;
}

void MyTraj::evaluate(Trajectory::TrajControl &tc, double dt) {

  time += dt;

  if (time < 1) {
    tc.t[joint] = torque; // apply torque for exactly 1 second
    ROS_DEBUG("MyTraj Time=%f secs",time);
  } else {
    ROS_DEBUG("MyTraj DONE Time=%f secs",time);
    runstate=Trajectory::DONE;
  }
  end_position =tc.q;  // keep tracking the current position

  return;
}

bool MyTraj::Register() {
  ros::NodeHandle n("~");
  ss_Add = n.advertiseService("AddMyTrajectory",&MyTraj::AddTrajectory);
  return true;
}

void MyTraj::Shutdown() {
  ss_Add.shutdown();
}

ros::ServiceServer MyTraj::ss_Add;

bool register_owd_plugin() {
  if (myplug) {
    delete myplug; // free the previous one in case register was called twice
  }
  try {
    myplug = new MyPlugin();
  } catch (...) {
    myplug=NULL;
    return false;
  }
  return true;
}

void unregister_owd_plugin() {
  if (myplug) {
    delete myplug;
    myplug=NULL;
  }
  return;
}
