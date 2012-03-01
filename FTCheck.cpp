/***********************************************************************

  Copyright 2011 Carnegie Mellon University and Intel Corporation
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "FTCheck.h"
#define PEAK_CAN
#include "openwam/CANdefs.hh"	// for HANDSTATE_* enumeration
#include "openwamdriver.h"

bool FTCheck::FTCheckSrv(pr_msgs::Reset::Request &req,
			 pr_msgs::Reset::Response &res) {
  const int testtime(5);
  // compute a new trajectory
  try {
    FTCheck *newtraj = new FTCheck(testtime);

    // before starting the trajectory we are going to tare the F/T sensor
    // and wait for the SW tare average to be computed
    OWD::WamDriver::bus->ft_tare();
    int sleepcount=0;
    while (!OWD::WamDriver::bus->valid_forcetorque_data
	   && (++sleepcount < 100)) {
      usleep(4000);
    }
    if (sleepcount == 100) {
      res.reason="FAILED: FT sensor never finished taring";
      res.ok=false;
      return true;
    }
      
    // send the traj to the arm
    int id = OWD::Plugin::AddTrajectory(newtraj,res.reason);
    if (id <= 0) {
      delete newtraj;
      res.ok=false;
      return true;
    }
  } catch (const char *err) {
    res.ok=false;
    res.reason=err;
    return true;
  }

  int waits(0);
  while (!done && (++waits < testtime *50)) {
    usleep(100000); // 0.1s
  }
  if (waits == testtime*50) {
    res.ok=false;
    res.reason="Never detected end of test";
  } else {
    res.ok=true;
    res.reason=summary;
  }
  return true;
}

FTCheck::FTCheck(int _testtime) :
  OWD::Trajectory("FTCheck"),
  testtime(_testtime),
  samples(0)
{
  if (hybridplug) {

    if ((hybridplug->ft_force.size() < 3) ||
	(hybridplug->ft_torque.size() < 3)) {
      throw "FTCheck requires that the Force/Torque sensor is installed and configured";
    }
    
    // Set the start position from the current position
    start_position=hybridplug->target_arm_position;
    end_position = start_position;

  } else {
    throw "Could not get current WAM values from Hybridplugin";
  }

  raw_max.resize(6);
  raw_min.resize(6);
  filtered_max.resize(6);
  filtered_min.resize(6);
  raw_sum.resize(6);
  filtered_sum.resize(6);
  strcpy(summary,"");
  done=false;
  hybridplug->recorder->reset();
}

void FTCheck::evaluate(OWD::Trajectory::TrajControl &tc, double dt) {
  time += dt;
  
  if (samples == testtime * 500) {

    snprintf(summary,2000,"\n\
Signal         MIN      MAX      AVG    MAX-MIN\n\
RAW X         % 1.3f   % 1.3f   % 1.3f   % 1.3f\n\
RAW Y         % 1.3f   % 1.3f   % 1.3f   % 1.3f\n\
RAW Z         % 1.3f   % 1.3f   % 1.3f   % 1.3f\n\
FILTERED X    % 1.3f   % 1.3f   % 1.3f   % 1.3f\n\
FILTERED Y    % 1.3f   % 1.3f   % 1.3f   % 1.3f\n\
FILTERED Z    % 1.3f   % 1.3f   % 1.3f   % 1.3f\n\
RAW R         % 1.3f   % 1.3f   % 1.3f   % 1.3f\n\
RAW P         % 1.3f   % 1.3f   % 1.3f   % 1.3f\n\
RAW Y         % 1.3f   % 1.3f   % 1.3f   % 1.3f\n\
FILTERED R    % 1.3f   % 1.3f   % 1.3f   % 1.3f\n\
FILTERED P    % 1.3f   % 1.3f   % 1.3f   % 1.3f\n\
FILTERED Y    % 1.3f   % 1.3f   % 1.3f   % 1.3f\n",
	     raw_min[0],raw_max[0],raw_sum[0]/samples,raw_max[0]-raw_min[0],
	     raw_min[1],raw_max[1],raw_sum[1]/samples,raw_max[1]-raw_min[1],
	     raw_min[2],raw_max[2],raw_sum[2]/samples,raw_max[2]-raw_min[2],
	     filtered_min[0],filtered_max[0],filtered_sum[0]/samples,filtered_max[0]-filtered_min[0],
	     filtered_min[1],filtered_max[1],filtered_sum[1]/samples,filtered_max[1]-filtered_min[1],
	     filtered_min[2],filtered_max[2],filtered_sum[2]/samples,filtered_max[2]-filtered_min[2],
	     raw_min[3],raw_max[3],raw_sum[3]/samples,raw_max[3]-raw_min[3],
	     raw_min[4],raw_max[4],raw_sum[4]/samples,raw_max[4]-raw_min[4],
	     raw_min[5],raw_max[5],raw_sum[5]/samples,raw_max[5]-raw_min[5],
	     filtered_min[3],filtered_max[3],filtered_sum[3]/samples,filtered_max[3]-filtered_min[3],
	     filtered_min[4],filtered_max[4],filtered_sum[4]/samples,filtered_max[4]-filtered_min[4],
	     filtered_min[5],filtered_max[5],filtered_sum[5]/samples,filtered_max[5]-filtered_min[5]);

    runstate=OWD::Trajectory::DONE;
    done=true;
    return;
  }

  ++samples;
  hybridplug->net_force.data.resize(48);
  hybridplug->net_force.data[0]=hybridplug->ft_force[0];
  hybridplug->net_force.data[1]=hybridplug->ft_force[1];
  hybridplug->net_force.data[2]=hybridplug->ft_force[2];
  hybridplug->net_force.data[3]=hybridplug->ft_torque[0];
  hybridplug->net_force.data[4]=hybridplug->ft_torque[1];
  hybridplug->net_force.data[5]=hybridplug->ft_torque[2];
  hybridplug->net_force.data[6]=hybridplug->filtered_ft_force[0];
  hybridplug->net_force.data[7]=hybridplug->filtered_ft_force[1];
  hybridplug->net_force.data[8]=hybridplug->filtered_ft_force[2];
  hybridplug->net_force.data[9]=hybridplug->filtered_ft_torque[0];
  hybridplug->net_force.data[10]=hybridplug->filtered_ft_torque[1];
  hybridplug->net_force.data[11]=hybridplug->filtered_ft_torque[2];
  for (int i=0; i<6; ++i) {
    double d=hybridplug->net_force.data[i];
    double fd=hybridplug->net_force.data[i+6];
    if (d>raw_max[i]) {
      raw_max[i]=d;
    }
    if (d<raw_min[i]) {
      raw_min[i]=d;
    }
    raw_sum[i] += d;
    if (fd > filtered_max[i]) {
      filtered_max[i]=fd;
    }
    if (fd < filtered_min[i]) {
      filtered_min[i]=fd;
    }
    filtered_sum[i] += fd;

    hybridplug->net_force.data[12+i]=raw_max[i];
    hybridplug->net_force.data[18+i]=raw_min[i];
    hybridplug->net_force.data[24+i]=raw_sum[i] / samples;
    hybridplug->net_force.data[30+i]=filtered_max[i];
    hybridplug->net_force.data[36+i]=filtered_min[i];
    hybridplug->net_force.data[42+i]=filtered_sum[i] / samples;
  }
  hybridplug->log_data(hybridplug->net_force.data);
  return;
}

ros::ServiceServer FTCheck::ss_FTCheck;
char FTCheck::summary[2000];
bool FTCheck::done(false);
  
// Set up our ROS service for receiving trajectory requests.
bool FTCheck::Register() {

  ros::NodeHandle n("~");
  ss_FTCheck = n.advertiseService("FTCheck",&FTCheck::FTCheckSrv);

  return true;
}

// Shut down our service so that it's no longer listed by the ROS master
void FTCheck::Shutdown() {
  ss_FTCheck.shutdown();
}

FTCheck::~FTCheck() {
  hybridplug->flush_recorder_data = true;
}

