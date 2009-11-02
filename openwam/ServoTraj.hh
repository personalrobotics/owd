#include <iostream>
#include <math.h>
#include "Trajectory.hh"
#include <vector>

#ifndef __SERVOTRAJ_HH__
#define __SERVOTRAJ_HH__

class ServoTraj : public Trajectory {
private:
  int nDOF;
  std::vector<double> stoptime;
  std::vector<double> target_velocity;
  std::vector<double> current_velocity;
  const float vlimit;
  const float accel;
  double jlimit_buffer;
  JointPos current_position;

public:

  ServoTraj(int DOF, int id, double *start_pos);
  virtual ~ServoTraj();

  bool SetVelocity(int j, float v, float duration = 0.5);
  void stop(); // override of base class stop()
  
  // mandatory functions inherited from Trajectory
  void evaluate(double y[], double yd[], double ydd[], double dt);
};

#endif
