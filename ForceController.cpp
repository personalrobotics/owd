/***********************************************************************

  Copyright 2011 Carnegie Mellon University
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "ForceController.h"

ForceController::ForceController()
  : fkp(1), fkd(0), fki(0),
    tkp(1), tkd(0), tki(0),
    last_ft_error(0,0,0,0,0,0),
    ft_error_integral(0,0,0,0,0,0) {

  if (OWD::Plugin::ft_torque.size() != 6) {
    throw "No Force/Torque sensor data available";
  }
}

ForceController::~ForceController() {
}

OWD::JointPos ForceController::control(R6 force_torque) {
  R6 ft_current(OWD::Plugin::ft_torque[0],
		OWD::Plugin::ft_torque[1],
		OWD::Plugin::ft_torque[2],
		OWD::Plugin::ft_torque[3],
		OWD::Plugin::ft_torque[4],
		OWD::Plugin::ft_torque[5]);
  R6 ft_error = force_torque - ft_current;
  R6 ft_error_delta = ft_error - last_ft_error;
  last_ft_error = ft_error;
  ft_error_integral += ft_error;

  R6 ft_correction(fkp * ft_error.v() +
		   fkd * ft_error_delta.v() +
		   fki * ft_error_integral.v(),
		   tkp * ft_error.w() +
		   tkd * ft_error_delta.w() +
		   tki * ft_error_integral.w());

  return OWD::Plugin::JacobianTranspose_times_vector(ft_correction);
}

void ForceController::reset() {
  last_ft_error = R6(0,0,0,0,0,0);
  ft_error_integral = R6(0,0,0,0,0,0);
}
