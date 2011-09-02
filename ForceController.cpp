/***********************************************************************

  Copyright 2011 Carnegie Mellon University
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "ForceController.h"

ForceController::ForceController()
  : fkp(8), fkd(0), fki(0.001),
    tkp(1), tkd(0), tki(0),
    last_ft_error(0,0,0,0,0,0),
    ft_error_integral(0,0,0,0,0,0) {
}

ForceController::~ForceController() {
}

OWD::JointPos ForceController::control(R6 ft_error) {
                                       
  R6 ft_error_delta = ft_error - last_ft_error;
  last_ft_error = ft_error;
  ft_error_integral += ft_error;

  R6 ft_correction(fkp * ft_error.v +
		   fkd * ft_error_delta.v +
		   fki * ft_error_integral.v,
		   tkp * ft_error.w +
		   tkd * ft_error_delta.w +
		   tki * ft_error_integral.w);

  return OWD::Plugin::JacobianTranspose_times_vector(ft_correction);
}

void ForceController::reset() {
  last_ft_error = R6(0,0,0,0,0,0);
  ft_error_integral = R6(0,0,0,0,0,0);
}
