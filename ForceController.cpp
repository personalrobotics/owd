/***********************************************************************

  Copyright 2011 Carnegie Mellon University
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/

#include "ForceController.h"

ForceController::ForceController()
  : fkp(2), fkd(0), fki(0.08),
    tkp(0), tkd(0), tki(0),
    last_ft_error(0,0,0,0,0,0),
    ft_error_integral(0,0,0,0,0,0),
    integral_saturation_limit(12) {
}

ForceController::~ForceController() {
}

OWD::JointPos ForceController::control(R6 ft_error) {
                                       
  ft_error_delta = ft_error - last_ft_error;
  last_ft_error = ft_error;

  // limit the magnitude of the proportional error (and thus the
  // amount that gets added to the integral term, too)
  static double ft_error_limit = 3.0;
  if (ft_error.norm() > ft_error_limit) {
    bounded_ft_error = ft_error * ft_error_limit / ft_error.norm();
  } else {
    bounded_ft_error = ft_error;
  }
  ft_error_integral += bounded_ft_error;

  // limit windup of the integral term
  if (ft_error_integral.norm() > integral_saturation_limit) {
    ft_error_integral = ft_error_integral * integral_saturation_limit / ft_error_integral.norm();
  }

  ft_correction.v =
    fkp * bounded_ft_error.v +
    fkd * ft_error_delta.v +
    fki * ft_error_integral.v;
  ft_correction.w =
    tkp * bounded_ft_error.w +
    tkd * ft_error_delta.w +
    tki * ft_error_integral.w;
  
  return OWD::Plugin::JacobianTranspose_times_vector(ft_correction);
}

void ForceController::reset() {
  last_ft_error = R6(0,0,0,0,0,0);
  ft_error_integral = R6(0,0,0,0,0,0);
}
