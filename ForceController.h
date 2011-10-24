/***********************************************************************

  Copyright 2011 Carnegie Mellon University
  Author: Mike Vande Weghe <vandeweg@cmu.edu>

**********************************************************************/
#ifndef FORCE_CONTROLLER_H
#define FORCE_CONTROLLER_H

#include <openwam/Plugin.hh>

class ForceController {
 public:
  ForceController();
  ~ForceController();

  OWD::JointPos control(R6 ft_error);
  void reset();

  double fkp, fkd, fki;  // force gains
  double tkp, tkd, tki;  // torque gains
  double f_multiplier; // gain scaling
  R6 last_ft_error;
  R6 bounded_ft_error, ft_error_delta, ft_error_integral;
  R6 ft_correction;
  double integral_saturation_limit;
};

#endif // FORCE_CONTROLLER_H
