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

  OWD::JointPos control(R6 force_torque);
  void reset();

  double fkp, fkd, fki;  // force gains
  double tkp, tkd, tki;  // torque gains
  R6 last_ft_error;
  R6 ft_error_integral;
};

#endif // FORCE_CONTROLLER_H
