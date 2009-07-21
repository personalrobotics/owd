/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openman is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openman is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "JointCtrl.hh"

#ifndef __JOINTCTRLSEA_HH__
#define __JOINTCTRLSEA_HH__

class JointCtrlSea : public JointCtrl{

public:

  JointCtrlSea();

  // real-time interface
  double evaluate(double qs, double q, double dt);
  void realtimeReset();

  // non-real time interface
  
  // safe settings will be used when there is no torque limit
  void specifySafeSettings(double safeKp, double safeKi, double safeKd); 
 
  void reset();

  void setTorqLimit(double newLimit);
  double getTorqLimit();
  unsigned int torqIsAtLimit();

  void setKp(double newKp);
  double getKp();

  void setKi(double newKi);
  double getKi();

  void setKd(double newKd);
  double getKd();

  istream& get(istream& s);
  ostream& put(ostream& s);

private:

  double iLimiter;
  double torqLimit, Kp, Ki, Kd;
  double pendingTorqLimit, pendingKp, pendingKi, pendingKd;
  double preferedKp, preferedKi, preferedKd;
  double safeKp, safeKi, safeKd;
  bool bPendingReset;
  unsigned int torqLimitCount;
  double laste;
  double se;         // Sum of error

  double _GetSign(double in);

  
};

#endif
