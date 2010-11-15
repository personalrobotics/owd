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
