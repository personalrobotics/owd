#include "JointCtrlSea.hh"
#include <unistd.h>

JointCtrlSea::JointCtrlSea(){

  torqLimit = 0.0; // negative val means no bound
  pendingTorqLimit = 0.0;
  
  pendingKp = 0.0;
  preferedKp = 0.0;
  safeKp = 0.0;
  Kp = 0.0;

  iLimiter = 0.1; // the i term can at most be this fraction of threshold
  pendingKi = 0.0;
  preferedKi = 0.0;
  safeKi = 0.0;
  Ki = 0.0;

  pendingKd = 0.0; 
  pendingKd = 0.0;
  preferedKd = 0.0;
  Kd = 0.0;

  realtimeReset();
}

void JointCtrlSea::specifySafeSettings(double _safeKp, double _safeKi, double _safeKd) {
  safeKp=_safeKp;
  safeKi=_safeKi;
  safeKd=_safeKd;
}

void JointCtrlSea::reset() {
  lock();
  bPendingReset = true;
  unlock();
}

void JointCtrlSea::setTorqLimit(double newLimit) {
  lock();
  pendingTorqLimit = newLimit;
  if (newLimit < 0.0) {
    pendingKp = safeKp;
    pendingKi = safeKi;
    pendingKd = safeKd;
  } else {
    pendingKp = preferedKp;
    pendingKi = preferedKi;
    pendingKd = preferedKd;
  }
  unlock();
}

double JointCtrlSea::getTorqLimit(){
  return pendingTorqLimit;
}

unsigned int JointCtrlSea::torqIsAtLimit(){
  return torqLimitCount;
}

void JointCtrlSea::setKp(double newKp){
  lock();
  if (pendingTorqLimit >= 0.0) {
    preferedKp = newKp;
    pendingKp = preferedKp;
  } else {
    pendingKp = safeKp;
  }
  unlock();
}

double JointCtrlSea::getKp(){
  return pendingKp;
}

void JointCtrlSea::setKi(double newKi){
  lock();
  if (pendingTorqLimit >= 0.0) {
    preferedKi = newKi;
    pendingKi = preferedKi;
  } else {
    pendingKi = safeKi;
  }
  unlock();
}

double JointCtrlSea::getKi(){
  return pendingKi;
}

void JointCtrlSea::setKd(double newKd){
  lock();
  if (pendingTorqLimit >= 0.0) {
    preferedKd = newKd;
    pendingKd = preferedKd;
  } else {
    pendingKd = safeKd;
  }
  unlock();
}

double JointCtrlSea::getKd() {
  return pendingKd;
}


double JointCtrlSea::evaluate(double qs, double q, double dt){

  // early out
  if (this->s != Controller::RUN) {
    return 0.0;
  }

  if ( trylock() == 0 ) { 
    if (bPendingReset == true) {
      realtimeReset();
    }
    torqLimit = pendingTorqLimit;
    Kp = pendingKp;    
    Ki = pendingKi;    
    Kd = pendingKd;
    unlock();
  }

  double sign;
  bool bLimit = false;

  double e = qs - q;
  double pTerm = Kp*e;
  if (torqLimit >= 0 && Kp != 0) {
    sign = _GetSign(pTerm);
    if (sign * pTerm > torqLimit) {
      pTerm = sign * torqLimit;
      bLimit = true;
    }
  }

  double ed = (e - laste)/dt;
  double dTerm = Kd*ed;
  if (torqLimit >= 0 && Kd != 0) {
    sign = _GetSign(dTerm);
    if (sign * dTerm > torqLimit) {
      dTerm = sign * torqLimit;
      bLimit = true;
    }
  }

  se += e;
  double iTerm = Ki*se;
  if (torqLimit >= 0 && Ki != 0) {
    sign = _GetSign(iTerm);
    if (sign * iTerm > torqLimit*iLimiter) {
      iTerm = sign * torqLimit*iLimiter;
      se = iTerm/Ki;
      bLimit = true;
    }
  }

  laste = e;

  double result = pTerm + iTerm + dTerm;
  sign = _GetSign(result);
  if (torqLimit >= 0 && sign * result > torqLimit) {
    result = sign * torqLimit;
    bLimit = true;
  }

  if (bLimit == true) {
    torqLimitCount++;
  } else {
    torqLimitCount = 0;
  }

  return result;
}

void JointCtrlSea::realtimeReset(){
  laste = 0.0; 
  se = 0.0;
  torqLimitCount=0;
  bPendingReset = false;
}


istream& JointCtrlSea::get(istream& s){
  s >> Kp >> Kd >> Ki;
  return s;
}
  
ostream& JointCtrlSea::put(ostream& s){
  s << "Kp: "  << setw(4) << Kp << "; "
    << "Kd: "  << setw(4) << Kd << "; "
    << "Ki: "  << setw(4) << Ki ;
  return s;
}


double JointCtrlSea::_GetSign(double in) {
  if(in >= 0) {
    return 1.0;
  } else {
    return -1.0;
  }
}
