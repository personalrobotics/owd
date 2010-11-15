/*
  Copyright 2006 Simon Leonard

  This file is part of openwam.

  openwam is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 3 of the License, or (at your
  option) any later version.

  openwam is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Modified 2007-2010 by:
      Mike Vande Weghe <vandeweg@cmu.edu>
      Robotics Institute
      Carnegie Mellon University
*/

#include "PID.hh"

istream& PID::get(istream& s){
  s >> Kp >> Kd >> Ki >> saturation;
  return s;
}

ostream& PID::put(ostream& s){
  s << "Kp: "  << setw(4) << Kp << "; "
    << "Kd: "  << setw(4) << Kd << "; "
    << "Ki: "  << setw(4) << Ki << "; "
    << "Sat: " << saturation;
  return s;
}

PID::PID(){
  Kp = Kd = Ki = qref = laste = se = saturation = 0.0;
  firsttick = 1;
}

void PID::reset(double qref){
  lock();
  //se = 0.0;
  laste = 0.0;
  //firsttick = 1;
  this->qref = qref;
  unlock();
}

void PID::set(double qref){
  lock();
  this->qref = qref;
  unlock();
}

double PID::evaluate(double q, double dt){
  double e, de, result;
  
  lock();
  
  e = qref - q;
  if(firsttick){
    laste = e;
    firsttick = 0;
  }
  
  de = (e - laste)/dt;
  laste = e;
  se += e;
  
  if(saturation)
    if(saturation < fabs(se)*Ki)
      se = sign(se)*saturation;
  
  result = Kp*e + Kd*de + Ki*se;
  
  unlock();

  return result;
}

