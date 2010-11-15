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

#include <iomanip>
#include <mathimf.h>

#include "Controller.hh"

#ifndef __PID_HH__
#define __PID_HH__

using namespace std;

# define sign(x) (x>=0?1:-1)

class PID : public Controller{
private:
  
  double Kp, Kd, Ki;
  double saturation; // Defaults to zero (doesn't ever saturate)
   
  double laste;
  double se;         // Sum of error
   
  int firsttick;     // handles special case de for first tick
   
public:

  PID();
  ~PID(){}

  // virtual interface
  void set(double qref);
  void reset(double qref);
  double evaluate(double q, double dt);
  
  void KpKdKi(double kp, double kd, double ki){Kp=kp;Kd=kd;Ki=ki;}

  istream& get(istream& s);
  ostream& put(ostream& s);
};

#endif 
