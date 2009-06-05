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

#ifndef __JOINTCTRLPID_HH__
#define __JOINTCTRLPID_HH__

class JointCtrlPID : public JointCtrl{

private:
  double Kp, Kd, Ki;
  double laste;
  double se;         // Sum of error
    //  static const double Isaturation = 500;
  static const double Isaturation = 50;

    double _GetSign(double in)
    {
        if(in >= 0)
            return 1.0;
        else
            return -1.0;

    }

public:

  JointCtrlPID(){reset();}
  
  double evaluate(double qs, double q, double dt){
    if(state() == Controller::RUN){
      set(qs);   // backup the command
      lock();
      double e = qs - q;
      double ed = (e - laste)/dt;
      se += e;
      laste = e;
      double Isign = _GetSign(se);
      if(Isign*se > Isaturation)
      {
          se = Isign*Isaturation;
      }
      unlock();
      return Kp*e + Kd*ed + Ki*se;
    }
    return 0.0;
  }


  void reset(){       lock();  laste = 0;  unlock();  }

  istream& get(istream& s){    s >> Kp >> Kd >> Ki;    return s;  }
  
  ostream& put(ostream& s){
    s << "Kp: "  << setw(4) << Kp << "; "
      << "Kd: "  << setw(4) << Kd << "; "
      << "Ki: "  << setw(4) << Ki ;
    return s;
  }
};

#endif
