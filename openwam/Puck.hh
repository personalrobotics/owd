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

#include <iostream>
#include "globals.h"

#ifndef __PUCK_H__
#define __PUCK_H__

using namespace std;

#define PUCK_ORDER_MIN 1
#define PUCK_ORDER_MAX 4
#define NUM_ORDERS     (PUCK_ORDER_MAX - PUCK_ORDER_MIN + 1)

enum{STATUS_OFFLINE = -1,
     STATUS_RESET,
     STATUS_ERR,
     STATUS_READY
};

class Puck{
 private:
   int IoffsetB;         // Offset to calibrate zero for current sensors
   int IoffsetC;         // Offset to calibrate zero for current sensors

   int ID;
   int group_id;
   int group_order; 
   int motor_id;   
   int cpr;             // Encoder counts per revolution

 public:

  static const int MIN_TRQ[];
  static const int MAX_TRQ[];
  
  Puck(){ID=-1;}
  
  int id(){return ID;}
  int group(){return group_id;}
  int order(){return group_order;}
  int motor(){return motor_id;}
  int CPR(){return cpr;}
  
  friend istream& operator >> (istream& s, Puck& p){
    s >> p.ID
      >> p.motor_id
      >> p.group_id
      >> p.group_order
      >> p.IoffsetB
      >> p.IoffsetC
      >> p.cpr;
    return s;
  }

  friend ostream& operator << (ostream& s, Puck& p){
    s << "Puck (id#): "     << p.id()     << "; "
      << "Motor#: "         << p.motor()  << "; "
      << "Group (id#): "    << p.group()  << "; "
      << "Group (order#): " << p.order()  << "; "
      << "IoffsetB: "       << p.IoffsetB << "; "
      << "IoffsetC: "       << p.IoffsetC << "; "
      << "Counts/Rev: "     << p.cpr;
    return s;
  }
}; 

#endif
