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

#include "Joint.hh"

//const double Joint::MIN_POS[]={0.00,-2.62,-1.80,-2.60,-0.70,-4.30,-1.25,-2.50};
//const double Joint::MAX_POS[]={0.00, 2.62, 1.80, 2.60, 3.10, 1.00, 1.25, 2.50};

// These values are based on the motor limits that are set in Puck.cc, then
// transformed to joint limits using the IPNM and transmission ratios for
// each motor and joint.
const double Joint::MAX_MECHANICAL_TORQ[]={
  //     Jtorque = MotorMax / Motor_IPNM * Motor_revs_per_joint_rev
  82.1,  // J1 = 4860 / 2472 * 41.782
  109.5, // J2 = 4860 / 2472 * 27.836 * 2.0 (combined M2 and M3)
  65.2,  // J3 = 4860 / 2472 * 27.836 * 2.0 / 1.68 (combined M2 and M3 plus diff)
  31.2,  // J4 = 4320 / 2472 * 17.86
  12.4,  // J5 = 3900 / 6105 * 9.68 * 2.0 (combined M5 and M6)
  12.4,  // J6 = 3900 / 6105 * 9.68 * 2.0 (combined M5 and M6)
  2.2    // J7 = 3200 / 21400 * 14.962
};

// The MAX_SAFE values are the most we ever want to use for each joint          
const double Joint::MAX_SAFE_TORQ[]={                                           
  82.1, // J1
  109.5,// J2
  65.2, // J3
  31.2, // J4
  12.4, // J5
  12.4, // J6
  2.2   // J7
};                                                                              
                                                                                
// Joint torques that exceed the SAFE values but are below the MAX_CLIPPED      
// values will be clipped to the SAFE level.                                    
const double Joint::MAX_CLIPPED_TORQ[]={                                        
  // double the SAFE values                                                     
  165, // J1
  220, // J2
  130, // J3
  62,  // J4
  25,  // J5
  25,  // J6
  10   // J7
};                                                                              
                                         
//const double Joint::VEL[]={0.00, 0.75, 0.75, 0.75, 0.75, 5.00, 5.00,10.75};
//const double Joint::ACC[]={0.00, 0.75, 0.75, 0.75, 0.75, 4.50, 4.50, 7.50};

