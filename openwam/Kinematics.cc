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

#include "Kinematics.hh"

SE3 forward_kinematics(Link* link){
  SE3 H = (SE3)link[Link::L0];

  for(int l=Link::L1; l<=Link::Ln; l++)
    H = H * (SE3)link[l];

  return H;
}

/*
 * Body manipulator Jacobian
 * Paul IEEE SMC 11(6) 1981
 * 
 * BIG FAT WARNING: The jacobian is in column major (for Fortran)
 */

void JacobianNF(double J[][6], Link *links){
  SE3 U;

  for(int l=Link::Ln; Link::L1<=l; l--){
    U = ((SE3)links[l]) * U;

    J[l-1][0] = U[SE3::TX]*U[SE3::NY] - U[SE3::TY]*U[SE3::NX];
    J[l-1][1] = U[SE3::TX]*U[SE3::OY] - U[SE3::TY]*U[SE3::OX];
    J[l-1][2] = U[SE3::TX]*U[SE3::AY] - U[SE3::TY]*U[SE3::AX];
    J[l-1][3] = U[SE3::NZ];
    J[l-1][4] = U[SE3::OZ];
    J[l-1][5] = U[SE3::AZ];
  }
}


void JacobianDB(double J[][6], Link *links){
    SE3 U,EE;
    R3 v,axis,anchor,tEE;

    /*
    for(int l=Link::L1; l <= Link::Ln; l++)
    {        
        EE = (((SE3)links[l])^-1) * EE;

    }*/

    EE = forward_kinematics(links);
    tEE = R3(EE[3],EE[7],EE[11]);

    U = (SE3)links[Link::L0];
    for(int l=Link::L1; l <= Link::Ln; l++)
    {

        axis = R3(U[2],U[6],U[10]);
        anchor = R3(U[3],U[7],U[11]);
        
        v = axis^(anchor - tEE);

        J[l-1][0] = v[0];
        J[l-1][1] = v[1];
        J[l-1][2] = v[2];
        J[l-1][3] = 0;
        J[l-1][4] = 0;
        J[l-1][5] = 0;

        U = U * (SE3)links[l];

    }
}



void JacobianN(double J[][7], Link *links){
  SE3 U;

  for(int l=Link::Ln; Link::L1<=l; l--){
    U = ((SE3)links[l]) * U;

    J[0][l-1] = U[SE3::TX]*U[SE3::NY] - U[SE3::TY]*U[SE3::NX];
    J[1][l-1] = U[SE3::TX]*U[SE3::OY] - U[SE3::TY]*U[SE3::OX];
    J[2][l-1] = U[SE3::TX]*U[SE3::AY] - U[SE3::TY]*U[SE3::AX];
    J[3][l-1] = U[SE3::NZ];
    J[4][l-1] = U[SE3::OZ];
    J[5][l-1] = U[SE3::AZ];
  }
}
