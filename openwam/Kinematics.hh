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

#include "Link.hh"

#ifndef __KINEMATICS_HH__
#define __KINEMATICS_HH__

SE3 forward_kinematics(Link* link);

// body Jacobian for Fortran (column major)
void JacobianNF(double J[][6], Link *links);

// body Jacobian (column major) but hopefully this one works
void JacobianDB(double J[][6], Link *links);

// body Jacobian for C (row major) 
void JacobianN(double J[][7], Link *links);

#endif
