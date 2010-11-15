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

// This is kind of lame but I don't know any other way of initializing
// a static const array.
// Putting this in the header file will generate multiple definition errors
// from the linker
//const double Joint::MIN_POS[]={0.00,-2.62,-1.80,-2.60,-0.70,-4.30,-1.25,-2.50};
//const double Joint::MAX_POS[]={0.00, 2.62, 1.80, 2.60, 3.10, 1.00, 1.25, 2.50};

//const double Joint::MIN_TRQ[]={0.0,-20.0,-30.0,-30.0,-20.0,-10.0,-10.0,-10.0};
//const double Joint::MAX_TRQ[]={0.00,20.0, 30.0, 30.0, 20.0, 10.0, 10.0, 10.0};

//const double Joint::VEL[]={0.00, 0.75, 0.75, 0.75, 0.75, 5.00, 5.00,10.75};
//const double Joint::ACC[]={0.00, 0.75, 0.75, 0.75, 0.75, 4.50, 4.50, 7.50};

