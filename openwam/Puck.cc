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

#include "Puck.hh"

// This is kind of lame but I don't know any other way of initializing
// a static const array.
// Putting this in the header file will generate multiple definition errors
// from the linker
const int Puck::MIN_TRQ[]={0,-4860, -4860, -4860, -4320, -3900, -3900, -1370};
const int Puck::MAX_TRQ[]={0, 4860,  4860,  4860,  4320,  3900,  3900,  1370};

