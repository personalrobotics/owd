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

#ifndef _GLOBALS_H_
#define _GLOBALS_H_

# define TWOPI 6.283185

# define OW_SUCCESS 0
# define OW_FAILURE -1

inline double dthrow(const char *message) {
  throw message;
  return 0;
}

#define CLIP(val, min, max) \
                (isnan(val) ? dthrow("NAN passed to CLIP") \
              :  isinf(val) ? dthrow("INF passed to CLIP") \
	      :  (val<min) ? min \
	      :  (max<val) ? max \
              : val )

#define IS_IN_RANGE(val, min, max) \
               (isnan(val) ? dthrow("NAN passed to IS_IN_RANGE") \
	      : isinf(val) ? dthrow("INF passed to CLIP") \
			   : (min < val) && (val < max))

#endif // GLOBALS_H




