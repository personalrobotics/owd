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
#include "SO3.hh"

so3::operator SO3() const{
  double ct, st, vt, w1, w2, w3;
  
  ct = cos(theta);    st = sin(theta);    vt = 1.0-ct;
  w1 = omega[R3::X];  w2 = omega[R3::Y];  w3 = omega[R3::Z];

  return SO3( R3(w1*w1*vt + ct,      w1*w2*vt + w3*st,   w1*w3*vt - w2*st),
	      R3(w1*w2*vt - w3*st,   w2*w2*vt + ct,      w2*w3*vt + w1*st),
	      R3(w1*w3*vt + w2*st,   w2*w3*vt - w1*st,   w3*w3*vt + ct   ));
}


SO3::operator so3() const{
  double theta = acos( (R[0][0]+R[1][1]+R[2][2] - 1.0)/2);

  if(so3::EPSILON < fabs(theta)){
    return so3(R3( (R[2][1]-R[1][2])/(2.0*sin(theta)),
		   (R[0][2]-R[2][0])/(2.0*sin(theta)),
		   (R[1][0]-R[0][1])/(2.0*sin(theta)) ), theta);
  }
  else return so3(R3(), theta); 
}

SO3 operator * (const SO3& r1, const SO3& r2){
  SO3 R;

  double* r = (double*)R;

  *r++ = (r1[SO3::R11]*r2[SO3::R11] +
	  r1[SO3::R12]*r2[SO3::R21] +
	  r1[SO3::R13]*r2[SO3::R31]);

  *r++ = (r1[SO3::R11]*r2[SO3::R12] +
	  r1[SO3::R12]*r2[SO3::R22] +
	  r1[SO3::R13]*r2[SO3::R32]);

  *r++ = (r1[SO3::R11]*r2[SO3::R13] +
	  r1[SO3::R12]*r2[SO3::R23] +
	  r1[SO3::R13]*r2[SO3::R33]);

  *r++ = (r1[SO3::R21]*r2[SO3::R11] +
	  r1[SO3::R22]*r2[SO3::R21] +
	  r1[SO3::R23]*r2[SO3::R31]);

  *r++ = (r1[SO3::R21]*r2[SO3::R12] +
	  r1[SO3::R22]*r2[SO3::R22] +
	  r1[SO3::R23]*r2[SO3::R32]);

  *r++ = (r1[SO3::R21]*r2[SO3::R13] +
	  r1[SO3::R22]*r2[SO3::R23] +
	  r1[SO3::R23]*r2[SO3::R33]);

  *r++ = (r1[SO3::R31]*r2[SO3::R11] +
	  r1[SO3::R32]*r2[SO3::R21] +
	  r1[SO3::R33]*r2[SO3::R31]);

  *r++ = (r1[SO3::R31]*r2[SO3::R12] +
	  r1[SO3::R32]*r2[SO3::R22] +
	  r1[SO3::R33]*r2[SO3::R32]);

  *r   = (r1[SO3::R31]*r2[SO3::R13] +
	  r1[SO3::R32]*r2[SO3::R23] +
	  r1[SO3::R33]*r2[SO3::R33]);

  return R;
}

ostream& operator <<  (ostream& s, const SO3& r){
  int p;
  p = cout.precision();
  cout.precision(12);
  cout.setf(ios::fixed, ios::floatfield);
  cout.setf(ios::right, ios::adjustfield);
  cout << setw(17) << r[SO3::R11] 
       << setw(17) << r[SO3::R12] 
       << setw(17) << r[SO3::R13] << endl
       << setw(17) << r[SO3::R21] 
       << setw(17) << r[SO3::R22] 
       << setw(17) << r[SO3::R23] << endl
       << setw(17) << r[SO3::R31] 
       << setw(17) << r[SO3::R32] 
       << setw(17) << r[SO3::R33];
  cout.setf(ios::left, ios::adjustfield);
  cout.precision(p);
  return s;
}

