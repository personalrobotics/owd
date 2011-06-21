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
#include <iomanip>
#include <math.h>

#ifndef __R3_HH__
#define __R3_HH__

using namespace std;

class R3{
public:
  double x[3];


  static const int X = 0;  static const int Y = 1;  static const int Z = 2;

  R3(){x[0]=x[1]=x[2]=0.0;}
  R3(double x0, double x1, double x2){x[0]=x0;  x[1]=x1;  x[2]=x2;}

  double norm() const {return sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);}

  void normalize() {
    double d=norm();
    x[0] /= d;
    x[1] /= d;
    x[2] /= d;
  }

  operator       double* ()        {return x;}
  operator const double* ()  const {return x;}
  
  double operator [] (int i) const {return x[i];}
  
  /// \brief Scale by a double
  inline R3 &operator *= (const double d) {
    x[0]*=d; x[1]*=d; x[2]*=d;
    return *this;
  }

  /// \brief Subtract another R3 in place
  inline R3 &operator -=(const R3 &rhs) {
    x[0] -= rhs.x[0];
    x[1] -= rhs.x[1];
    x[2] -= rhs.x[2];
    return *this;
  }

  friend R3     operator * (double s, const R3& r)
  {return R3(r[0]*s, r[1]*s, r[2]*s);}
  friend R3     operator * (const R3& r, double s) {return s*r;}
  friend R3     operator / (const R3& r, double s) {return (1.0/s)*r;}
  
  friend R3     operator + (const R3& r1, const R3& r2)
  {return R3(r1[0]+r2[0], r1[1]+r2[1], r1[2]+r2[2]);}
  
  R3 &operator += (const R3 &rhs) {
    x[0] += rhs.x[0];
    x[1] += rhs.x[1];
    x[2] += rhs.x[2];
    return *this;
  }

  friend R3     operator - (const R3& r1, const R3& r2)
  {return R3(r1[0]-r2[0], r1[1]-r2[1], r1[2]-r2[2]);}
  // cross product
  friend R3     operator ^ (const R3& r1, const R3& r2)
  {return R3(r1[1]*r2[2] - r1[2]*r2[1], 
	     r1[2]*r2[0] - r1[0]*r2[2], 
	     r1[0]*r2[1] - r1[1]*r2[0]);}
  
  friend double operator * (const R3& r1, const R3& r2)
  {return r1[0]*r2[0] + r1[1]*r2[1] + r1[2]*r2[2];}

  friend ostream& operator <<  (ostream& s, const R3& r){
    int p;
    p = cout.precision();
    cout.precision(12);
    cout.setf(ios::fixed, ios::floatfield);
    cout << r[0] << " " << r[1] << " " << r[2];
    cout.precision(p);
    return s;
  }
};


#endif
